/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ecto/ecto.hpp>

#include <object_recognition_core/common/json.hpp>
#include <object_recognition_core/common/json_spirit/json_spirit_reader_template.h>
#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/model_utils.h>
#include <object_recognition_core/db/view.h>

namespace object_recognition_core
{
  namespace db
  {
    /**
     * \brief A runtime ModelWriter, this takes in a db::Document and saves it to the DB,
     * appending meta info to it.
     *
     * TODO Add date info to the model.
     */
    struct ModelWriter
    {
      typedef ModelWriter C;

      static void
      declare_params(ecto::tendrils& params)
      {
        params.declare(&C::model_method_, "method", //
                       "The method used to compute the model (e.g. 'TOD' ...).").required(true);
        params.declare(&C::model_parameters_, "json_params", //
                       "The non-discriminative parameters used, as JSON.").required(true);
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        inputs.declare(&C::db_document_, "db_document");
        inputs.declare(&C::json_db_, "json_db", "The DB parameters", "{}").required(
            true);
        inputs.declare(&C::object_id_, "object_id",
                       "The object id, to associate this model with.").required(true);
      }

      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        db_ = ObjectDbParameters(*json_db_).generateDb();

        Document doc_new = *db_document_;
        PopulateModel(db_, *object_id_, *model_method_, *model_parameters_, doc_new);

        // Find all the models of that type for that object
        View view(View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE);
        view.Initialize(*model_method_);
        view.set_key(*object_id_);
        ViewIterator view_iterator(view, db_);

        ViewIterator iter = view_iterator.begin(), end = view_iterator.end();
        for (; iter != end; ++iter)
        {
          // If they are the same, delete the current model in the database
          // TODO only delete if the parameters are the same, or have an option to delete all models of a method
          DocumentId model_id = (*iter).id();
          std::cout << "Deleting the previous model " << model_id << " of object " << *object_id_ << std::endl;
          db_->Delete(model_id);
        }
        doc_new.Persist();
        return ecto::OK;
      }
    private:
      ObjectDbPtr db_;
      ecto::spore<std::string> json_db_;
      ecto::spore<DocumentId> object_id_;
      ecto::spore<std::string> model_parameters_, model_method_;
      ecto::spore<Document> db_document_;
    };
  }
}

ECTO_CELL(db, object_recognition_core::db::ModelWriter, "ModelWriter",
          "Takes a document, that should be considered as a Model, and persists it."
          " Also stores common meta data that is useful for searching.")
