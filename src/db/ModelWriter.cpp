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

#include <object_recognition/common/json_spirit/json_spirit_reader_template.h>
#include <object_recognition/common/types.h>
#include <object_recognition/common/json.hpp>

#include <object_recognition/db/db.h>
#include <object_recognition/db/model_utils.h>
#include <object_recognition/db/view_types.h>

namespace object_recognition
{
  namespace db
  {
    struct ModelWriter
    {
      typedef ModelWriter C;

      static void
      declare_params(ecto::tendrils& params)
      {
        params.declare(&C::db_params_, "db_params", //
                       "The DB parameters").required(true);
        params.declare(&C::object_id_, "object_id", //
                       "The object id, to associate this model with.").required(true);
        params.declare(&C::session_ids_, "session_ids", //
                       "The session sid, to associate this model with.").required(true);
        params.declare(&C::model_params_, "model_json_params", //
                       "The parameters used for the model, as JSON.").required(true);
        params.declare(&C::model_type_, "model_type", //
                       "The type of model.").required(true);
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        inputs.declare(&C::db_document_, "db_document");
      }

      void
      configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        db_.set_parameters(*db_params_);
      }

      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        Document doc_new = *db_document_;
        doc_new.update_db(db_);
        PopulateDoc(*object_id_, *session_ids_, *model_params_, *model_type_, doc_new);

        // Read the input model parameters
        or_json::mObject in_parameters = to_json(*model_params_);

        std::cout << "persisting " << doc_new.id() << std::endl;
        // Find all the models of that type for that object
        View view(View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE);
        view.Initialize(*object_id_, *model_type_);
        ViewIterator view_iterator(view, db_);

        ViewIterator iter = view_iterator.begin(), end = view_iterator.end();
        for (; iter != end; ++iter)
        {
          // Compare the parameters
          or_json::mObject db_parameters = (*iter).get_value<or_json::mObject>("parameters");

          // If they are the same, delete the current model in the database
          if (CompareJsonIntersection(in_parameters, db_parameters))
          {
            std::cout << "Deleting the previous model " << (*iter).id() << " of object " << *object_id_ << std::endl;
            db_.Delete((*iter).id());
            break;
          }

          doc_new.Persist();
        }
        std::cout << "done persisting " << doc_new.id() << std::endl;
        return ecto::OK;
      }
    private:
      ObjectDb db_;
      ecto::spore<ObjectDbParameters> db_params_;
      ecto::spore<DocumentId> object_id_;
      ecto::spore<std::string> session_ids_, model_params_, model_type_;
      ecto::spore<Document> db_document_;

    };
  }
}
ECTO_CELL( object_recognition_db, object_recognition::db::ModelWriter, "ModelWriter",
          "Takes a document, that should be considered as a Model, and persists it."
          " Also stores common meta data that is useful for searching.");
