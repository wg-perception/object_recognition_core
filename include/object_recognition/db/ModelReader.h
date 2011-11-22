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

#pragma once

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <ecto/ecto.hpp>

#include "object_recognition/common/json_spirit/json_spirit_reader_template.h"
#include "object_recognition/common/types.h"
#include "object_recognition/db/ModelWriter.h"
#include "object_recognition/db/view_types.h"

namespace object_recognition
{
  namespace db
  {
    namespace bases
    {
      /** When creating your own cell to insert models in the DB first have an implementation class that inherits from
       * ModelReaderImpl
       */
      struct ModelReaderImpl
      {
        virtual
        ~ModelReaderImpl()
        {
        }

        virtual void
        ParameterCallback(const Documents & db_documents) = 0;

        static void
        declare_params(ecto::tendrils& params)
        {
        }

        static void
        declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
        {
        }

        virtual void
        configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
        {
        }

        virtual int
        process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
        {
          return ecto::OK;
        }
      };

      /** Class inserting the arbitrary Models into the DB. If you want to create a cell that persists to the DB, first
       * implement a ModelReaderImpl class and then have your model inserter class inherit the ModelReaderBase as
       * follows:
       * struct MyAwesomeModelReader db::bases::ModelReaderNase<MyAwesomeModelReaderImpl> {};
       *
       * You have to jump through those hoops because of the static member functions
       */
      template<typename T>
      struct ModelReaderBase: T
      {
        typedef ModelReaderBase<T> C;

        static void
        declare_params(ecto::tendrils& params)
        {
          params.declare(&C::collection_name_, "collection",
                         "std::string The collection in which to store the models on the db", //
                         "object_recognition").required(true);
          params.declare(&C::db_params_, "db_params", "The DB parameters").required(true);
          params.declare(&C::bp_object_ids_, "object_ids",
                         "The object ids to load the models of (those models will match the parameters).").required(
              true);
          params.declare(&C::bp_model_ids_, "model_ids", "The ids of the models to load (complementary to object-ids). "
                         "Ids are trustedd and not matched to the parameters.").required(true);
          params.declare(&C::model_params_, "model_json_params", "The parameters used for the model, as JSON.").required(
              true);
          T::declare_params(params);
        }

        static void
        declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
        {
          T::declare_io(params, inputs, outputs);
        }

        void
        configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
        {
          // Make sure that whenever parameters related to the models or objects changes, the list of models is regenerated
          bp_object_ids_.set_callback(boost::bind(&ModelReaderBase<T>::ParameterCallback, this));
          bp_model_ids_.set_callback(boost::bind(&ModelReaderBase<T>::ParameterCallback, this));
          model_params_.set_callback(boost::bind(&ModelReaderBase<T>::ParameterCallback, this));
          db_params_.set_callback(boost::bind(&ModelReaderBase<T>::ParameterCallback, this));
          bp_object_ids_.dirty(true);

          T::configure(params, inputs, outputs);
        }

        int
        process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
        {
          return T::process(inputs, outputs);
        }
      private:
        void
        ParameterCallback()
        {
          db_.set_params(*db_params_);
          {
            boost::python::stl_input_iterator<ObjectId> object_begin(*bp_object_ids_), model_begin(*bp_model_ids_), end;
            std::copy(object_begin, end, std::back_inserter(object_ids_));
            std::copy(model_begin, end, std::back_inserter(model_ids_));
          }

          Documents model_documents;
          model_documents.reserve(object_ids_.size() + model_ids_.size());
          // First, load all the models where their id belongs to model_ids_, blindly trusting them
          BOOST_FOREACH(const ModelId & model_id, model_ids_)
                model_documents.push_back(Document(db_, *collection_name_, model_id));

          // ext, for each object id, find the models (if any) that fit the parameters
          json_spirit::mObject in_parameters;
          {
            json_spirit::mValue value;
            json_spirit::read(*model_params_, value);
            in_parameters = value.get_obj();
          }

          BOOST_FOREACH(const ModelId & object_id, object_ids_)
              {
                View view(View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE);
                view.Initialize(object_id, *collection_name_);
                ViewIterator view_iterator(view, db_, *collection_name_);

                while (view_iterator != ViewIterator::end())
                {
                  // Compare the parameters to the input ones
                  json_spirit::mObject db_parameters;
                  {
                    json_spirit::mValue value;
                    json_spirit::read((*view_iterator).get_value<std::string>("parameters"), value);
                    db_parameters = value.get_obj();
                  }
                  if (CompareJsonIntersection(in_parameters, db_parameters))
                    model_documents.push_back(Document(db_, *collection_name_, (*view_iterator).id()));

                  ++view_iterator;
                }
              }

          T::ParameterCallback(model_documents);
        }

        ObjectDb db_;
        ecto::spore<ObjectDbParameters> db_params_;
        ecto::spore<CollectionName> collection_name_;
        ecto::spore<std::string> model_params_;

        ecto::spore<boost::python::object> bp_object_ids_;
        ecto::spore<boost::python::object> bp_model_ids_;
        std::vector<ObjectId> object_ids_;
        std::vector<ModelId> model_ids_;
        /** The DB documents for the models */
        ecto::spore<Documents> model_documents_;
      };
    }
  }
}
