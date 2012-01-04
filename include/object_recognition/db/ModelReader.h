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
#include "object_recognition/db/model_utils.h"
#include "object_recognition/db/view.h"

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
          params.declare(&C::model_documents_, "model_documents", "A set of DbDocuments, one for each model to load.").required(
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
          model_documents_.set_callback(boost::bind(&T::ParameterCallback, this, _1));
          model_documents_.dirty(true);

          T::configure(params, inputs, outputs);
        }

        int
        process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
        {
          return T::process(inputs, outputs);
        }
      private:
        /** The DB documents for the models */
        ecto::spore<Documents> model_documents_;
      };
    }
  }
}
