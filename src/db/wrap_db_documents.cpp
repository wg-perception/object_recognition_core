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
#include <string>

#include <boost/function.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/shared_ptr.hpp>
#include <object_recognition/db/db.h>

#include "object_recognition/db/model_utils.h"

namespace bp = boost::python;

namespace object_recognition
{
  namespace db
  {
    typedef boost::shared_ptr<Documents> DocumentsPtr;

    /** Function used to create a vector of db Document's from Python
     * @param db_params
     * @param python_document_ids
     * @return
     */
    DocumentsPtr
    DocumentsConstructor(const ObjectDbParameters & db_params, const bp::object & python_document_ids)
    {
      // Read the document ids from the input
      std::vector<DocumentId> document_ids;
      boost::python::stl_input_iterator<std::string> begin(python_document_ids), end;
      std::copy(begin, end, std::back_inserter(document_ids));

      // Create the Documents from the ids
      DocumentsPtr p(new Documents());
      p->reserve(document_ids.size());
      object_recognition::db::ObjectDb db(db_params);

      BOOST_FOREACH(const DocumentId & document_id, document_ids)
          {
            p->push_back(Document(db, db_params.collection_, document_id));
          }

      return p;
    }

    // Define the pickling of the object
    struct db_documents_pickle_suite: boost::python::pickle_suite
    {
      static boost::python::tuple
      getinitargs(const ObjectDbParameters& db_params)
      {
        return boost::python::make_tuple();
      }

      static boost::python::tuple
      getstate(const ObjectDbParameters& db_params)
      {
        // TODO
        return boost::python::make_tuple();
      }

      static
      void
      setstate(ObjectDbParameters& db_params, boost::python::tuple state)
      {
        using namespace boost::python;
        // TODO
      }
    };

    void
    wrap_db_documents()
    {
      bp::class_<Documents, DocumentsPtr> DocumentsClass("DbDocuments");
      DocumentsClass.def("__init__", bp::make_constructor(DocumentsConstructor));
      DocumentsClass.def("size", &Documents::size);
      DocumentsClass.def_pickle(db_documents_pickle_suite());
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    DocumentsPtr
    ModelDocumentsFromPython(const ObjectDbParameters & db_params, const std::string & collection_name,
                             const bp::object & bp_object_ids, const bp::object & bp_model_ids,
                             const std::string & model_json_params)
    {
      ObjectDb db;
      db.set_params(db_params);

      std::vector<ObjectId> object_ids;
      std::vector<ModelId> model_ids;
      {
        boost::python::stl_input_iterator<ObjectId> object_begin(bp_object_ids), model_begin(bp_model_ids), end;
        std::copy(object_begin, end, std::back_inserter(object_ids));
        std::copy(model_begin, end, std::back_inserter(model_ids));
      }
      DocumentsPtr p(new Documents());
      *p = ModelDocuments(db, collection_name, object_ids, model_ids, model_json_params);
      return p;
    }

    // DbModels are DbDocuments that are models
    void
    wrap_db_models()
    {
      boost::python::def("DbModels", ModelDocumentsFromPython);
    }
  }
}
