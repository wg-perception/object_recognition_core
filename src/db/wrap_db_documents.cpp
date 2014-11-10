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
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/shared_ptr.hpp>

#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/model_utils.h>

namespace bp = boost::python;

namespace object_recognition_core
{
  namespace db
  {
    typedef boost::shared_ptr<Document> DocumentPtr;
    typedef boost::shared_ptr<Documents> DocumentsPtr;

    /** Function used to create a vector of db Document's from Python
     * @param db_params
     * @param python_document_ids
     * @return
     */
    DocumentsPtr
    DocumentsConstructor(object_recognition_core::db::ObjectDbPtr & db, const bp::object & python_document_ids)
    {
      bp::list ids_list = bp::extract<bp::list>(python_document_ids);
      size_t ids_nbr = bp::len(ids_list);

      // Create the Documents from the ids
      DocumentsPtr p(new Documents());
      p->reserve(ids_nbr);

      for(size_t i = 0; i < ids_nbr; ++i) {
        std::string object_classname = boost::python::extract<std::string>(ids_list[i].attr("__class__").attr("__name__"));
        DocumentId document_id;
        if (object_classname == "str")
          document_id = bp::extract<std::string>(ids_list[i]);
        else
          document_id = bp::extract<std::string>(bp::str(ids_list[i]).encode("utf-8"));

        Document doc;
        doc.set_db(db);
        doc.set_document_id(document_id);
        doc.load_fields();
        p->push_back(doc);
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
      bp::class_<Document, DocumentPtr> DocumentClass("Document");
      DocumentClass.def(bp::init<>()).def(bp::init<Document>());
      DocumentClass.def("id", &Document::id, bp::return_value_policy<bp::return_by_value>());

      bp::class_<Documents, DocumentsPtr> DocumentsClass("Documents");
      DocumentsClass.def("__init__", bp::make_constructor(DocumentsConstructor));
      DocumentsClass.def(boost::python::vector_indexing_suite<Documents>());
      DocumentsClass.def("size", &Documents::size);
      DocumentsClass.def_pickle(db_documents_pickle_suite());
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    DocumentsPtr
    ModelDocumentsFromPython(object_recognition_core::db::ObjectDbPtr & db, const bp::object & bp_object_ids,
                             const std::string & method)
    {
      DocumentsPtr p(new Documents());

      std::vector<ObjectId> object_ids;
      {
        boost::python::stl_input_iterator<ObjectId> object_begin(bp_object_ids), end;
        std::copy(object_begin, end, std::back_inserter(object_ids));
      }

      if (!object_ids.empty())
      {
        *p = ModelDocuments(db, object_ids, method);
      }
      return p;
    }

    // Models are Documents that are models
    void
    wrap_db_models()
    {
      boost::python::def("Models", ModelDocumentsFromPython);
    }
  }
}
