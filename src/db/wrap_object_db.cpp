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
#include <boost/python/return_value_policy.hpp>
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
    typedef boost::shared_ptr<ObjectDb> ObjectDbPtr;

    /** Function used to create an ObjectDb object from Python
     * @param db_params
     * @param python_document_ids
     * @return
     */
    ObjectDbPtr
    ObjectDbConstructor(const ObjectDbParameters & db_params)
    {
      // Create the ObjectDb from the ids
      ObjectDbPtr p(new ObjectDb(db_params));
      return p;
    }

    // Define the pickling of the object
    struct object_db_pickle_suite: boost::python::pickle_suite
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
    wrap_object_db()
    {
      bp::class_ < ObjectDb > ("ObjectDb").def(bp::init<>()).def(bp::init<ObjectDb>());

      bp::class_<ObjectDb, ObjectDbPtr> ObjectDbClass("ObjectDb");
      ObjectDbClass.def("__init__", bp::make_constructor(ObjectDbConstructor));
      ObjectDbClass.def("parameters", &ObjectDb::parameters,
                        boost::python::return_value_policy<boost::python::copy_const_reference>());
      ObjectDbClass.def_pickle(object_db_pickle_suite());
    }
  }
}
