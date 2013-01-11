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
    /** Provide a default empty pickling infrastructure
     */
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

    /** If you have your own Db to deal with, just call that function to make it visible from Python
     * You can override some Python definition after calling it
     * @param object_db_name the name of your DB
     * @param constructor the constructor of your DB
     */
    template<typename Constructor>
    void
    wrap_object_db(const std::string &object_db_name, Constructor constructor)
    {
      bp::class_<ObjectDb, boost::noncopyable> ObjectDbClass(object_db_name.c_str(), bp::no_init);
      ObjectDbClass.def("__init__", bp::make_constructor(constructor));
      ObjectDbClass.def("parameters", &ObjectDb::parameters,
                        boost::python::return_value_policy<boost::python::copy_const_reference>());
      ObjectDbClass.def_pickle(object_db_pickle_suite());
    }
  }
}
