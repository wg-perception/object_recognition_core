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
#include <map>
#include <string>

#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>

#include <object_recognition_core/common/dict_json_conversion.h>
#include <object_recognition_core/db/parameters.h>

namespace bp = boost::python;

namespace object_recognition_core
{
  namespace db
  {
    typedef boost::shared_ptr<ObjectDbParameters> ObjectDbParametersPtr;

    /** Constructor for the ObjectDbParameters class from a python dictionary
     * @param obj
     * @return
     */
    boost::shared_ptr<ObjectDbParameters>
    ObjectDbParametersConstructorDict(const bp::dict &obj)
    {
      or_json::mObject params = common::BpDictToJson(obj);
      if (params.empty())
        params.insert(std::make_pair("type", ObjectDbParameters::TypeToString(ObjectDbParameters::EMPTY)));
      ObjectDbParametersPtr p(new ObjectDbParameters(params));
      return p;
    }

    /** Another constructor for the ObjectDbParameters class from a JSON string
     * @param str the JSON string to parse parameters from
     * @return
     */
    boost::shared_ptr<ObjectDbParameters>
    ObjectDbParametersConstructorStr(const std::string &str)
    {
      ObjectDbParametersPtr p(new ObjectDbParameters(str));
      return p;
    }

    /** Define the pickling of the object
     */
    struct db_parameters_pickle_suite: boost::python::pickle_suite
    {
      static boost::python::tuple
      getinitargs(const ObjectDbParameters& db_params)
      {
        return boost::python::make_tuple();
      }

      static boost::python::tuple
      getstate(const ObjectDbParameters& db_params)
      {
        return boost::python::make_tuple(common::JsonToBpDict(db_params.raw()));
      }

      static
      void
      setstate(ObjectDbParameters& db_params, boost::python::tuple state)
      {
        using namespace boost::python;
        if (len(state) != 1)
        {
          PyErr_SetObject(PyExc_ValueError, ("expected 1-item tuple in call to __setstate__; got %s" % state).ptr());
          throw_error_already_set();
        }

        db_params = ObjectDbParameters(common::BpDictToJson(extract<bp::dict>(state[3])));
      }
    };

    ObjectDbParameters::ObjectDbType
    type(const ObjectDbParametersPtr &params)
    {
      return params->type();
    }

    bp::dict
    raw(const ObjectDbParametersPtr &params)
    {
      return common::JsonToBpDict(params->raw());
    }

    void
    wrap_db_parameters()
    {
      bp::class_<ObjectDbParameters, ObjectDbParametersPtr> ObjectDbParametersClass("ObjectDbParameters"); //"The parameters of any database");
      ObjectDbParametersClass.def("__init__", bp::make_constructor(ObjectDbParametersConstructorDict));
      ObjectDbParametersClass.def("__init__", bp::make_constructor(ObjectDbParametersConstructorStr));
      ObjectDbParametersClass.add_property("type", type, "The type of the database.");
      ObjectDbParametersClass.add_property("raw", raw, "The raw parameters of the database.");
      ObjectDbParametersClass.def_pickle(db_parameters_pickle_suite());
      bp::enum_<ObjectDbParameters::ObjectDbType>("ObjectDbTypes").value("COUCHDB", ObjectDbParameters::COUCHDB).value(
          "EMPTY", ObjectDbParameters::EMPTY).value("FILESYSTEM", ObjectDbParameters::FILESYSTEM).value(
          "NONCORE", ObjectDbParameters::NONCORE);
    }
  }
}
