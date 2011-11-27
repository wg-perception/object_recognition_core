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
#include <object_recognition/db/db.h>

namespace bp = boost::python;

namespace
{
  std::map<std::string, std::string>
  BpDictToMap(const bp::dict &bp_dict)
  {
    std::map<std::string, std::string> params;
    bp::list l = bp_dict.items();
    for (int j = 0, end = bp::len(l); j < end; ++j)
    {
      std::string key = bp::extract<std::string>(l[j][0]);
      std::string value = bp::extract<std::string>(l[j][1]);
      params[key] = value;
    }
    return params;
  }

  bp::dict
  MapToBpDict(const std::map<std::string, std::string> & map)
  {
    bp::dict bp_dict;
    for (std::map<std::string, std::string>::const_iterator iter = map.begin(), end = map.end(); iter != end; ++iter)
    {
      std::cerr << iter->second;
      std::cerr << iter->first << " " << iter->second << std::endl;
      bp_dict[iter->first] = iter->second;
    }
    return bp_dict;
  }
}

namespace object_recognition
{
  namespace db
  {
    typedef boost::shared_ptr<ObjectDbParameters> ObjectDbParametersPtr;

    /** Constructor for the ObjectDbParameters class from a python dictionary
     * @param obj
     * @return
     */
    boost::shared_ptr<ObjectDbParameters>
    ObjectDbParametersConstructor(const bp::dict &obj)
    {
      std::map<std::string, std::string> params = BpDictToMap(bp::dict(obj));
      if (params.empty())
        params.insert(std::make_pair("type", ObjectDbParameters::TypeToString(ObjectDbParameters::EMPTY)));
      ObjectDbParametersPtr p(new ObjectDbParameters(params));
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
        return boost::python::make_tuple(ObjectDbParameters::TypeToString(db_params.type_), db_params.root_,
                                         db_params.collection_, MapToBpDict(db_params.all_parameters_));
      }

      static
      void
      setstate(ObjectDbParameters& db_params, boost::python::tuple state)
      {
        using namespace boost::python;
        if (len(state) != 4)
        {
          PyErr_SetObject(PyExc_ValueError, ("expected 4-item tuple in call to __setstate__; got %s" % state).ptr());
          throw_error_already_set();
        }

        db_params.type_ = ObjectDbParameters::StringToType(extract<std::string>(state[0]));
        db_params.root_ = extract<std::string>(state[1]);
        db_params.collection_ = extract<std::string>(state[2]);
        db_params.all_parameters_ = BpDictToMap(extract<bp::dict>(state[3]));
      }
    };

    // Define some fucntions to access the members
    std::string
    collection(const ObjectDbParametersPtr &params)
    {
      return params->collection_;
    }

    std::string
    root(const ObjectDbParametersPtr &params)
    {
      return params->root_;
    }

    std::string
    type(const ObjectDbParametersPtr &params)
    {
      return ObjectDbParameters::TypeToString(params->type_);
    }

    void
    wrap_db_parameters()
    {
      bp::class_<ObjectDbParameters, ObjectDbParametersPtr> ObjectDbParametersClass("ObjectDbParameters"); //"The parameters of any database");
      ObjectDbParametersClass.def("__init__", bp::make_constructor(ObjectDbParametersConstructor));
      ObjectDbParametersClass.add_property("collection", collection, "The collection of the database.");
      ObjectDbParametersClass.add_property("root", root, "The root of the database.");
      ObjectDbParametersClass.add_property("type", type, "The type of the database.");
      ObjectDbParametersClass.def_pickle(db_parameters_pickle_suite());
      bp::enum_<ObjectDbParameters::ObjectDbType>("color").value("COUCHDB", ObjectDbParameters::COUCHDB).value(
          "EMPTY", ObjectDbParameters::EMPTY).value("FILESYSTEM", ObjectDbParameters::FILESYSTEM);
    }
  }
}
