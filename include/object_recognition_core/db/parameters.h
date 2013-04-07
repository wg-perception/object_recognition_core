/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef ORK_CORE_DB_PARAMETERS_H_
#define ORK_CORE_DB_PARAMETERS_H_

#include <object_recognition_core/common/json.hpp>

namespace object_recognition_core {
namespace db {

typedef or_json::mObject ObjectDbParametersRaw;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class ObjectDb;

    /** A class that stores the common parameters for the object DB
     * If it is not from a type provided by object_recognition_core, it is of type NONCORE
     */
    class ObjectDbParameters
    {
    public:
      enum ObjectDbType
      {
        EMPTY, COUCHDB, FILESYSTEM, NONCORE
      };
      ObjectDbParameters();

      /** Default constructor for certain types
       * @param type Default type
       */
      explicit
      ObjectDbParameters(const std::string& type);

      /** Default constructor for certain types
       * @param type Default type
       */
      explicit
      ObjectDbParameters(ObjectDbType type);

      /**
       * @param params A map between some db parameters and their value
       */
      explicit
      ObjectDbParameters(const ObjectDbParametersRaw& params);

      static ObjectDbType
      StringToType(const std::string & type);

      static std::string
      TypeToString(const ObjectDbParameters::ObjectDbType & type);

      inline ObjectDbType
      type() const
      {
        return type_;
      }

      template<typename T>
      void
      set_parameter(const std::string& key, const T& value)
      {
        if (key == "type")
          set_type(value);
        else
        {
          if ((type() != NONCORE) && (raw_.find(key) == raw_.end()))
            throw std::runtime_error("Key \"" + key + "\" not a default key in db of type " + TypeToString(type()));

          raw_[key] = or_json::mValue(value);
        }
      }

      void
      set_parameter(const std::string& key, const or_json::mValue& value)
      {
        if (key == "type")
          set_type(value.get_str());
        else
        {
          if ((type() != NONCORE) && (raw_.find(key) == raw_.end()))
            throw std::runtime_error("Key \"" + key + "\" not a default key in db of type " + TypeToString(type()));

          raw_[key] = value;
        }
      }

      void
      set_type(const std::string & type);

      void
      set_type(const ObjectDbType & type)
      {
        if (type == NONCORE)
        {
          if (type != type_)
            raw_.clear();
          type_ = type;
        }
        else
          set_type(TypeToString(type));
      }

      or_json::mValue
      at(const std::string & key) const
      {
        return raw_.at(key);
      }

      const or_json::mObject &
      raw() const
      {
        return raw_;
      }

      boost::shared_ptr<ObjectDb>
      generateDb() const;
    protected:
      /** The type of the collection 'CouchDB' ... */
      ObjectDbType type_;
      /** All the raw parameters: they are of integral types. 'type' is there */
      or_json::mObject raw_;
    };
  }
}

#endif /* ORK_CORE_DB_PARAMETERS_H_ */

