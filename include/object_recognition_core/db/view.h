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

#ifndef VIEW_TYPES_H_
#define VIEW_TYPES_H_

#include <object_recognition_core/common/json_spirit/json_spirit.h>
#include <object_recognition_core/common/types.h>

namespace object_recognition_core
{
  namespace db
  {
    // Forward declare the base class
    class ObjectDb;
    class Document;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Contains the different values return when doing a query as mentioned here:
     * http://wiki.apache.org/couchdb/Introduction_to_CouchDB_views
     */
    class ViewElement
    {
    public:
      ViewElement(const std::string &id, const or_json::mValue & key, const or_json::mValue & value)
          :
            id_(id),
            key_(key),
            value_(value)
      {
      }

      bool
      operator==(const ViewElement & view_element) const
      {
        return id_ == view_element.id_;
      }

      std::string id_;
      or_json::mValue key_;
      or_json::mValue value_;
    };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** A view can be of different type, and for each, the Initialize function needs to be called with the right
     * arguments
     * VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE: Initialize(std::string model_type)
     */
    class View
    {
    public:
      typedef or_json::mValue Key;
      typedef or_json::mValue Value;

      /** The list of possible Views
       * When updating it, make sure to update AllViewTypes
       */
      enum ViewType
      {
        VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE
      //!< VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE
      };

      View(ViewType type)
      {
        type_ = type;
      }

      void
      Initialize(const std::string & arg1)
      {
        switch (type_)
        {
          case VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE:
            parameters_["model_type"] = or_json::mValue(arg1);
            break;
          default:
            throw std::runtime_error("Not a valid View type for initialization arguments: std::string");
            break;
        }
      }

      /** Given a document, returns whether it is in the view, and if so, returns the key and value
       * @param document
       * @param key
       * @param value
       * @return
       */
      bool
      GetKey(const or_json::mObject & document, Key & key, Value & value);

      /** Set the results to have one specific key
       * @param key
       */
      void
      set_key(const Key & key);

      /** Set the view to return all results, no matter the key
       */
      void
      unset_key();

      /** Return the key to which the view is set
       * @param key
       * @return true if the key is set, false otherwise
       */
      bool
      key(Key & key) const
      {
        key = key_;
        return is_key_set_;
      }

      static std::vector<ViewType>
      AllViewTypes()
      {
        View::ViewType all_views[] =
        { View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE };
        return std::vector<View::ViewType>(all_views, all_views + 1);
      }

      ViewType
      type() const
      {
        return type_;
      }

      const or_json::mObject &
      parameters() const
      {
        return parameters_;
      }
    private:
      ViewType type_;
      or_json::mObject parameters_;
      bool is_key_set_;
      or_json::mValue key_;
    };
  }
}
#endif /* VIEW_TYPES_H_ */
