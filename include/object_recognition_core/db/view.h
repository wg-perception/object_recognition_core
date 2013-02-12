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

#include <boost/noncopyable.hpp>

#ifdef CV_MAJOR_VERSION
#include <opencv2/core/core.hpp>
#endif

#include <object_recognition_core/common/json_spirit/json_spirit.h>
#include <object_recognition_core/common/types.h>

namespace object_recognition_core
{
  namespace db
  {
    /** A dummy document just contains fields and attachments
     */
    class DummyDocument
    {
    public:
      DummyDocument()
      {
      }

      virtual
      ~DummyDocument()
      {
      }

      /** Extract a specific attachment from a document in the DB
       * @param attachment_name
       * @param value
       */
      template<typename T>
      void
      get_attachment(const AttachmentName &attachment_name, T & value) const;

      /** Extract the stream of a specific attachment for a Document from the DB
       * @param attachment_name the name of the attachment
       * @param stream the string of data to write to
       * @param mime_type the MIME type as stored in the DB
       */
      virtual void
      get_attachment_stream(const AttachmentName &attachment_name, std::ostream& stream, MimeType mime_type =
          MIME_TYPE_DEFAULT) const = 0;

      /** Add a specific field to a Document (that has been pre-loaded or not)
       * @param attachment_name the name of the attachment
       * @param value the attachment itself, that needs to be boost serializable
       */
      template<typename T>
      void
      set_attachment(const AttachmentName &attachment_name, const T & value);

      /** Add a stream attachment to a a Document
       * @param attachment_name the name of the stream
       * @param stream the stream itself
       * @param mime_type the MIME type of the stream
       */
      void
      set_attachment_stream(const AttachmentName &attachment_name, const std::istream& stream,
                            const MimeType& mime_type = MIME_TYPE_DEFAULT);

      /** Get a specific value */
      template<typename T>
      T
      get_value(const std::string& key) const
      {
        or_json::mObject::const_iterator iter = fields_.find(key);
        if (iter != fields_.end())
          return iter->second.get_value<T>();
        else
          throw std::runtime_error("\"" + key + "\" not a valid key for the JSON tree: " + or_json::write(fields_));
      }

      /** Get a specific value */
      or_json::mValue
      get_value(const std::string& key) const
      {
        or_json::mObject::const_iterator iter = fields_.find(key);
        if (iter != fields_.end())
          return iter->second;
        else
          throw std::runtime_error("\"" + key + "\" not a valid key for the JSON tree: " + or_json::write(fields_));
      }

      /** Get a specific value */
      const or_json::mObject &
      fields() const
      {
        return fields_;
      }

      /** Set a specific value */
      template<typename T>
      void
      set_value(const std::string& key, const T& val)
      {
        fields_[key] = or_json::mValue(val);
      }

      /** Set several values by inserting a property tree */
      void
      set_values(const or_json::mObject & json_tree)
      {
        fields_.insert(json_tree.begin(), json_tree.end());
      }

      /** Set several values by inserting a property tree at a specific key*/
      void
      set_values(const std::string& key, const or_json::mObject & json_tree)
      {
        or_json::mObject::const_iterator iter = fields_.find(key);
        if (iter == fields_.end())
          fields_.insert(std::make_pair(key, json_tree));
        else
          iter->second.get_value<or_json::mObject>().insert(json_tree.begin(), json_tree.end());
      }

      /** Clear all the fields, there are no fields left after */
      void
      ClearAllFields();

      /** Remove a specific field */
      void
      ClearField(const std::string& key);

    protected:
      /** contains the attachments: binary blobs */
      struct StreamAttachment: boost::noncopyable
      {
        StreamAttachment()
        {
        }

        StreamAttachment(const MimeType &type)
            :
              type_(type)
        {
        }

        StreamAttachment(const MimeType &type, const std::istream &stream)
            :
              type_(type)
        {
          copy_from(stream);
        }
        void
        copy_from(const std::istream& stream)
        {
          stream_ << stream.rdbuf();
          stream_.seekg(0);
        }
        MimeType type_;
        std::stringstream stream_;
        typedef boost::shared_ptr<StreamAttachment> ptr;
      };

      typedef std::map<AttachmentName, StreamAttachment::ptr> AttachmentMap;

      /** ALl the attachments */
      AttachmentMap attachments_;

      /** contains the fields: they are of integral types */
      or_json::mObject fields_;
    };

#ifdef CV_MAJOR_VERSION
    // Specializations for cv::Mat
    template<>
    void
    DummyDocument::get_attachment<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value) const;

    template<>
    void
    DummyDocument::set_attachment<cv::Mat>(const AttachmentName &attachment_name, const cv::Mat & value);
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Contains the different values return when doing a query as mentioned here:
     * http://wiki.apache.org/couchdb/Introduction_to_CouchDB_views
     */
    class ViewElement: public DummyDocument
    {
    public:
      ViewElement(const std::string &id, const or_json::mValue & key)
          :
            id_(id),
            key_(key)
      {
      }

      ~ViewElement();

      bool
      operator==(const ViewElement & view_element) const
      {
        return id_ == view_element.id_;
      }

      virtual void
      get_attachment_stream(const AttachmentName &attachment_name, std::ostream& stream, MimeType mime_type =
          MIME_TYPE_DEFAULT) const;

      /** contains the fields: they are of integral types */
      std::string id_;
      or_json::mValue key_;
    };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** A view can be of different type, and for each, the Initialize function needs to be called with the right
     * arguments
     * VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE: Initialize(std::string model_type)
     * VIEW_OBJECT_INFO_WHERE_OBJECT_ID: no need to initialize. Returns a document that contains the
     *                           following attributes: name (std:string), mesh_uri (std::string)
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
        VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE, VIEW_OBJECT_INFO_WHERE_OBJECT_ID
      };

      View(ViewType type):
        is_key_set_(false)
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
        { View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE, View::VIEW_OBJECT_INFO_WHERE_OBJECT_ID };
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
