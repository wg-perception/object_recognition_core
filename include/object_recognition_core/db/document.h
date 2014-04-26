/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef ORK_CORE_DB_DOCUMENT_H_
#define ORK_CORE_DB_DOCUMENT_H_

#include <sstream>
#include <map>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/common/json_spirit/json_spirit.h>

namespace object_recognition_core {
namespace db {

class ObjectDb;
typedef boost::shared_ptr<ObjectDb> ObjectDbPtr;
typedef boost::shared_ptr<const ObjectDb> ObjectDbConstPtr;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

      /**
       * @param attachment_name the name of the attachment to check
       * @return true if there is such an attachment stored
       */
      bool
      has_attachment(const AttachmentName &attachment_name)
      {
        return attachments_.find(attachment_name) != attachments_.end();
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
          MIME_TYPE_DEFAULT) const;

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

      /**
       * @param key the name of the field to check
       * @return true if there is such a value stored
       */
      bool
      has_field(const std::string& key) const
      {
        return fields_.find(key) != fields_.end();
      }

      /** Get a specific value */
      template<typename T>
      T
      get_field(const std::string& key) const
      {
        or_json::mObject::const_iterator iter = fields_.find(key);
        if (iter != fields_.end())
          return iter->second.get_value<T>();
        else
          throw std::runtime_error("\"" + key + "\" not a valid key for the JSON tree: " + or_json::write(fields_));
      }

      /** Get a specific value */
      or_json::mValue
      get_field(const std::string& key) const
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

      /** Get a specific value */
      std::vector<std::string>
      attachment_names() const
      {
        std::vector<std::string> attachment_names;
        or_json::mObject::const_iterator attachment_field = fields_.find("_attachments");
        if (attachment_field == fields_.end())
          return attachment_names;
        or_json::mObject attachments = attachment_field->second.get_obj();
        for(or_json::mObject::const_iterator iter = attachments.begin(); iter != attachments.end(); ++iter)
          attachment_names.push_back(iter->first);
        return attachment_names;
      }

      /** Set a specific value */
      template<typename T>
      void
      set_field(const std::string& key, const T& val)
      {
        fields_[key] = or_json::mValue(val);
      }

      /** Set several values by inserting a property tree */
      void
      set_fields(const or_json::mObject & json_tree)
      {
        fields_.insert(json_tree.begin(), json_tree.end());
      }

      /** Set several values by inserting a property tree at a specific key*/
      void
      set_fields(const std::string& key, const or_json::mObject & json_tree)
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

      /** All the attachments */
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

    /** A Document holds fields (in the CouchDB sense) which are strings that are queryable, and attachments (that are
     * un-queryable binary blobs)
     */
    class Document: public DummyDocument
    {
    public:
      Document();
      ~Document();

      bool
      operator==(const Document & document) const
      {
        return document_id_ == document.document_id_;
      }

      /**
       * Update the db that this document should be associated with.
       * @param db
       */
      void
      set_db(const ObjectDbPtr& db);

      /**
       * Update the document_id that this document should be associated with.
       * @param document_id
       */
      void
      set_document_id(const DocumentId &document_id);

      /** Fill the fields of the object
       */
      void
      load_fields();

      /** Persist your object to a given DB
       */
      void
      Persist();

      /** Set the id and the revision number */
      void
      SetIdRev(const std::string& id, const std::string& rev);

      const std::string &
      id() const
      {
        return document_id_;
      }

      const std::string &
      rev() const
      {
        return revision_id_;
      }

      /** Extract a specific attachment from a document in the DB
       * @param attachment_name
       * @param value
       */
      template<typename T>
      void
      get_attachment_and_cache(const AttachmentName &attachment_name, T & value);

      /** Extract the stream of a specific attachment for a Document from the DB
       * @param attachment_name the name of the attachment
       * @param stream the string of data to write to
       * @param mime_type the MIME type as stored in the DB
       */
      virtual void
      get_attachment_stream(const AttachmentName &attachment_name, std::ostream& stream, MimeType mime_type =
          MIME_TYPE_DEFAULT) const;

      /** Extract the stream of a specific attachment for a Document from the DB
       * @param attachment_name the name of the attachment
       * @param stream the string of data to write to
       * @param mime_type the MIME type as stored in the DB
       */
      void
      get_attachment_stream_and_cache(const AttachmentName &attachment_name, std::ostream& stream, MimeType mime_type =
          MIME_TYPE_DEFAULT);
    private:
      ObjectDbPtr db_;
      DocumentId document_id_;
      RevisionId revision_id_;
    };

#ifdef CV_MAJOR_VERSION
    // Specializations for cv::Mat
    template<>
    void
    DummyDocument::get_attachment<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value) const;

    template<>
    void
    DummyDocument::set_attachment<cv::Mat>(const AttachmentName &attachment_name, const cv::Mat & value);

    template<>
    void
    Document::get_attachment_and_cache<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value);
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    typedef std::vector<Document> Documents;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
}

#endif /* ORK_CORE_DB_DOCUMENT_H_ */
