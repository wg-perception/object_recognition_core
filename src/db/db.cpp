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

#include <algorithm>
#include <string>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include "db_couch.h"
#include "db_default.h"
#include "db_empty.h"
#include "db_filesystem.h"
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/opencv.h>

namespace object_recognition_core
{
  namespace db
  {
    ObjectDbParameters::ObjectDbParameters()
    {
      type_ = EMPTY;
    }

    /** Default constructor for certain types
     * @param type Default type
     */
    ObjectDbParameters::ObjectDbParameters(const std::string& json_str)
    {
      or_json::mValue value;
      or_json::read(json_str, value);
      ObjectDbParametersRaw params = value.get_obj();

      *this = ObjectDbParameters(params);
    }

    ObjectDbParameters::ObjectDbParameters(ObjectDbType type)
    {
      set_type(type);
    }

    void
    ObjectDbParameters::set_type(const std::string &type)
    {
      type_ = StringToType(type);

      if ((raw_.find("type") != raw_.end()) && (raw_["type"] == type))
        return;

      switch (type_)
      {
        case ObjectDbParameters::COUCHDB:
        {
          raw_ = object_recognition_core::db::ObjectDbDefaults<ObjectDbCouch>::default_raw_parameters();
          break;
        }
        case ObjectDbParameters::EMPTY:
        {
          raw_ = object_recognition_core::db::ObjectDbDefaults<ObjectDbEmpty>::default_raw_parameters();
          break;
        }
        case ObjectDbParameters::FILESYSTEM:
        {
          raw_ = object_recognition_core::db::ObjectDbDefaults<ObjectDbFilesystem>::default_raw_parameters();
          break;
        }
        case ObjectDbParameters::NONCORE:
        default:
        {
          // No implementation for that db type
          raw_["type"] = type;
          break;
        }
      }
    }

    ObjectDbParameters::ObjectDbParameters(const ObjectDbParametersRaw& parameters)
    {
      if (parameters.find("type") == parameters.end())
      {
        throw std::runtime_error("You must supply a database type. e.g. CouchDB");
      }
      // Set some default parameters
      set_type(parameters.at("type").get_str());
      // Fill the other parameters
      for (or_json::mObject::const_iterator iter = parameters.begin(), end = parameters.end(); iter != end; ++iter)
      {
        if (iter->first == "type")
          continue;
        set_parameter(iter->first, iter->second);
      }
    }

    ObjectDbParameters::ObjectDbType
    ObjectDbParameters::StringToType(const std::string & type_str)
    {
      std::string type_str_lower = type_str;
      std::transform(type_str.begin(), type_str.end(), type_str_lower.begin(), ::tolower);

      if (type_str_lower == "couchdb")
        return COUCHDB;
      else if (type_str_lower == "empty")
        return EMPTY;
      else if (type_str_lower == "filesystem")
        return FILESYSTEM;
      else
        return NONCORE;
    }

    std::string
    ObjectDbParameters::TypeToString(const ObjectDbParameters::ObjectDbType & type)
    {
      switch (type)
      {
        case COUCHDB:
          return "CouchDB";
        case EMPTY:
          return "empty";
        case FILESYSTEM:
          return "filesystem";
        default:
          return "noncore";
      }
    }

    ObjectDbPtr
    ObjectDbParameters::generateDb() const
    {
      ObjectDbPtr res;

      ObjectDbParametersRaw params_raw = raw();

      switch (type())
      {
        case ObjectDbParameters::COUCHDB:
          res.reset(new ObjectDbCouch());
          break;
        case ObjectDbParameters::EMPTY:
          res.reset(new ObjectDbEmpty());
          break;
        case ObjectDbParameters::FILESYSTEM:
          res.reset(new ObjectDbFilesystem());
          break;
        default:
          std::cerr << "Cannot generate DB for non-core" << std::endl;
          break;
      }

      ObjectDbParameters params_non_const = *this;
      res->set_parameters(params_non_const);

      return res;
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Document::Document()
    {

    }
    Document::~Document()
    {

    }

    void
    Document::set_db(const ObjectDbPtr& db)
    {
      db_ = db;
    }

    void
    Document::set_document_id(const DocumentId &document_id)
    {
      document_id_ = document_id;
    }

    void
    Document::load_fields() {
      // Load all fields from the DB (not the attachments)
      db_->load_fields(document_id_, fields_);
    }

    /** Persist your object to a given DB
     * @param db the DB to persist to
     * @param collection the collection/schema where it should be saved
     */
    void
    Document::Persist()
    {
      // Persist the object if it does not exist in the DB
      if (document_id_.empty())
        db_->insert_object(fields_, document_id_, revision_id_);
      else
        db_->persist_fields(document_id_, fields_, revision_id_);

      // Persist the attachments
      for (AttachmentMap::const_iterator attachment = attachments_.begin(), attachment_end = attachments_.end();
          attachment != attachment_end; ++attachment)
      {
        // Persist the attachment
        db_->set_attachment_stream(document_id_, attachment->first, attachment->second->type_,
                                  attachment->second->stream_, revision_id_);
      }
    }

    /** Extract the stream of a specific attachment for a Document from the DB
     * Not const because it might change the revision_id_
     * @param db the db to read from
     * @param attachment_name the name of the attachment
     * @param stream the string of data to write to
     * @param mime_type the MIME type as stored in the DB
     * @param do_use_cache if true, try to load and store data in the object itself
     */
    void
    Document::get_attachment_stream(const AttachmentName &attachment_name, std::ostream& stream,
                                    MimeType mime_type) const
    {
      // check if it is loaded
      AttachmentMap::const_iterator val = attachments_.find(attachment_name);
      if (val != attachments_.end())
      {
        stream << val->second->stream_.rdbuf();
        return;
      }

      StreamAttachment::ptr stream_attachment(new StreamAttachment(mime_type));
      // Otherwise, load it from the DB
      db_->get_attachment_stream(document_id_, revision_id_, attachment_name, mime_type, stream_attachment->stream_);

      stream << stream_attachment->stream_.rdbuf();
    }

    /** Extract the stream of a specific attachment for a Document from the DB
     * Not const because it might change the revision_id_
     * @param db the db to read from
     * @param attachment_name the name of the attachment
     * @param stream the string of data to write to
     * @param mime_type the MIME type as stored in the DB
     * @param do_use_cache if true, try to load and store data in the object itself
     */
    void
    Document::get_attachment_stream_and_cache(const AttachmentName &attachment_name, std::ostream& stream,
                                              MimeType mime_type)
    {
      // check if it is loaded
      AttachmentMap::const_iterator val = attachments_.find(attachment_name);
      if (val != attachments_.end())
      {
        stream << val->second->stream_.rdbuf();
        return;
      }

      StreamAttachment::ptr stream_attachment(new StreamAttachment(mime_type));
      // Otherwise, load it from the DB
      db_->get_attachment_stream(document_id_, revision_id_, attachment_name, mime_type, stream_attachment->stream_);
      stream << stream_attachment->stream_.rdbuf();

      attachments_[attachment_name] = stream_attachment;
    }

    void
    Document::SetIdRev(const std::string& id, const std::string& rev)
    {
      document_id_ = id;
      revision_id_ = rev;
      set_field<std::string>("_id", id);
      set_field<std::string>("_rev", rev);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Extract the stream of a specific attachment for a Document from the DB
     * Not const because it might change the revision_id_
     * @param db the db to read from
     * @param attachment_name the name of the attachment
     * @param stream the string of data to write to
     * @param mime_type the MIME type as stored in the DB
     * @param do_use_cache if true, try to load and store data in the object itself
     */
    void
    DummyDocument::get_attachment_stream(const AttachmentName &attachment_name, std::ostream& stream,
                                    MimeType mime_type) const
    {
      // check if it is loaded
      AttachmentMap::const_iterator val = attachments_.find(attachment_name);
      if (val != attachments_.end())
      {
	  val->second->stream_.seekg(0);
	  stream << val->second->stream_.rdbuf();
      }
    }

    /** Add a stream attachment to a a Document
     * @param attachment_name the name of the stream
     * @param stream the stream itself
     * @param content_type the MIME type of the stream
     */
    void
    DummyDocument::set_attachment_stream(const AttachmentName &attachment_name, const std::istream& stream,
                                         const MimeType& mime_type)
    {
      StreamAttachment::ptr stream_attachment(new StreamAttachment(mime_type, stream));
      attachments_[attachment_name] = stream_attachment;
    }

    void
    DummyDocument::ClearAllFields()
    {
      fields_.clear();
    }

    void
    DummyDocument::ClearField(const std::string& key)
    {
      fields_.erase(key);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef CV_MAJOR_VERSION
    // Specializations for cv::Mat
    template<>
    void
    DummyDocument::get_attachment<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value) const
    {
      std::stringstream ss;
      get_attachment_stream(attachment_name, ss, "text/x-yaml");
      std::map<std::string, cv::Mat> ss_map;
      ss_map[attachment_name] = cv::Mat();
      object_recognition_core::db::yaml2mats(ss_map, ss, true);
      value = ss_map[attachment_name];
    }

    template<>
    void
    DummyDocument::set_attachment<cv::Mat>(const AttachmentName &attachment_name, const cv::Mat & value)
    {
      std::stringstream ss;
      std::map<std::string, cv::Mat> ss_map;
      ss_map[attachment_name] = value;
      object_recognition_core::db::mats2yaml(ss_map, ss, true);
      set_attachment_stream(attachment_name, ss, "text/x-yaml");
    }

    // Specializations for cv::Mat
    template<>
    void
    Document::get_attachment_and_cache<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value)
    {
      std::stringstream ss;
      get_attachment_stream_and_cache(attachment_name, ss, "text/x-yaml");
      std::map<std::string, cv::Mat> ss_map;
      ss_map[attachment_name] = cv::Mat();
      object_recognition_core::db::yaml2mats(ss_map, ss, true);
      value = ss_map[attachment_name];
    }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
}
