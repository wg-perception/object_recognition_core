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
#include <boost/bind.hpp>

#include "db_couch.h"
#include "db_filesystem.h"
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/db_base.h>
#include <object_recognition_core/db/opencv.h>
#include <object_recognition_core/db/view.h>

#define PRECONDITION_DB() if(!db_) throw std::runtime_error(std::string("This ObjectDb instance is uninitialized."));
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
    ObjectDbParameters::ObjectDbParameters(const std::string& type_str)
    {
      type_ = StringToType(type_str);
      switch (type_)
      {
        case ObjectDbParameters::COUCHDB:
        {
          ObjectDbCouch tmp;
          root_ = tmp.root();
          collection_ = tmp.collection();
          break;
        }
        case ObjectDbParameters::EMPTY:
        {
          break;
        }
        case ObjectDbParameters::FILESYSTEM:
        {
          ObjectDbFilesystem tmp;
          root_ = tmp.root();
          collection_ = tmp.collection();
          break;
        }
        default:
        {
          throw std::runtime_error("No implementation for that db enum type.");
          break;
        }
      }
    }
    ObjectDbParameters::ObjectDbParameters(const or_json::mObject& parameters)
    {
      FillParameters(parameters);
    }

    ObjectDbParameters::ObjectDbType
    ObjectDbParameters::StringToType(const std::string & type_str)
    {
      if (type_str == "CouchDB")
        return COUCHDB;
      else if (type_str == "empty")
        return EMPTY;
      else if (type_str == "filesystem")
        return FILESYSTEM;
      else
        throw std::runtime_error(type_str + ": Invalid type. Possible are 'CouchDB', 'empty' and 'filesystem'");
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
          throw std::runtime_error("No conversion to string implemented for that type");
      }
      return "";
    }

    void
    ObjectDbParameters::FillParameters(const or_json::mObject& parameters)
    {
      raw_ = parameters;
      if (raw_.find("type") == raw_.end())
      {
        throw std::runtime_error("You must supply a database type. e.g. CouchDB");
      }
      type_ = StringToType(raw_.at("type").get_str());
      if (type_ == ObjectDbParameters::EMPTY)
        return;

      if (raw_.find("collection") != raw_.end())
        collection_ = raw_.at("collection").get_str();
      if (raw_.find("root") != raw_.end())
        root_ = raw_.at("root").get_str();
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ObjectDb::ObjectDb(const ObjectDbParameters &in_params)
    {
      set_parameters(in_params);
    }
    ObjectDb::ObjectDb(const std::string& json_params)
    {
      set_parameters(json_params);
    }

    void
    ObjectDb::set_parameters(const std::string& json_params)
    {
      set_parameters(ObjectDbParameters(json_params));
    }
    void
    ObjectDb::set_parameters(const ObjectDbParameters &in_params)
    {
      parameters_ = in_params;
      switch (parameters_.type_)
      {
        case ObjectDbParameters::COUCHDB:
          db_ = boost::shared_ptr<ObjectDbBase>(new ObjectDbCouch(parameters_.root_, parameters_.collection_));
          return;
        case ObjectDbParameters::EMPTY:
          return;
        case ObjectDbParameters::FILESYSTEM:
          db_ = boost::shared_ptr<ObjectDbBase>(new ObjectDbFilesystem(parameters_.root_, parameters_.collection_));
          return;
        default:
          throw std::runtime_error("No set_parameters implemented for that db type.");
      }
    }

    void
    ObjectDb::set_db_and_parameters(const boost::shared_ptr<ObjectDbBase> & db_base, const ObjectDbParameters &in_params) {
      db_ = db_base;
      parameters_ = in_params;
    }

    void
    ObjectDb::insert_object(const or_json::mObject &fields, DocumentId & document_id, RevisionId & revision_id) const
    {
      PRECONDITION_DB()
      db_->insert_object(fields, document_id, revision_id);
    }

    void
    ObjectDb::set_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                                    const MimeType& content_type, const std::istream& stream,
                                    RevisionId & revision_id) const
    {
      PRECONDITION_DB()
      db_->set_attachment_stream(document_id, attachment_name, content_type, stream, revision_id);
    }

    void
    ObjectDb::get_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                                    MimeType& content_type, std::ostream& stream, RevisionId & revision_id) const
    {
      PRECONDITION_DB()
      db_->get_attachment_stream(document_id, attachment_name, content_type, stream, revision_id);
    }

    void
    ObjectDb::load_fields(const DocumentId & document_id, or_json::mObject &fields) const
    {
      PRECONDITION_DB()
      db_->load_fields(document_id, fields);
    }

    void
    ObjectDb::persist_fields(const DocumentId & document_id, const or_json::mObject &fields,
                             RevisionId & revision_id) const
    {
      PRECONDITION_DB()
      db_->persist_fields(document_id, fields, revision_id);
    }

    void
    ObjectDb::Delete(const ObjectId & id) const
    {
      PRECONDITION_DB()
      db_->Delete(id);
    }

    ObjectDb::QueryFunction
    ObjectDb::Query(const View &view) const
    {
      return boost::bind(&ObjectDb::Query_, *this, view, _1, _2, _3, _4, _5);
    }

    void
    ObjectDb::Query_(const View &view, int limit_rows, int start_offset, int& total_rows, int& offset,
                     std::vector<ViewElement> & view_elements)
    {
      PRECONDITION_DB()
      db_->Query(view, limit_rows, start_offset, total_rows, offset, view_elements);
    }

    std::string
    ObjectDb::Status()
    {
      PRECONDITION_DB()
      return db_->Status();
    }

    std::string
    ObjectDb::Status(const CollectionName& collection)
    {
      PRECONDITION_DB()
      return db_->Status(collection);
    }
    void
    ObjectDb::CreateCollection(const CollectionName &collection)
    {
      PRECONDITION_DB()
      db_->CreateCollection(collection);
    }

    void
    ObjectDb::DeleteCollection(const CollectionName &collection)
    {
      PRECONDITION_DB()
      db_->DeleteCollection(collection);
    }

    DbType
    ObjectDb::type()
    {
      PRECONDITION_DB()
      return db_->type();
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Document::Document()
    {

    }
    Document::~Document()
    {

    }

    Document::Document(const ObjectDb& db)
        :
          db_(db)
    {

    }

    Document::Document(const ObjectDb & db, const DocumentId &document_id)
        :
          db_(db),
          document_id_(document_id)
    {
      // Load all fields from the DB (not the attachments)
      db.load_fields(document_id_, fields_);
    }

    void
    Document::update_db(const ObjectDb& db)
    {
      db_ = db;
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
        db_.insert_object(fields_, document_id_, revision_id_);
      else
        db_.persist_fields(document_id_, fields_, revision_id_);

      // Persist the attachments
      for (AttachmentMap::const_iterator attachment = attachments_.begin(), attachment_end = attachments_.end();
          attachment != attachment_end; ++attachment)
      {
        // Persist the attachment
        db_.set_attachment_stream(document_id_, attachment->first, attachment->second->type_,
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
      RevisionId revision_id;
      db_.get_attachment_stream(document_id_, attachment_name, mime_type, stream_attachment->stream_, revision_id);
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
      db_.get_attachment_stream(document_id_, attachment_name, mime_type, stream_attachment->stream_, revision_id_);
      stream << stream_attachment->stream_.rdbuf();

      attachments_[attachment_name] = stream_attachment;
    }

    /** Add a stream attachment to a a Document
     * @param attachment_name the name of the stream
     * @param stream the stream itself
     * @param content_type the MIME type of the stream
     */
    void
    Document::set_attachment_stream(const AttachmentName &attachment_name, const std::istream& stream,
                                    const MimeType& mime_type)
    {
      StreamAttachment::ptr stream_attachment(new StreamAttachment(mime_type, stream));
      attachments_[attachment_name] = stream_attachment;
    }

    void
    Document::ClearAllFields()
    {
      fields_.clear();
    }

    void
    Document::ClearField(const std::string& key)
    {
      fields_.erase(key);
    }

    void
    Document::SetIdRev(const std::string& id, const std::string& rev)
    {
      document_id_ = id;
      revision_id_ = rev;
      set_value<std::string>("_id", id);
      set_value<std::string>("_rev", rev);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef CV_MAJOR_VERSION
// Specializations for cv::Mat
    template<>
    void
    Document::get_attachment<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value) const
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
    Document::get_attachment_and_cache<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value)
    {
      std::stringstream ss;
      get_attachment_stream_and_cache(attachment_name, ss, "text/x-yaml");
      std::map<std::string, cv::Mat> ss_map;
      ss_map[attachment_name] = cv::Mat();
      object_recognition_core::db::yaml2mats(ss_map, ss, true);
      value = ss_map[attachment_name];
    }

    template<>
    void
    Document::set_attachment<cv::Mat>(const AttachmentName &attachment_name, const cv::Mat & value)
    {
      std::stringstream ss;
      std::map<std::string, cv::Mat> ss_map;
      ss_map[attachment_name] = value;
      object_recognition_core::db::mats2yaml(ss_map, ss, true);
      set_attachment_stream(attachment_name, ss, "text/x-yaml");
    }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ViewIterator::ViewIterator(const View &view, ObjectDb& db)
        :
          start_offset_(0),
          query_(db.Query(view)),
          db_(db)
    {
    }

    const unsigned int ViewIterator::BATCH_SIZE = 100;

    ViewIterator::ViewIterator()
        :
          start_offset_(0)
    {
    }

    /** Set the db on which to perform the Query
     * @param db The db on which the query is performed
     */
    void
    ViewIterator::set_db(const ObjectDb & db)
    {
      db_ = db;
    }

    /** Perform the query itself
     * @return an Iterator that will iterate over each result
     */
    ViewIterator &
    ViewIterator::begin()
    {
      // Process the query and get the ids of several objects
      query_(BATCH_SIZE, start_offset_, total_rows_, start_offset_, view_elements_);
      return *this;
    }

    ViewIterator
    ViewIterator::end()
    {
      return ViewIterator();
    }

    ViewIterator &
    ViewIterator::operator++()
    {
      // If we have nothing else to pop, try to get more from the DB
      if (view_elements_.empty())
      {
        // Figure out if we need to query for more document ids
        if (start_offset_ < total_rows_)
          query_(BATCH_SIZE, start_offset_, total_rows_, start_offset_, view_elements_);
      }
      else if (!view_elements_.empty())
        view_elements_.pop_back();
      return *this;
    }

    bool
    ViewIterator::operator!=(const ViewIterator & document_view) const
    {
      if (document_view.view_elements_.empty())
        return (!view_elements_.empty());
      if (view_elements_.size() >= document_view.view_elements_.size())
        return std::equal(view_elements_.begin(), view_elements_.end(), document_view.view_elements_.begin());
      else
        return std::equal(document_view.view_elements_.begin(), document_view.view_elements_.end(),
                          view_elements_.begin());
    }

    ViewElement
    ViewIterator::operator*() const
    {
      return view_elements_.back();
    }
  }
}
