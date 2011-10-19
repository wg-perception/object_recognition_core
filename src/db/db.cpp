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

#include "db_base.h"
#include "db_couch.h"
#include "object_recognition/db/db.h"
#include <object_recognition/db/opencv.h>

#define PRECONDITION_DB() if(!db_) throw std::runtime_error(std::string("This ObjectDb instance is uninitialized."));
namespace object_recognition
{
  namespace db_future
  {
    const std::string ObjectDbParameters::EMPTY = "empty";
    const std::string ObjectDbParameters::COUCHDB = "CouchDB";
    const std::string DEFAULT_COUCHDB_URL = "http://localhost:5984";

    ObjectDbParameters::ObjectDbParameters()
    {
      type_ = EMPTY;
    }

    /** Default constructor for certain types
     * @param type Default type
     */
    ObjectDbParameters::ObjectDbParameters(const std::string& json_params)
    {
      if (json_params == ObjectDbParameters::COUCHDB)
      {
        type_ = ObjectDbParameters::COUCHDB;
        root_ = DEFAULT_COUCHDB_URL;
      } else
        throw std::runtime_error("Invalid type.");
    }
    ObjectDbParameters::ObjectDbParameters(const std::map<std::string, std::string>& parameters)
    {
      FillParameters(parameters);
    }

    void
    ObjectDbParameters::FillParameters(const std::map<std::string, std::string>& parameters)
    {
      all_parameters_ = parameters;
      if (all_parameters_.find("type") == all_parameters_.end())
      {
        throw std::runtime_error("You must supply a database type. e.g. CouchDB");
      }
      type_ = all_parameters_.at("type");
      if (type_ == ObjectDbParameters::EMPTY)
        return;

      if (all_parameters_.find("root") == all_parameters_.end())
      {
        throw std::runtime_error("You must supply a root . e.g. /home/me, http://localhost:5984");
      }
      root_ = all_parameters_.at("root");
      if (all_parameters_.find("collection") != all_parameters_.end())
        collection_ = all_parameters_.at("collection");
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ObjectDb::ObjectDb(const ObjectDbParameters &in_params)
    {
      set_params(in_params);
    }
    ObjectDb::ObjectDb(const std::string& json_params)
    {
      set_params(json_params);
    }

    void
    ObjectDb::set_params(const std::string& json_params)
    {
      set_params(ObjectDbParameters(json_params));
    }
    void
    ObjectDb::set_params(const ObjectDbParameters &in_params)
    {
      db_parameters_ = in_params;
      db_ = boost::shared_ptr<ObjectDbBase>(new ObjectDbCouch(db_parameters_.root_));
    }

    void
    ObjectDb::insert_object(const CollectionName &collection, const boost::property_tree::ptree &fields,
                            DocumentId & document_id, RevisionId & revision_id) const
    {
      PRECONDITION_DB()
      db_->insert_object(collection, fields, document_id, revision_id);
    }

    void
    ObjectDb::set_attachment_stream(const DocumentId & document_id, const CollectionName &collection,
                                    const AttachmentName& attachment_name, const MimeType& content_type,
                                    const std::istream& stream, RevisionId & revision_id) const
    {
      PRECONDITION_DB()
      db_->set_attachment_stream(document_id, collection, attachment_name, content_type, stream, revision_id);
    }

    void
    ObjectDb::get_attachment_stream(const DocumentId & document_id, const CollectionName &collection,
                                    const AttachmentName& attachment_name, MimeType& content_type, std::ostream& stream,
                                    RevisionId & revision_id) const
    {
      PRECONDITION_DB()
      db_->get_attachment_stream(document_id, collection, attachment_name, content_type, stream, revision_id);
    }

    void
    ObjectDb::load_fields(const DocumentId & document_id, const CollectionName &collection,
                          boost::property_tree::ptree &fields) const
    {
      PRECONDITION_DB()
      db_->load_fields(document_id, collection, fields);
    }

    void
    ObjectDb::persist_fields(const DocumentId & document_id, const CollectionName &collection,
                             const boost::property_tree::ptree &fields, RevisionId & revision_id) const
    {
      PRECONDITION_DB()
      db_->persist_fields(document_id, collection, fields, revision_id);
    }

    void
    ObjectDb::Query(const std::vector<std::string> & queries, const CollectionName & collection_name, int limit_rows,
                    int start_offset, int& total_rows, int& offset, std::vector<DocumentId> & document_ids) const
    {
      PRECONDITION_DB()
      db_->Query(queries, collection_name, limit_rows, start_offset, total_rows, offset, document_ids);
    }

    void
    ObjectDb::Status(std::string& status)
    {
      PRECONDITION_DB()
      db_->Status(status);
    }

    void
    ObjectDb::Status(const CollectionName& collection, std::string& status)
    {
      PRECONDITION_DB()
      db_->Status(collection, status);
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

    Document::Document(const ObjectDb& db, const CollectionName & collection)
        :
          db_(db),
          collection_(collection)
    {

    }
    Document::Document(const ObjectDb & db, const CollectionName & collection, const DocumentId &document_id)
        :
          db_(db),
          collection_(collection),
          document_id_(document_id)
    {
      // Load all fields from the DB (not the attachments)
      db.load_fields(document_id_, collection_, fields_);
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
        db_.insert_object(collection_, fields_, document_id_, revision_id_);
      else
        db_.persist_fields(document_id_, collection_, fields_, revision_id_);

      // Persist the attachments
      boost::any nothing_any;
      for (AttachmentMap::const_iterator attachment = attachments_.begin(), attachment_end = attachments_.end();
          attachment != attachment_end; ++attachment)
      {
        // Persist the attachment
        db_.set_attachment_stream(document_id_, collection_, attachment->first, attachment->second->type_,
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
    Document::get_attachment_stream(const AttachmentName &attachment_name, std::ostream& stream, MimeType mime_type,
                                    bool do_use_cache)
    {
      // check if it is loaded
      if (do_use_cache)
      {
        AttachmentMap::const_iterator val = attachments_.find(attachment_name);
        if (val != attachments_.end())
        {
          stream << val->second->stream_.rdbuf();
          return;
        }
      }

      StreamAttachment::ptr stream_attachment(new StreamAttachment(mime_type));
      // Otherwise, load it from the DB
      db_.get_attachment_stream(document_id_, collection_, attachment_name, mime_type, stream_attachment->stream_,
                                revision_id_);
      stream << stream_attachment->stream_.rdbuf();
      if (do_use_cache)
      {
        attachments_[attachment_name] = stream_attachment;
      }
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

    const unsigned int DocumentView::BATCH_SIZE = 100;

    DocumentView::DocumentView()
        :
          start_offset_(0)
    {
    }

    /** Add requirements for the documents to retrieve
     * @param db_type the type of the db that will be used
     * @param view a View that will filter the Documents. The format depends on your ObjectDb
     */
    void
    DocumentView::AddView(const DbType &db_type, const View & view)
    {
      pod_views_.push_back(PodView(db_type, view));
    }

    /** Set the db on which to perform the Query
     * @param db The db on which the query is performed
     */
    void
    DocumentView::set_db(const ObjectDb & db)
    {
      db_ = db;
    }

    /** Set the collection on which to perform the Query. This might be part of the views_
     * and unnecessary for certain DB's
     * @param collection The collection on which the query is performed
     */
    void
    DocumentView::set_collection(const CollectionName & collection)
    {
      collection_ = collection;
    }

    /** Perform the query itself
     * @return an Iterator that will iterate over each result
     */
    DocumentView &
    DocumentView::begin()
    {
      BOOST_FOREACH(const PodView & view_pod, pod_views_)
            if (view_pod.db_type_ == db_.type())
              views_.push_back(view_pod.view_);

      // Process the query and get the ids of several objects
      std::vector<DocumentId> document_ids;
      db_.Query(views_, collection_, BATCH_SIZE, start_offset_, total_rows_, start_offset_, document_ids_);
      return *this;
    }

    DocumentView
    DocumentView::end()
    {
      return DocumentView();
    }

    DocumentView &
    DocumentView::operator++()
    {
      // Move forward in the list of Objects to check
      document_ids_.pop_back();
      // Return the end iterator if we are done
      if (document_ids_.empty())
      {
        // Figure out if we need to query for more document ids
        if (start_offset_ < total_rows_)
          db_.Query(views_, collection_, BATCH_SIZE, start_offset_, total_rows_, start_offset_, document_ids_);
      }
      else
      {
        // Fill the current object
        document_ids_.pop_back();
      }
      return *this;
    }

    bool
    DocumentView::operator!=(const DocumentView & document_view) const
    {
      if (document_view.document_ids_.empty())
        return (!document_ids_.empty());
      if (document_ids_.size() >= document_view.document_ids_.size())
        return std::equal(document_ids_.begin(), document_ids_.end(), document_view.document_ids_.begin());
      else
        return std::equal(document_view.document_ids_.begin(), document_view.document_ids_.end(), document_ids_.begin());
    }

    Document
    DocumentView::operator*() const
    {
      return Document(db_, collection_, document_ids_.back());
    }

    // Specializations for cv::Mat
    template<>
    void
    Document::get_attachment<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value, bool do_use_cache)
    {
      std::stringstream ss;
      get_attachment_stream(attachment_name, ss, "text/x-yaml", do_use_cache);
      std::map<std::string, cv::Mat> ss_map;
      ss_map[attachment_name] = cv::Mat();
      object_recognition::db::yaml2mats(ss_map, ss, true);
      value = ss_map[attachment_name];
    }
    template<>
    void
    Document::set_attachment<cv::Mat>(const AttachmentName &attachment_name, const cv::Mat & value)
    {
      std::stringstream ss;
      std::map<std::string, cv::Mat> ss_map;
      ss_map[attachment_name] = value;
      object_recognition::db::mats2yaml(ss_map, ss, true);
      set_attachment_stream(attachment_name, ss, "text/x-yaml");
    }
  }
}
