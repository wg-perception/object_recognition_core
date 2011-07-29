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

namespace object_recognition
{
  namespace db_future
  {
    const std::string ObjectDb::JSON_PARAMS_EMPTY_DB = "{\"type\":\"empty\"}";

    ObjectDb::ObjectDb(const boost::property_tree::ptree& params)
    {
      set_db(params);
    }

    ObjectDb::ObjectDb(const std::string & json_params)
    {
      boost::property_tree::ptree params;
      std::stringstream ssparams;
      ssparams << json_params;
      boost::property_tree::read_json(ssparams, params);

      set_db(params);
    }

    void
    ObjectDb::set_params(const std::string & json_params)
    {
      boost::property_tree::ptree params;
      std::stringstream ssparams;
      ssparams << json_params;
      boost::property_tree::read_json(ssparams, params);

      set_db(params);
    }

    void
    ObjectDb::set_params(const boost::property_tree::ptree& params)
    {
      set_db(params);
    }

    /** Set the db_ using a property tree
     * @params the boost property tree containing the different parameters
     */
    void
    ObjectDb::set_db(const boost::property_tree::ptree& params)
    {
      std::string db_type = params.get<std::string>("type");
      if (db_type == "empty")
      {
      }
      else if (db_type == "CouchDB")
      {
        db_ = boost::shared_ptr<ObjectDbBase>(new ObjectDbCouch(params.get<std::string>("url")));
      }
    }

    void
    ObjectDb::insert_object(const CollectionName &collection, const boost::property_tree::ptree &fields,
                            DocumentId & document_id, RevisionId & revision_id) const
    {
      db_->insert_object(collection, fields, document_id, revision_id);
    }

    void
    ObjectDb::set_attachment_stream(const DocumentId & document_id, const CollectionName &collection,
                                    const AttachmentName& attachment_name, const MimeType& content_type,
                                    const std::istream& stream, RevisionId & revision_id) const
    {
      db_->set_attachment_stream(document_id, collection, attachment_name, content_type, stream, revision_id);
    }

    void
    ObjectDb::get_attachment_stream(const DocumentId & document_id, const CollectionName &collection,
                                    const AttachmentName& attachment_name, MimeType& content_type, std::ostream& stream,
                                    RevisionId & revision_id) const
    {
      db_->get_attachment_stream(document_id, collection, attachment_name, content_type, stream, revision_id);
    }

    void
    ObjectDb::load_fields(const DocumentId & document_id, const CollectionName &collection,
                          boost::property_tree::ptree &fields) const
    {
      db_->load_fields(document_id, collection, fields);
    }

    void
    ObjectDb::persist_fields(const DocumentId & document_id, const CollectionName &collection,
                             const boost::property_tree::ptree &fields, RevisionId & revision_id) const
    {
      db_->persist_fields(document_id, collection, fields, revision_id);
    }

    void
    ObjectDb::query(const CollectionName &collection, const std::map<AttachmentName, std::string> &regexps
                    , std::vector<DocumentId> & document_ids) const
    {
      db_->query(collection, regexps, document_ids);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Persist your object to a given DB
     * @param db the DB to persist to
     * @param collection the collection/schema where it should be saved
     */
    void
    Document::Persist(ObjectDb & db, const CollectionName & collection)
    {
      collection_ = collection;
      // Persist the object if it does not exist in the DB
      if (document_id_.empty())
        db.insert_object(collection_, fields_, document_id_, revision_id_);
      else
        db.persist_fields(document_id_, collection_, fields_, revision_id_);

      // Persist the attachments
      boost::any nothing_any;
      for (AttachmentMap::const_iterator attachment = attachments_.begin(), attachment_end = attachments_.end();
          attachment != attachment_end; ++attachment)
          {
        // Persist the attachment
        db.set_attachment_stream(document_id_, collection_, attachment->first, attachment->second->type_,
                                 attachment->second->stream_, revision_id_);
      }
    }

    /** Extract the stream of a specific attachment from the pre-loaded Document
     * @param attachment_name the name of the attachment
     * @param stream the string of data to write to
     * @param mime_type the MIME type as stored in the DB
     */
    void
    Document::get_attachment_stream(const AttachmentName &attachment_name, std::ostream& stream,
                                    MimeType mime_type) const
    {
      // check if it is loaded
      AttachmentMap::const_iterator val = attachments_.find(attachment_name);
      if (val != attachments_.end())
        stream << val->second->stream_.rdbuf();
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
    Document::get_attachment_stream(ObjectDb & db, const AttachmentName &attachment_name, std::ostream& stream,
                                    MimeType mime_type, bool do_use_cache) const
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
      db.get_attachment_stream(document_id_, collection_, attachment_name, mime_type, stream_attachment->stream_,
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

    View::View()
    {
    }

    /** Add requirements for the documents to retrieve
     * @param field a field to match. Only one regex per field will be accepted
     * @param regex the regular expression the field verifies, in TODO format
     */
    void
    View::AddWhere(const AttachmentName & field, const std::string & regex)
    {
      regexes_[field] = regex;
    }

    /** Add collections that should be checked for specific fields
     * @param collection
     */
    void
    View::set_collection(const CollectionName & collection)
    {
      collection_ = collection;
    }

    /** Set the db on which to perform the Query
     * @param db The db on which the query is performed
     */
    void
    View::set_db(const ObjectDb & db)
    {
      db_ = db;
    }

    /** Perform the query itself
     * @return an Iterator that will iterate over each result
     */
    DocumentIterator
    View::begin()
    {
      // Process the query and get the ids of several objects
      std::vector<DocumentId> document_ids;
      db_.query(collection_, regexes_, document_ids);
      return DocumentIterator(db_, collection_, document_ids);
    }
  }
}
