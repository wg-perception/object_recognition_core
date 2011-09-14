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

#ifndef DB_COUCH_H_
#define DB_COUCH_H_

#include "curl_interface.h"
#include "db_base.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using object_recognition::db_future::AttachmentName;
using object_recognition::db_future::CollectionName;
using object_recognition::db_future::DocumentId;
using object_recognition::db_future::DbType;
using object_recognition::db_future::MimeType;
using object_recognition::db_future::RevisionId;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ObjectDbCouch: public object_recognition::db_future::ObjectDbBase
{
public:
  ObjectDbCouch(const std::string &url);

  virtual void
  insert_object(const CollectionName &collection, const boost::property_tree::ptree &fields, DocumentId & document_id,
                RevisionId & revision_id);

  virtual void
  persist_fields(const DocumentId & document_id, const CollectionName &collection,
                 const boost::property_tree::ptree &fields, RevisionId & revision_id);

  virtual void
  load_fields(const DocumentId & document_id, const CollectionName &collection, boost::property_tree::ptree &fields);

  virtual void
  get_attachment_stream(const DocumentId & document_id, const CollectionName &collection,
                        const std::string& attachment_name, const std::string& content_type, std::ostream& stream,
                        RevisionId & revision_id);

  virtual void
  set_attachment_stream(const DocumentId & document_id, const CollectionName &collection,
                        const AttachmentName& attachment_name, const MimeType& mime_type, const std::istream& stream,
                        RevisionId & revision_id);

  virtual void
  Query(const std::vector<std::string> & queries, const CollectionName & collection_name, int limit_rows,
        int start_offset, int& total_rows, int& offset, std::vector<DocumentId> & document_ids);

  virtual DbType
  type()
  {
    return "CouchDB";
  }
private:

  inline void
  precondition_id(const DocumentId & id) const
  {
    if (id.empty())
      throw std::runtime_error("The document's id must be initialized.");
  }

  inline void
  precondition_rev(const RevisionId & rev) const
  {
    if (rev.empty())
      throw std::runtime_error("The document must have a valid revision.");
  }

  void
  upload_json(const boost::property_tree::ptree &ptree, const std::string& url, const std::string& request);

  inline std::string
  url_id(const CollectionName & collection_name, const DocumentId & id) const
  {
    return url_ + "/" + collection_name + (id.empty() ? "" : "/" + id);
  }
  inline std::string
  url_id_rev(const CollectionName &collection_name, const DocumentId & id, const RevisionId & rev) const
  {
    return url_id(collection_name, id) + "?rev=" + rev;
  }

  void
  GetObjectRevisionId(DocumentId& document_id, RevisionId & revision_id);

  void
  GetRevisionId(RevisionId & revision_id);

  /** The URL of the DB, including port */
  std::string url_;

  //FIXME why are these mutable
  mutable object_recognition::curl::cURL curl_;
  mutable std::stringstream json_writer_stream_, json_reader_stream_;

  object_recognition::curl::writer json_writer_;
  object_recognition::curl::reader json_reader_;
};

#endif /* DB_COUCH_H_ */
