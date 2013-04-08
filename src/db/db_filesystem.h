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

#ifndef DB_FILESYSTEM_H_
#define DB_FILESYSTEM_H_

#include <boost/filesystem.hpp>

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/db.h>

#include "curl_interface.h"
#include "db_default.h"

using object_recognition_core::db::AttachmentName;
using object_recognition_core::db::CollectionName;
using object_recognition_core::db::DbType;
using object_recognition_core::db::Document;
using object_recognition_core::db::DocumentId;
using object_recognition_core::db::ObjectId;
using object_recognition_core::db::MimeType;
using object_recognition_core::db::ObjectDbParametersRaw;
using object_recognition_core::db::RevisionId;
using object_recognition_core::db::View;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ObjectDbFilesystem;

class ObjectDbFilesystem;

namespace object_recognition_core {
namespace db {

template<>
struct ObjectDbDefaults<ObjectDbFilesystem> {
  static object_recognition_core::db::ObjectDbParametersRaw default_raw_parameters() {
    ObjectDbParametersRaw res;
    res["path"] = "/tmp";
    res["collection"] = "object_recognition";
    res["type"] = type();

    return res;
  }
  static object_recognition_core::db::DbType type() {
    return "filesystem";
  }
};
}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** This class saves any data to disk, following: http://code.google.com/p/couchdb-fuse
 * Or again:
 *     mnt/
 *       dbname/
 *         all_docs/
 *           id1/
 *             value (this contains the JSON-encoded dump of this document)
 *             attachments
 *               att1.jpg
 *               ...
 *           id2/
 *             value
 *             ...
 *         view/
 *           designdoc1/
 *             viewname/
 *               key1/
 *                 doc (this is a symlink to something like `all_docs/id1'
 *                 value
 */
class ObjectDbFilesystem: public object_recognition_core::db::ObjectDb
{
public:
  ObjectDbFilesystem();

  ObjectDbParametersRaw
  default_raw_parameters() const;

  void
  set_parameters(object_recognition_core::db::ObjectDbParameters & parameters);

  virtual void
  insert_object(const or_json::mObject &fields, DocumentId & document_id, RevisionId & revision_id);

  virtual void
  persist_fields(const DocumentId & document_id, const or_json::mObject &fields, RevisionId & revision_id);

  virtual void
  load_fields(const DocumentId & document_id, or_json::mObject &fields);

  virtual void
  get_attachment_stream(const DocumentId & document_id, const RevisionId & revision_id, const std::string& attachment_name,
                        const std::string& content_type, std::ostream& stream);

  virtual void
  set_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                        const MimeType& mime_type, const std::istream& stream, RevisionId & revision_id);

  virtual
  void
  Delete(const DocumentId & id);

  virtual
  void
  QueryView(const View & view, int limit_rows, int start_offset, int& total_rows, int& offset,
            std::vector<Document> & view_elements);

  virtual void
  QueryGeneric(const std::vector<std::string> & queries, int limit_rows, int start_offset, int& total_rows, int& offset,
               std::vector<Document> & view_elements);

  virtual std::string
  Status() const;

  virtual std::string
  Status(const CollectionName& collection) const;

  virtual void
  CreateCollection(const CollectionName &collection);

  virtual void
  DeleteCollection(const CollectionName &collection);

  virtual DbType
  type() const;
private:
  static const RevisionId DEFAULT_REVISION_ID_;

  inline void
  precondition_id(const DocumentId & id) const
  {
    if (id.empty())
      throw std::runtime_error("The document's id must be initialized.");
  }

  template<typename T>
  void
  read_json(T &reader, or_json::mObject& object)
  {
    or_json::mValue value;
    or_json::read(reader, value);
    object = value.get_obj();
  }

  template<typename T>
  void
  write_json(const or_json::mObject& object, T &writer)
  {
    or_json::mValue value(object);
    or_json::write(value, writer);
  }

  inline boost::filesystem::path
  url_id(const DocumentId & id) const
  {
    return path_ / collection_ / "all_docs" / id;
  }

  inline boost::filesystem::path
  url_value(const DocumentId & id) const
  {
    return url_id(id) / "value";
  }

  inline boost::filesystem::path
  url_attachments(const DocumentId & id) const
  {
    return url_id(id) / "attachments";
  }

  /** The path of the DB, not including the collection */
  boost::filesystem::path path_;
  /** The collection to operate upon */
  std::string collection_;
};

#endif /* DB_FILESYSTEM_H_ */
