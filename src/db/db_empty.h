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

#ifndef LOCAL_ORK_CORE_DB_DB_EMPTY_H_
#define LOCAL_ORK_CORE_DB_DB_EMPTY_H_

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/db.h>

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

class ObjectDbEmpty;

namespace object_recognition_core {
namespace db {

template<>
struct ObjectDbDefaults<ObjectDbEmpty> {
  static object_recognition_core::db::ObjectDbParametersRaw default_raw_parameters() {
    ObjectDbParametersRaw res;
    res["type"] = type();

    return res;
  }
  static object_recognition_core::db::DbType type() {
    return "empty";
  }
};
}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** This class saves any data to disk, following: http://code.google.com/p/couchdb-fuse
 */
class ObjectDbEmpty: public object_recognition_core::db::ObjectDb
{
public:
  inline virtual ObjectDbParametersRaw
  default_raw_parameters() const {
    return object_recognition_core::db::ObjectDbDefaults<ObjectDbEmpty>::default_raw_parameters();
  }

  inline virtual void
  insert_object(const or_json::mObject &fields, DocumentId & document_id, RevisionId & revision_id) {}

  inline virtual void
  persist_fields(const DocumentId & document_id, const or_json::mObject &fields, RevisionId & revision_id) {}

  inline virtual void
  load_fields(const DocumentId & document_id, or_json::mObject &fields) {}

  inline virtual void
  get_attachment_stream(const DocumentId & document_id, const RevisionId & revision_id, const std::string& attachment_name,
                        const std::string& content_type, std::ostream& stream) {}

  inline virtual void
  set_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                        const MimeType& mime_type, const std::istream& stream, RevisionId & revision_id) {}

  inline virtual
  void
  Delete(const DocumentId & id) {}

  inline virtual
  void
  QueryView(const View & view, int limit_rows, int start_offset, int& total_rows, int& offset,
        std::vector<Document> & view_elements) {}

  inline virtual void
  QueryGeneric(const std::vector<std::string> & queries, int limit_rows, int start_offset, int& total_rows, int& offset,
               std::vector<Document> & view_elements) {}

  inline virtual std::string
  Status() const {
    return "";
  }

  inline virtual std::string
  Status(const CollectionName& collection) const { return "";}

  inline virtual void
  CreateCollection(const CollectionName &collection) {}

  inline virtual void
  DeleteCollection(const CollectionName &collection){}

  inline virtual DbType
  type() const
  {
    return object_recognition_core::db::ObjectDbDefaults<ObjectDbEmpty>::type();
  }
};

#endif /* LOCAL_ORK_CORE_DB_DB_EMPTY_H_ */
