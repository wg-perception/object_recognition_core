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
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

#include "db_filesystem.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const RevisionId ObjectDbFilesystem::DEFAULT_REVISION_ID_ = "0";

ObjectDbFilesystem::ObjectDbFilesystem()
    :
      ObjectDbBase("/tmp", "object_recognition"),
      path_(root_)
{
}

ObjectDbFilesystem::ObjectDbFilesystem(const std::string &root, const std::string &collection)
    :
      ObjectDbBase(root, collection),
      path_(root)
{
}

void
ObjectDbFilesystem::insert_object(const or_json::mObject &fields, DocumentId & document_id, RevisionId & revision_id)
{
  // Find a hash key that is not on disk
  std::string hexa_values = "0123456789abcdef";
  while (true)
  {
    document_id = "";
    // 32 is the CouchDB hash key size
    for (unsigned int i = 0; i < 32; ++i)
      document_id.append(hexa_values.substr(rand() % 16, 1));
    boost::filesystem::path url = url_id(document_id);
    // Stop if the folder does not exist
    if (!boost::filesystem::exists(url))
      break;
  }

  persist_fields(document_id, fields, revision_id);
}

void
ObjectDbFilesystem::persist_fields(const DocumentId & document_id, const or_json::mObject &fields,
                                   RevisionId & revision_id)
{
  precondition_id(document_id);

  // Save the JSON to disk
  boost::filesystem::create_directories(url_id(document_id));
  std::ofstream file(url_value(document_id).string().c_str());
  write_json(fields, file);
  file.close();

  revision_id = DEFAULT_REVISION_ID_;
}

void
ObjectDbFilesystem::load_fields(const DocumentId & document_id, or_json::mObject &fields)
{
  Status();
  precondition_id(document_id);

  // Read the JSON from disk
  if (!boost::filesystem::exists(url_value(document_id)))
    throw std::runtime_error("Object Not Found : " + url_value(document_id).string());
  std::ifstream file(url_value(document_id).string().c_str());
  read_json(file, fields);
  file.close();
}

void
ObjectDbFilesystem::set_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                                          const MimeType& mime_type, const std::istream& stream,
                                          RevisionId & revision_id)
{
  precondition_id(document_id);

  // Write the stream to a file
  boost::filesystem::path path = url_attachments(document_id) / attachment_name;
  std::ofstream file(path.string().c_str(), std::ios::binary);
  std::stringstream stream_;
  stream_ << stream.rdbuf();
  stream_.seekg(0);
  file << stream_;
  file.close();

  // TODO use MIME type

  revision_id = DEFAULT_REVISION_ID_;
}

void
ObjectDbFilesystem::get_attachment_stream(const DocumentId & document_id, const std::string& attachment_name,
                                          const std::string& content_type, std::ostream& stream,
                                          RevisionId & revision_id)
{
  // Write the stream to a file
  boost::filesystem::path path = url_attachments(document_id) / attachment_name;
  std::ifstream file(path.string().c_str(), std::ios::binary);
  stream << file;
  file.close();

  // TODO use MIME type

  revision_id = DEFAULT_REVISION_ID_;
}

void
ObjectDbFilesystem::Delete(const ObjectId & id)
{
  boost::filesystem::remove_all(url_id(id));
}

void
ObjectDbFilesystem::Query(const object_recognition::db::View & view, int limit_rows, int start_offset, int& total_rows,
                          int& offset, std::vector<DocumentId> & document_ids)
{
  throw std::runtime_error("Function not implemented in the Filesystem DB.");
  /*json_reader_stream_.str("");
   or_json::mObject parameters = view.parameters();
   std::string url;
   bool do_throw;
   switch (view.type())
   {
   case object_recognition::db::View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE:
   url = url_ + "/" + collection_ + "/_design/models/_view/by_object_id_and_" + parameters["model_type"].get_str();
   do_throw = false;
   break;
   }

   ObjectId object_id = parameters["object_id"].get_str();
   QueryView(url, limit_rows, start_offset, "&startkey=\"" + object_id + "\"&endkey=\"" + object_id + "\"", total_rows,
   offset, document_ids, do_throw);*/
}

void
ObjectDbFilesystem::Query(const std::vector<std::string> & queries, int limit_rows, int start_offset, int& total_rows,
                          int& offset, std::vector<DocumentId> & document_ids)
{
  throw std::runtime_error("Function not implemented in the Filesystem DB.");
  /*{
   or_json::mObject fields;
   BOOST_FOREACH(const std::string& query, queries)
   {
   fields["map"] = or_json::mValue(query);
   }
   json_reader_stream_.str("");
   write_json(fields, json_reader_stream_);
   }

   QueryView(url_ + "/" + collection_ + "/_temp_view", limit_rows, start_offset, "", total_rows, offset, document_ids,
   true);*/
}

/** Once json_reader_stream_ has been filled, call that function to get the results of the view
 *
 */
void
ObjectDbFilesystem::QueryView(const std::string & in_url, int limit_rows, int start_offset, const std::string &options,
                              int& total_rows, int& offset, std::vector<DocumentId> & document_ids, bool do_throw)
{
  throw std::runtime_error("Function not implemented in the Filesystem DB.");
  /*if (limit_rows <= 0)
   limit_rows = std::numeric_limits<int>::max();
   json_writer_stream_.str("");
   curl_.reset();
   curl_.setReader(&json_reader_);
   curl_.setWriter(&json_writer_);
   std::string url = in_url + "?limit=" + boost::lexical_cast<std::string>(limit_rows) + "&skip="
   + boost::lexical_cast<std::string>(start_offset)
   + options;
   curl_.setURL(url);
   curl_.setHeader("Content-Type: application/json");
   curl_.setCustomRequest("GET");
   curl_.perform();

   if (curl_.get_response_code() != object_recognition::curl::cURL::OK)
   {
   if (do_throw)
   throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
   else
   {
   total_rows = 0;
   offset = 0;
   return;
   }
   }

   json_reader_stream_.seekg(0);
   json_writer_stream_.seekg(0);

   or_json::mObject fields;
   read_json(json_writer_stream_, fields);

   total_rows = fields["total_rows"].get_int();
   document_ids.clear();
   BOOST_FOREACH(or_json::mValue & v, fields["rows"].get_array())
   {
   // values are: id, key, value
   document_ids.push_back(v.get_obj()["id"].get_str());
   }
   offset = fields["offset"].get_int() + document_ids.size();*/
}

void
ObjectDbFilesystem::CreateCollection(const CollectionName &collection)
{
  std::string status;
  Status(status);
  boost::filesystem::create_directories(path_ / collection);
}

std::string
ObjectDbFilesystem::Status()
{
  // To comply the CouchDB status function
  if (boost::filesystem::exists(path_))
  {
    return "{\"filesystem\":\"Welcome\",\"version\":\"1.0\"}";
  }
  else
    throw std::runtime_error("Path " + path_.string() + " does not exist. Please create.");
}

std::string
ObjectDbFilesystem::Status(const CollectionName& collection)
{
  Status();
  if (!boost::filesystem::exists(path_ / collection))
    return "{\"error\":\"not_found\",\"reason\":\"no_db_file\"}";
  else
    return "{\"db_name\":\"" + collection + "\"}";
}

void
ObjectDbFilesystem::DeleteCollection(const CollectionName &collection)
{
  std::string status;
  Status(status);
  if (boost::filesystem::exists(path_ / collection))
    boost::filesystem::remove_all(path_ / collection);
}
