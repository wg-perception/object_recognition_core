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

#include <sstream>
#include "db_couch.h"

object_recognition::curl::cURL_GS curl_init_cleanup;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ObjectDbCouch::ObjectDbCouch(const std::string &url)
    :
      url_(url),
      json_writer_(json_writer_stream_),
      json_reader_(json_reader_stream_)
{
}

void
ObjectDbCouch::insert_object(const CollectionName &collection, const json_spirit::mObject &fields,
                             DocumentId & document_id, RevisionId & revision_id)
{
  CreateCollection(collection);
  std::string url = url_id(collection, "");
  upload_json(fields, url, "POST");
  GetObjectRevisionId(document_id, revision_id);
}

void
ObjectDbCouch::persist_fields(const DocumentId & document_id, const CollectionName &collection,
                              const json_spirit::mObject &fields, RevisionId & revision_id)
{
  precondition_id(document_id);
  upload_json(fields, url_id(collection, document_id), "PUT");
  //need to update the revision here.
  GetRevisionId(revision_id);
}

void
ObjectDbCouch::load_fields(const DocumentId & document_id, const CollectionName &collection,
                           json_spirit::mObject &fields)
{
  precondition_id(document_id);
  curl_.reset();
  json_writer_stream_.str("");
  curl_.setWriter(&json_writer_);

  curl_.setURL(url_id(collection, document_id));
  curl_.GET();

  curl_.perform();

  if (curl_.get_response_code() != object_recognition::curl::cURL::OK)
  {
    throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
  }
  //update the object from the result.
  read_json(json_writer_stream_, fields);
}

void
ObjectDbCouch::set_attachment_stream(const DocumentId & document_id, const CollectionName &collection,
                                     const AttachmentName& attachment_name, const MimeType& mime_type,
                                     const std::istream& stream, RevisionId & revision_id)
{
  precondition_id(document_id);
  precondition_rev(revision_id);

  object_recognition::curl::reader binary_reader(stream);
  curl_.reset();
  curl_.setReader(&binary_reader);
  json_writer_stream_.str("");
  curl_.setWriter(&json_writer_);
  curl_.setHeader("Content-Type: " + mime_type);
  curl_.setURL(url_id(collection, document_id) + "/" + attachment_name + "?rev=" + revision_id);
  curl_.PUT();
  curl_.perform();
  GetRevisionId(revision_id);
}

void
ObjectDbCouch::get_attachment_stream(const DocumentId & document_id, const CollectionName &collection,
                                     const std::string& attachment_name, const std::string& content_type,
                                     std::ostream& stream, RevisionId & revision_id)
{
  object_recognition::curl::writer binary_writer(stream);
  curl_.reset();
  json_writer_stream_.str("");
  curl_.setWriter(&binary_writer);
  curl_.setURL(url_id(collection, document_id) + "/" + attachment_name);
  curl_.GET();
  curl_.perform();
  if (curl_.get_response_code() != object_recognition::curl::cURL::OK)
  {
    throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
  }
}

void
ObjectDbCouch::GetObjectRevisionId(DocumentId& document_id, RevisionId & revision_id)
{
  json_spirit::mObject params;
  read_json(json_writer_stream_, params);
  document_id = params["id"].get_str();
  revision_id = params["rev"].get_str();
  if (document_id.empty())
    throw std::runtime_error("Could not find the document id");
  if (revision_id.empty())
    throw std::runtime_error("Could not find the revision number");
}

void
ObjectDbCouch::GetRevisionId(RevisionId & revision_id)
{
  json_spirit::mObject params;
  read_json(json_writer_stream_, params);
  revision_id = params["rev"].get_str();
  if (revision_id.empty())
    throw std::runtime_error("Could not find the revision number, from GetRevisionId");
}

void
ObjectDbCouch::Delete(const ObjectId & id, const CollectionName & collection_name)
{
  //TODO
}

void
ObjectDbCouch::Query(const object_recognition::db::View & view, const CollectionName & collection_name, int limit_rows,
                     int start_offset, int& total_rows, int& offset, std::vector<DocumentId> & document_ids)
{
//TODO
}

void
ObjectDbCouch::Query(const std::vector<std::string> & queries, const CollectionName & collection_name, int limit_rows,
                     int start_offset, int& total_rows, int& offset, std::vector<DocumentId> & document_ids)
{
  if (limit_rows <= 0)
    limit_rows = std::numeric_limits<int>::max();
  {
    json_spirit::mObject fields;
    BOOST_FOREACH(const std::string& query, queries)
        {
          fields["map"] = json_spirit::mValue(query);
        }
    json_reader_stream_.str("");
    write_json(fields, json_reader_stream_);
  }
  json_writer_stream_.str("");
  curl_.reset();
  curl_.setReader(&json_reader_);
  curl_.setWriter(&json_writer_);
  std::string url = url_ + "/" + collection_name + "/_temp_view?limit=" + boost::lexical_cast<std::string>(limit_rows)
                    + "&skip="
                    + boost::lexical_cast<std::string>(start_offset);
  curl_.setURL(url);
  curl_.setHeader("Content-Type: application/json");
  curl_.setCustomRequest("POST");
  curl_.perform();

  if (curl_.get_response_code() != object_recognition::curl::cURL::OK)
  {
    throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
  }

  json_reader_stream_.seekg(0);
  json_writer_stream_.seekg(0);
  json_spirit::mObject fields;
  read_json(json_writer_stream_, fields);

  total_rows = fields["total_rows"].get_int();
  document_ids.clear();
  BOOST_FOREACH(json_spirit::mValue & v, fields["rows"].get_array())
      {
        // values are: id, key, value
        document_ids.push_back(v.get_obj()["id"].get_str());
      }
  offset = fields["offset"].get_int() + document_ids.size();
}

void
ObjectDbCouch::CreateCollection(const CollectionName &collection)
{
  json_spirit::mObject params;
  std::string status;
  Status(collection, status);
  std::stringstream ss(status);
  read_json(ss, params);

  json_spirit::mObject::const_iterator iter = params.find("reason");
  if ((params.find("error") != params.end()) && (iter != params.end()) && (iter->second.get_str() == "no_db_file"))
  {
    json_writer_stream_.str("");
    json_reader_stream_.str("");
    curl_.setWriter(&json_writer_);
    curl_.setReader(&json_reader_);
    curl_.setCustomRequest("PUT");
    curl_.perform();
    if (curl_.get_response_code() != object_recognition::curl::cURL::Created)
    {
      throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
    }
    read_json(json_writer_stream_, value);

    iter = params.find("ok");
    if ((iter == params.end()) || (!iter->second.get_bool()))
    {
      std::stringstream ss;
      write_json(params, ss);
      throw std::runtime_error("Could not create to database.\n" + ss.str());
    }
    return;
  }
  iter = params.find("db_name");
  if ((iter == params.end()) || (iter->second.get_str() != collection))
  {
    std::stringstream ss;
    write_json(params, ss);
    throw std::runtime_error("Could not connect to database.\n" + ss.str());
  }
}

void
ObjectDbCouch::Status(std::string& status)
{
  json_writer_stream_.str("");
  json_reader_stream_.str("");
  curl_.setWriter(&json_writer_);
  curl_.setReader(&json_reader_);
  //couch db post to the db
  curl_.setURL(url_);
  curl_.setCustomRequest("GET");
  curl_.perform();
  if (curl_.get_response_code() != object_recognition::curl::cURL::OK)
  {
    throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
  }
  status = json_writer_stream_.str();
}

void
ObjectDbCouch::Status(const CollectionName& collection, std::string& status)
{
  json_writer_stream_.str("");
  json_reader_stream_.str("");
  curl_.setWriter(&json_writer_);
  curl_.setReader(&json_reader_);
  //couch db post to the db
  curl_.setURL(url_ + "/" + collection);
  curl_.setCustomRequest("GET");
  curl_.perform();
  if (curl_.get_response_code() == 0)
  {
    throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
  }
  status = json_writer_stream_.str();
}

void
ObjectDbCouch::DeleteCollection(const CollectionName &collection)
{
  json_spirit::mObject params;
  std::string status;
  Status(collection, status);
  if (curl_.get_response_code() == object_recognition::curl::cURL::OK)
  {
    curl_.setCustomRequest("DELETE");
    curl_.perform();
    if (curl_.get_response_code() != object_recognition::curl::cURL::OK)
    {
      throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
    }
  }
  else if (curl_.get_response_code() != 404)
  {
    throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
  }

}
void
ObjectDbCouch::upload_json(const json_spirit::mObject &ptree, const std::string& url, const std::string& request)
{
  curl_.reset();
  json_writer_stream_.str("");
  json_reader_stream_.str("");
  write_json(ptree, json_reader_stream_);
  curl_.setWriter(&json_writer_);
  curl_.setReader(&json_reader_);
//couch db post to the db
  curl_.setURL(url);
  curl_.setHeader("Content-Type: application/json");
  if (request == "PUT")
  {
    curl_.PUT();
  }
  else
  {
    curl_.setCustomRequest(request.c_str());
  }
  curl_.perform();
}
