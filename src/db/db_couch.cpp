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

object_recognition_core::curl::cURL_GS curl_init_cleanup;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ObjectDbCouch::ObjectDbCouch()
    :
      json_writer_(json_writer_stream_),
      json_reader_(json_reader_stream_)
{
  object_recognition_core::db::ObjectDbParameters parameters(default_raw_parameters());
  this->set_parameters(parameters);
}

ObjectDbParametersRaw
ObjectDbCouch::default_raw_parameters() const
{
  return object_recognition_core::db::ObjectDbDefaults<ObjectDbCouch>::default_raw_parameters();
}

void
ObjectDbCouch::set_parameters(object_recognition_core::db::ObjectDbParameters & parameters) {
  parameters_ = parameters;

  root_ = parameters.at("root").get_str();
  collection_ = parameters.at("collection").get_str();
}

void
ObjectDbCouch::insert_object(const or_json::mObject &fields, DocumentId & document_id, RevisionId & revision_id)
{
  CreateCollection(collection_);
  std::string url = url_id("");
  upload_json(fields, url, "POST");
  GetObjectRevisionId(document_id, revision_id);
}

void
ObjectDbCouch::persist_fields(const DocumentId & document_id, const or_json::mObject &fields, RevisionId & revision_id)
{
  precondition_id(document_id);
  upload_json(fields, url_id(document_id), "PUT");
  //need to update the revision here.
  GetRevisionId(revision_id);
}

void
ObjectDbCouch::load_fields(const DocumentId & document_id, or_json::mObject &fields)
{
  precondition_id(document_id);
  curl_.reset();
  json_writer_stream_.str("");
  curl_.setWriter(&json_writer_);

  curl_.setURL(url_id(document_id));
  curl_.GET();

  curl_.perform();

  if (curl_.get_response_code() != object_recognition_core::curl::cURL::OK)
  {
    throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
  }
  //update the object from the result.
  read_json(json_writer_stream_, fields);
}

void
ObjectDbCouch::set_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                                     const MimeType& mime_type, const std::istream& stream, RevisionId & revision_id)
{
  precondition_id(document_id);
  precondition_rev(revision_id);

  object_recognition_core::curl::reader binary_reader(stream);
  curl_.reset();
  curl_.setReader(&binary_reader);
  json_writer_stream_.str("");
  curl_.setWriter(&json_writer_);
  curl_.setHeader("Content-Type: " + mime_type);
  curl_.setURL(url_id(document_id) + "/" + attachment_name + "?rev=" + revision_id);
  curl_.PUT();
  curl_.perform();
  GetRevisionId(revision_id);
}

void
ObjectDbCouch::get_attachment_stream(const DocumentId & document_id, const RevisionId & revision_id, const std::string& attachment_name,
                                     const std::string& content_type, std::ostream& stream)
{
  object_recognition_core::curl::writer binary_writer(stream);
  curl_.reset();
  json_writer_stream_.str("");
  curl_.setWriter(&binary_writer);
  curl_.setURL(url_id(document_id) + "/" + attachment_name);
  curl_.GET();
  curl_.perform();
  if (curl_.get_response_code() != object_recognition_core::curl::cURL::OK)
  {
    throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
  }
}

void
ObjectDbCouch::GetObjectRevisionId(DocumentId& document_id, RevisionId & revision_id)
{
  or_json::mObject params;
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
  or_json::mObject params;
  read_json(json_writer_stream_, params);
  revision_id = params["rev"].get_str();
  if (revision_id.empty())
    throw std::runtime_error("Could not find the revision number, from GetRevisionId");
}

void
ObjectDbCouch::Delete(const ObjectId & id)
{
  std::string status = Status(collection_ + "/" + id);
  if (curl_.get_response_code() == object_recognition_core::curl::cURL::OK)
  {
    DocumentId document_id;
    RevisionId revision_id;
    {
      or_json::mObject params;
      read_json(json_writer_stream_, params);
      document_id = params["_id"].get_str();
      revision_id = params["_rev"].get_str();
    }

    json_writer_stream_.str("");
    json_reader_stream_.str("");
    curl_.setURL(root_ + "/" + collection_ + "/" + id + "?rev=" + revision_id);
    curl_.setWriter(&json_writer_);
    curl_.setReader(&json_reader_);

    or_json::mObject params;
    params["rev"] = or_json::mValue(revision_id);
    write_json(params, json_reader_stream_);
    curl_.setCustomRequest("DELETE");
    curl_.perform();
    if (curl_.get_response_code() != object_recognition_core::curl::cURL::OK)
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
ObjectDbCouch::QueryView(const object_recognition_core::db::View & view, int limit_rows, int start_offset, int& total_rows,
                     int& offset, std::vector<Document> & view_elements)
{
  json_reader_stream_.str("");
  or_json::mObject parameters = view.parameters();
  std::string url;
  bool do_throw;
  switch (view.type())
  {
    case object_recognition_core::db::View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE:
    {
      url = root_ + "/" + collection_ + "/_design/models/_view/by_object_id_and_" + parameters["model_type"].get_str();
      break;
    }
    case object_recognition_core::db::View::VIEW_OBSERVATION_WHERE_OBJECT_ID:
    {
      url = root_ + "/" + collection_ + "/_design/observations/_view/by_object_id";
      break;
    }
  }

  do_throw = false;

  object_recognition_core::db::View::Key key;
  std::string options;
  if (view.key(key))
    options = "&key=\"" + key.get_str() + "\"";
  QueryView(url, limit_rows, start_offset, options, total_rows, offset, view_elements, do_throw);
}

void
ObjectDbCouch::QueryGeneric(const std::vector<std::string> & queries, int limit_rows, int start_offset, int& total_rows,
                     int& offset, std::vector<Document> & view_elements)
{
  {
    or_json::mObject fields;
    BOOST_FOREACH(const std::string& query, queries)
    {
      fields["map"] = or_json::mValue(query);
    }
    json_reader_stream_.str("");
    write_json(fields, json_reader_stream_);
  }

  QueryView(root_ + "/" + collection_ + "/_temp_view", limit_rows, start_offset, "", total_rows, offset, view_elements,
            true);
}

/** Once json_reader_stream_ has been filled, call that function to get the results of the view
 *
 */
void
ObjectDbCouch::QueryView(const std::string & in_url, int limit_rows, int start_offset, const std::string &options,
                         int& total_rows, int& offset, std::vector<Document> & view_elements, bool do_throw)
{
  if (limit_rows <= 0)
    limit_rows = std::numeric_limits<int>::max();
  json_writer_stream_.str("");
  curl_.reset();
  curl_.setReader(&json_reader_);
  curl_.setWriter(&json_writer_);
  std::string url = in_url + "?limit=" + boost::lexical_cast<std::string>(limit_rows) + "&skip="
                    + boost::lexical_cast<std::string>(start_offset) + options;

  curl_.setURL(url);
  curl_.setHeader("Content-Type: application/json");
  curl_.setCustomRequest("GET");
  curl_.perform();

  if (curl_.get_response_code() != object_recognition_core::curl::cURL::OK)
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
  view_elements.clear();
  view_elements.reserve(fields["rows"].get_array().size());
  BOOST_FOREACH(or_json::mValue & v, fields["rows"].get_array())
  {
    // values are: id, key, value
    const or_json::mObject & object = v.get_obj();
    Document doc;
    doc.SetIdRev(object.find("id")->second.get_str(), object.find("key")->second.get_str());
    view_elements.push_back(doc);
    view_elements.back().set_fields(object.find("value")->second.get_obj());
  }
  offset = fields["offset"].get_int() + view_elements.size();
}

void
ObjectDbCouch::CreateCollection(const CollectionName &collection)
{
  or_json::mObject params;
  std::string status = Status(collection);
  std::stringstream ss(status);
  read_json(ss, params);

  or_json::mObject::const_iterator iter = params.find("reason");
  if ((params.find("error") != params.end()) && (iter != params.end()) && (iter->second.get_str() == "no_db_file"))
  {
    json_writer_stream_.str("");
    json_reader_stream_.str("");
    curl_.setWriter(&json_writer_);
    curl_.setReader(&json_reader_);
    curl_.setCustomRequest("PUT");
    curl_.perform();
    if (curl_.get_response_code() != object_recognition_core::curl::cURL::Created)
    {
      throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
    }
    read_json(json_writer_stream_, params);

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

std::string
ObjectDbCouch::Status() const
{
  json_writer_stream_.str("");
  json_reader_stream_.str("");
  curl_.setWriter(&json_writer_);
  curl_.setReader(&json_reader_);
  //couch db post to the db
  curl_.setURL(root_);
  curl_.setCustomRequest("GET");
  curl_.perform();
  if (curl_.get_response_code() != object_recognition_core::curl::cURL::OK)
  {
    throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
  }
  return json_writer_stream_.str();
}

std::string
ObjectDbCouch::Status(const CollectionName& collection) const
{
  json_writer_stream_.str("");
  json_reader_stream_.str("");
  curl_.setWriter(&json_writer_);
  curl_.setReader(&json_reader_);
  //couch db post to the db
  curl_.setURL(root_ + "/" + collection);
  curl_.setCustomRequest("GET");
  curl_.perform();
  if (curl_.get_response_code() == 0)
  {
    throw std::runtime_error(curl_.get_response_reason_phrase() + " : " + curl_.getURL());
  }
  return json_writer_stream_.str();
}

void
ObjectDbCouch::DeleteCollection(const CollectionName &collection)
{
  std::string status = Status(collection);
  if (curl_.get_response_code() == object_recognition_core::curl::cURL::OK)
  {
    curl_.setCustomRequest("DELETE");
    curl_.perform();
    if (curl_.get_response_code() != object_recognition_core::curl::cURL::OK)
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
ObjectDbCouch::upload_json(const or_json::mObject &params, const std::string& url, const std::string& request)
{
  curl_.reset();
  json_writer_stream_.str("");
  json_reader_stream_.str("");
  write_json(params, json_reader_stream_);
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

DbType
ObjectDbCouch::type() const {
  return object_recognition_core::db::ObjectDbDefaults<ObjectDbCouch>::type();
}
