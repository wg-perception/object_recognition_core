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
{
  object_recognition_core::db::ObjectDbParameters parameters(default_raw_parameters());
  this->set_parameters(parameters);
}

void
ObjectDbFilesystem::set_parameters(object_recognition_core::db::ObjectDbParameters & parameters) {
  parameters_ = parameters;

  path_ = parameters.at("path").get_str();
  collection_ = parameters.at("collection").get_str();
}

ObjectDbParametersRaw
ObjectDbFilesystem::default_raw_parameters() const
{
  return object_recognition_core::db::ObjectDbDefaults<ObjectDbFilesystem>::default_raw_parameters();
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

  // TODO update all the views

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
  boost::filesystem::create_directories(url_attachments(document_id));
  boost::filesystem::path path = url_attachments(document_id) / attachment_name;
  std::ofstream file(path.string().c_str(), std::ios::binary);
  {
    std::istream & un_const_stream = const_cast<std::istream &>(stream);
    size_t stream_position = un_const_stream.tellg();
    un_const_stream.seekg(0);
    file << un_const_stream.rdbuf();
    un_const_stream.seekg(stream_position);
  }
  file.close();

  // TODO use MIME type
  std::cout << path.string() << std::endl;

  revision_id = DEFAULT_REVISION_ID_;
}

void
ObjectDbFilesystem::get_attachment_stream(const DocumentId & document_id, const RevisionId & revision_id, const std::string& attachment_name,
                                          const std::string& content_type, std::ostream& stream)
{
  // Write the stream to a file
  boost::filesystem::path path = url_attachments(document_id) / attachment_name;
  std::ifstream file(path.string().c_str(), std::ios::binary);
  stream << file.rdbuf();
  file.close();
}

void
ObjectDbFilesystem::Delete(const DocumentId & id)
{
  boost::filesystem::remove_all(url_id(id));

  // For each pre-defined view, figure out the potential keys, and delete those
  BOOST_FOREACH(const object_recognition_core::db::View::ViewType & view_type, object_recognition_core::db::View::AllViewTypes())
  {
    object_recognition_core::db::View view(view_type);
    // TODO read the document from the file
    or_json::mObject document;
    or_json::mValue key;
    or_json::mValue value;
    if (view.GetKey(document, key, value))
    {
      // Delete the element from the view
    }
  }
}

void
ObjectDbFilesystem::QueryView(const object_recognition_core::db::View & view, int limit_rows, int start_offset,
                          int& total_rows, int& offset, std::vector<Document> & view_elements)
{
  or_json::mObject parameters = view.parameters();
  boost::filesystem::path path;
  switch (view.type())
  {
    case object_recognition_core::db::View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE:
    {
      path = path_ / collection_ / "view" / "designdoc1"
             / std::string("by_object_id_and_" + parameters["model_type"].get_str());
      // TODO go over the symlinks in the folder and return the ids
      throw std::runtime_error("Function not implemented in the Filesystem DB.");
      break;
    }
    case object_recognition_core::db::View::VIEW_OBSERVATION_WHERE_OBJECT_ID:
    {
      throw std::runtime_error("Function not implemented in the Filesystem DB.");
      break;
    }
  }
}

void
ObjectDbFilesystem::QueryGeneric(const std::vector<std::string> & queries, int limit_rows, int start_offset, int& total_rows,
                          int& offset, std::vector<Document> & view_elements)
{
  throw std::runtime_error("Function not implemented in the Filesystem DB.");
}

void
ObjectDbFilesystem::CreateCollection(const CollectionName &collection)
{
  std::string status;
  Status(status);
  boost::filesystem::create_directories(path_ / collection);
}

std::string
ObjectDbFilesystem::Status() const
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
ObjectDbFilesystem::Status(const CollectionName& collection) const
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
  {
    // First, figure out all the DocumentId's in the collection
    //TODO
    // For each, delete it (which will also delete it from the views)
    //TODO
    // Delete the folder infrastructure
    boost::filesystem::remove_all(path_ / collection);
  }
}

DbType
ObjectDbFilesystem::type() const
{
  return object_recognition_core::db::ObjectDbDefaults<ObjectDbFilesystem>::type();
}
