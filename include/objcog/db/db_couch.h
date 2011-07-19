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

#include <curl/curl.h>

#include "db_base.h"

#ifndef DB_COUCH_H_
#define DB_COUCH_H_

namespace
{
struct writer
{
  std::ostream& stream;
  writer(std::ostream& stream) :
      stream(stream)
  {
  }

  static size_t cb(char *ptr, size_t size, size_t nmemb, void *userdata)
  {
    if (!userdata)
    {
      return 0;
    }

    writer* data = static_cast<writer*>(userdata);
    //data->written +=size*nmemb;
    data->stream.write(ptr, size * nmemb);
    //std::cout << "**"<< data->written << std::endl;
    return size * nmemb;
  }

  // size_t written;

};

struct reader
{
  std::istream& stream;

  reader(std::istream& stream) :
      stream(stream)
  {
  }

  /**
   * static callback for c style void pointer cookies.
   */
  static size_t cb(char *ptr, size_t size, size_t nmemb, void *thiz)
  {
    if (!thiz)
    {
      return 0;
    }
    reader* data = static_cast<reader*>(thiz);
    return data->stream.rdbuf()->sgetn(ptr, size * nmemb);
  }
};

struct cURL : boost::noncopyable
{
  enum HTTP_CODES
  {
    Continue = 100, OK = 200, Created = 201, Accepted = 202, BadRequest = 400
  };
  cURL() :
      curl_(curl_easy_init()), headers_(0), header_writer_(header_writer_stream_)
  {
    if (curl_ == 0)
      throw std::runtime_error("Unable to connect CURL.");
    reset();
  }

  ~cURL()
  {
    curl_slist_free_all(headers_);
    curl_easy_cleanup(curl_);
  }

  void perform()
  {
    //need to reset stream.
    header_writer_stream_.str("");
    curl_easy_perform(curl_);
    parse_response_header();
  }

  void setURL(const std::string& url)
  {
    curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
  }

  void setHeader(const std::string& header)
  {
    headers_ = curl_slist_append(headers_, header.c_str());
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers_);
  }

  void setCustomRequest(const char* request_type)
  {
    curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, request_type);
  }
  void setWriter(writer* w)
  {
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, &writer::cb);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, w);
  }
  void setReader(reader* r)
  {
    curl_easy_setopt(curl_, CURLOPT_READFUNCTION, &reader::cb);
    curl_easy_setopt(curl_, CURLOPT_READDATA, r);
    curl_easy_setopt(curl_, CURLOPT_UPLOAD, 1L);
  }
  void PUT()
  {
    curl_easy_setopt(curl_, CURLOPT_PUT, 1L);
  }

  void POST()
  {
    curl_easy_setopt(curl_, CURLOPT_POST, 1L);
  }

  void GET()
  {
    curl_easy_setopt(curl_, CURLOPT_HTTPGET, 1L);
  }

  void HEAD()
  {
    curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "HEAD");
  }
  void DELETE()
  {
    curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "DELETE");
  }
  void reset()
  {
    curl_slist_free_all(headers_);
    curl_easy_reset(curl_);
    headers_ = 0;
    curl_easy_setopt(curl_, CURLOPT_HEADERFUNCTION, &writer::cb);
    curl_easy_setopt(curl_, CURLOPT_HEADERDATA, &header_writer_);
  }

  int get_response_code() const
  {
    return response_status_code_;
  }
  const std::string& get_response_reason_phrase() const
  {
    return response_reason_phrase_;
  }

  const std::string& get_response_header(const std::string& header_name) const
  {
    if (header_response_values.count(header_name))
      return header_response_values.find(header_name)->second;
    else
      throw std::runtime_error(header_name + " does not exist");
  }

private:
  void parse_response_header()
  {
    //parse codes
    std::string _, reason, x;
    do
    {
      header_writer_stream_ >> _ /*used to eatup the http version.*/
      >> response_status_code_;
      std::getline(header_writer_stream_, response_reason_phrase_, '\n');
      response_reason_phrase_.resize(response_reason_phrase_.size() - 1);
      //          std::cout << "code: " << response_status_code_ << " reason: "
      //              << response_reason_phrase_ << "\n";
    } while (response_status_code_ == Continue); //handle continuecontinue

    header_response_values.clear();
    while (true)
    {
      std::string headerX, line;
      std::getline(header_writer_stream_, headerX, ':'); //eats the colon
      if (header_writer_stream_.eof())
      {
        header_writer_stream_.clear();
        break;
      }
      header_writer_stream_.ignore(1, ' '); //eatup the space.
      std::getline(header_writer_stream_, line, '\n');
      line.resize(line.size() - 1);
      //std::cout << headerX << ">==<" << line << "\n";
      header_response_values[headerX] = line;
    }
  }
  CURL * curl_;
  curl_slist *headers_;
  std::stringstream header_writer_stream_;
  writer header_writer_;
  int response_status_code_;
  std::string response_reason_phrase_;
  std::map<std::string, std::string> header_response_values;

};
struct cURL_GS
{
  cURL_GS()
  {
    //std::cout << "curl init" << std::endl;
    curl_global_init(CURL_GLOBAL_ALL);
  }
  ~cURL_GS()
  {
    //std::cout << "curl cleanup" << std::endl;
    curl_global_cleanup();
  }
};

cURL_GS curl_init_cleanup;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ObjectDbCouch : public ObjectDbBase
{
public:
  ObjectDbCouch(const std::string &url) :
      url_(url), json_writer_(json_writer_stream_), json_reader_(json_reader_stream_)
  {
  }

  virtual void insert_object(const CollectionName &collection, const boost::property_tree::ptree &fields,
                             ObjectId & object_id, RevisionId & revision_id)
  {
    upload_json(fields, url_id(""), "POST");
    getid(object_id, revision_id);
  }

  virtual void persist_fields(ObjectId & object_id, RevisionId & revision_id, const CollectionName &collection,
                              const boost::property_tree::ptree &fields)
  {
    precondition_id(object_id);
    precondition_rev(revision_id);
    upload_json(fields, url_id(object_id), "PUT");
    //need to update the revision here.
    getid(object_id, revision_id);
  }

  virtual void load_fields(const ObjectId & object_id, const CollectionName &collection,
                           boost::property_tree::ptree &fields)
  {
    precondition_id(object_id);
    curl_.reset();
    curl_.setWriter(&json_writer_);

    curl_.setURL(url_id(object_id));
    curl_.GET();
    curl_.perform();

    // TODO go from the json_writer_ to filling the property_tree
  }

  virtual void query(const CollectionName &collection, const std::map<FieldName, std::string> &regexps
                     , std::vector<ObjectId> & object_ids) const
  {
  }

  void set_attachment_stream(ObjectId & object_id, RevisionId & revision_id, const CollectionName &collection,
                             const std::string& attachment_name, std::istream& stream, const std::string& content_type)
  {
    reader binary_reader(stream);
    curl_.reset();
    curl_.setReader(&binary_reader);
    json_writer_stream_.str("");
    curl_.setWriter(&json_writer_);
    curl_.setHeader("Content-Type: " + content_type);
    curl_.setURL(url_id(object_id) + "/" + attachment_name + "?rev=" + revision_id);
    curl_.PUT();
    curl_.perform();
    getid(object_id, revision_id);
  }

  void get_attachment_stream(const std::string& attachment_name, std::ostream& stream)
  {
    writer binary_writer(stream);
    curl_.reset();
    json_writer_stream_.str("");
    curl_.setWriter(&binary_writer);
    curl_.setURL(url_id("") + "/" + attachment_name);
    curl_.GET();
    curl_.perform();
  }

  void getid(std::string & object_id, std::string & revision_id, const std::string& prefix = "")
  {
    boost::property_tree::ptree params;
    boost::property_tree::read_json(json_writer_stream_, params);
    object_id = params.get<std::string>("id", "");
    revision_id = params.get<std::string>("rev", "");
    if ((object_id.empty()) || (revision_id.empty()))
      throw std::runtime_error("Could not find the id or revision number");
  }

  void query(const CollectionName &collection, const std::map<FieldName, std::string> &regexps
             , std::vector<ObjectId> & object_ids)
  {
  }
private:

  void precondition_id(const ObjectId & id) const
  {
    if (id.empty())
      throw std::runtime_error("The document's id must be initialized.");

  }

  void precondition_rev(const RevisionId & rev) const
  {
    if (rev.empty())
      throw std::runtime_error("The document must have a valid revision.");
  }
  void upload_json(const boost::property_tree::ptree &ptree, const std::string& url, const std::string& request)
  {
    curl_.reset();
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
  inline std::string url_id(const ObjectId & id) const
  {
    return url_ + (id.empty() ? "" : "/" + id);
  }
  inline std::string url_id_rev(const ObjectId & id, const RevisionId & rev) const
  {
    return url_id(id) + "?rev=" + rev;
  }

  /** The URL of the DB, including port */
  std::string url_;
  mutable cURL curl_;
  mutable std::stringstream json_writer_stream_, json_reader_stream_;

  writer json_writer_;
  reader json_reader_;
};

#endif /* DB_COUCH_H_ */
