#include <objcog/db/couch.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/progress.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

#include <curl/curl.h>

#include <json_spirit/json_spirit.h>

#include <streambuf>
#include <string>
#include <cstdio>

namespace
{
  std::string
  type2str(json_spirit::Value_type t)
  {
    using namespace json_spirit;
    switch (t)
    {
      case obj_type:
        return "obj_type";
        break;
      case array_type:
        return "array_type";
        break;
      case str_type:
        return "str_type";
        break;
      case bool_type:
        return "bool_type";
        break;
      case int_type:
        return "int_type";
        break;
      case real_type:
        return "real_type";
        break;
      case null_type:
        return "null_type";
        break;
      default:
        return "unknown type";
    }
  }
  struct writer
  {
    std::ostream& stream;
    writer(std::ostream& stream)
        :
          stream(stream)
    {
    }

    static size_t
    cb(char *ptr, size_t size, size_t nmemb, void *userdata)
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

    reader(std::istream& stream)
        :
          stream(stream)
    {
    }

    /**
     * static callback for c style void pointer cookies.
     */
    static size_t
    cb(char *ptr, size_t size, size_t nmemb, void *thiz)
    {
      if (!thiz)
      {
        return 0;
      }
      reader* data = static_cast<reader*>(thiz);
      return data->stream.rdbuf()->sgetn(ptr, size * nmemb);
    }
  };

  struct cURL: boost::noncopyable
  {
    enum HTTP_CODES
    {
      Continue = 100, OK = 200, Created = 201, Accepted = 202, BadRequest = 400
    };
    cURL()
        :
          curl_(curl_easy_init()),
          headers_(0),
          header_writer_(header_writer_stream_)
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

    void
    perform()
    {
      //need to reset stream.
      header_writer_stream_.str("");
      curl_easy_perform(curl_);
      parse_response_header();
    }

    void
    setURL(const std::string& url)
    {
      curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
    }

    void
    setHeader(const std::string& header)
    {
      headers_ = curl_slist_append(headers_, header.c_str());
      curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers_);
    }

    void
    setCustomRequest(const char* request_type)
    {
      curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, request_type);
    }
    void
    setWriter(writer* w)
    {
      curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, &writer::cb);
      curl_easy_setopt(curl_, CURLOPT_WRITEDATA, w);
    }
    void
    setReader(reader* r)
    {
      curl_easy_setopt(curl_, CURLOPT_READFUNCTION, &reader::cb);
      curl_easy_setopt(curl_, CURLOPT_READDATA, r);
      curl_easy_setopt(curl_, CURLOPT_UPLOAD, 1L);
    }
    void
    PUT()
    {
      curl_easy_setopt(curl_, CURLOPT_PUT, 1L);
    }

    void
    POST()
    {
      curl_easy_setopt(curl_, CURLOPT_POST, 1L);
    }

    void
    GET()
    {
      curl_easy_setopt(curl_, CURLOPT_HTTPGET, 1L);
    }

    void
    HEAD()
    {
      curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "HEAD");
    }
    void
    DELETE()
    {
      curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "DELETE");
    }
    void
    reset()
    {
      curl_slist_free_all(headers_);
      curl_easy_reset(curl_);
      headers_ = 0;
      curl_easy_setopt(curl_, CURLOPT_HEADERFUNCTION, &writer::cb);
      curl_easy_setopt(curl_, CURLOPT_HEADERDATA, &header_writer_);
    }

    int
    get_response_code() const
    {
      return response_status_code_;
    }
    const std::string&
    get_response_reason_phrase() const
    {
      return response_reason_phrase_;
    }

    const std::string&
    get_response_header(const std::string& header_name) const
    {
      if (header_response_values.count(header_name))
        return header_response_values.find(header_name)->second;
      else
        throw std::runtime_error(header_name + " does not exist");
    }

  private:
    void
    parse_response_header()
    {
      //parse codes
      std::string _, reason, x;
      do
      {
        header_writer_stream_ >> _ /*used to eatup the http version.*/
        >> response_status_code_;
        std::getline(header_writer_stream_, response_reason_phrase_, '\n');
        if (!response_reason_phrase_.empty())
          response_reason_phrase_.resize(response_reason_phrase_.size() - 1);
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
        if (!line.empty())
          line.resize(line.size() - 1);
        std::cout << headerX << ">==<" << line << "\n";
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

  struct value_type_find
  {
    std::string name;
    json_spirit::Value_type t;
    value_type_find(const std::string& name, json_spirit::Value_type t)
        :
          name(name),
          t(t)
    {

    }
    bool
    operator()(const json_spirit::Pair& v)
    {
      return v.value_.type() == t && v.name_ == name;
    }
  };

  template<typename T>
  struct val2
  {
    T
    operator()(const json_spirit::Value& val)
    {
      return val.get_value<T>();
    }
  };
}
namespace couch
{
  struct Db::impl
  {

    impl(const std::string& url)
        :
          url_(url),
          json_writer_(stream_)
    {
      curl_.setURL(url);
      curl_.setWriter(&json_writer_);
    }

    impl(const impl& rhs)
        :
          url_(rhs.url_),
          json_writer_(stream_)
    {
    }

    impl&
    operator=(const impl& rhs)
    {
      if (&rhs == this)
        return *this;
      url_ = rhs.url_;
      return *this;

    }

    bool
    create()
    {
      stream_.str("");
      curl_.PUT();
      curl_.perform();
      //On success, HTTP status 201 is returned. If a database already exists a 412 error is returned.
      switch (curl_.get_response_code())
      {
        case 201:
          return true;
        case 412:
          return false;
        default:
          throw std::runtime_error("Unexpected result: " + curl_.get_response_reason_phrase());
      }
    }

    void
    update_info()
    {
      stream_.str("");
      curl_.reset();
      curl_.setURL(url_);
      curl_.setWriter(&json_writer_);
      curl_.GET();
      curl_.perform();
      //On success, HTTP status 200 is returned. If the database doesn't exist, a 404 error is returned.
      switch (curl_.get_response_code())
      {
        case 200:
        {
          json_spirit::Value val;
          get(val);
          json_spirit::obj_to_map(val.get_obj(), db_info_);
          break;
        }
        default:
          throw std::runtime_error("Unexpected result: " + curl_.get_response_reason_phrase());
      }
    }
    void
    print_info(std::ostream& out)
    {
      json_spirit::Object obj;
      json_spirit::map_to_obj(db_info_, obj);
      json_spirit::write_formatted(obj, out);
    }
    template<typename T>
    T
    get_info_item(const std::string& item)
    {
      return db_info_[item].get_value<T>();
    }
    bool
    delete_()
    {
      stream_.str("");
      curl_.reset();
      curl_.setURL(url_);
      curl_.setWriter(&json_writer_);
      curl_.DELETE();
      curl_.perform();
      //On success, HTTP status 200 is returned. If the database doesn't exist, a 404 error is returned.
      switch (curl_.get_response_code())
      {
        case 200:
          return true;
        case 404:
          return false;
        default:
          throw std::runtime_error("Unexpected result: " + curl_.get_response_reason_phrase());
      }

    }

    void
    get(json_spirit::Value& val)
    {
      stream_.seekg(0);
      json_spirit::read(stream_, val);
    }

    void
    print()
    {
      stream_.seekg(0);
      std::cout << stream_.rdbuf();
    }

    void
    run_view(View& v, int limit_rows, int start_offset, int& total_rows, int& offset,
             std::vector<couch::Document>& docs)
    {
      if (limit_rows <= 0)
        limit_rows = std::numeric_limits<int>::max();
      json_spirit::Object obj;
      typedef std::pair<std::string, std::string> value;
      BOOST_FOREACH(const value& p, v.map)
          {
            obj.push_back(json_spirit::Pair("map", p.second));
          }
      std::stringstream stream;
      json_spirit::write(obj, stream);
      json_spirit::write(obj, std::cout);
      reader r(stream);
      stream_.str("");
      curl_.reset();
      curl_.setReader(&r);
      curl_.setWriter(&json_writer_);
      curl_.setURL(
          url_ + "/_temp_view?limit=" + boost::lexical_cast<std::string>(limit_rows) + "&skip="
          + boost::lexical_cast<std::string>(start_offset));
      curl_.setHeader("Content-Type: application/json");
      curl_.setCustomRequest("POST");
      curl_.perform();
      json_spirit::Value val;
      get(val);
      std::map<std::string, json_spirit::Value> result_map;
      json_spirit::obj_to_map(val.get_obj(), result_map);
      total_rows = result_map["total_rows"].get_int();
      offset = result_map["offset"].get_int();
      std::vector<json_spirit::Value> rows = result_map["rows"].get_array();
      Db db(url_);
      BOOST_FOREACH(const json_spirit::Value& v, rows)
          {
            std::map<std::string, json_spirit::Value> row_map;
            json_spirit::obj_to_map(v.get_obj(), row_map);
            Document doc(db, row_map["id"].get_str());
            docs.push_back(doc);
          }

    }

    cURL curl_;
    std::string url_;
    std::stringstream stream_;
    writer json_writer_;
    std::map<std::string, json_spirit::Value> db_info_;

  };

  struct Document::impl
  {
    impl(const Db& db, const std::string& id)
        :
          id_(id),
          db_url_(db.url()),
          json_writer_(json_writer_stream_),
          json_reader_(json_reader_stream_)
    {
    }

    impl(const impl& rhs)
        :
          id_(rhs.id_),
          db_url_(rhs.db_url_),
          json_writer_(json_writer_stream_),
          json_reader_(json_reader_stream_)
    {
      obj_ = rhs.obj_;
    }
    impl&
    operator=(const impl& rhs)
    {
      if (&rhs == this)
        return *this;
      id_ = rhs.id_;
      db_url_ = rhs.db_url_;
      obj_ = rhs.obj_;
      return *this;

    }

    void
    create()
    {
      std::string url = url_id();
      if (id_.empty())
      {
        upload_json(url, "POST");
      }
      else
      {
        update();
        if (!getid())
        {
          upload_json(url, "PUT"); //create it since it doesn't exist.
        }
      }
      if (!getid())
        throw std::runtime_error("Failed to get id: " + url);
    }

    void
    whoami()
    {
      std::cout << "url:" << url_id_rev() << "\n";
    }

    std::string
    toString()
    {
      json_spirit::Object obj;
      json_spirit::map_to_obj(obj_, obj);
      return json_spirit::write_formatted(obj);
    }

    void
    print_result(std::ostream& out)
    {
      json_writer_stream_.seekg(0);
      out << json_writer_stream_.rdbuf();
    }

    void
    update()
    {
      precondition_id();
      curl_.reset();
      json_writer_stream_.str("");
      curl_.setWriter(&json_writer_);

      curl_.setURL(url_id());
      curl_.GET();

      curl_.perform();

      if (curl_.get_response_code() == cURL::OK)
      {
        //update the object from the result.
        json_spirit::Value val;
        get(val);
        json_spirit::obj_to_map(val.get_obj(), obj_);
      }
    }

    void
    commit()
    {
      precondition_id();
      precondition_rev();
      upload_json(url_id(), "PUT");
      //need to update the revision here.
      getid();
    }

    void
    clear_all()
    {
      obj_.clear();
    }

    void
    clear_value(const std::string& key)
    {
      obj_.erase(key);
    }

    template<typename T>
    T
    get_value(const std::string& key)
    {
      return obj_[key].get_value<T>();
    }

    template<typename T>
    void
    set_value(const std::string& key, const T& val)
    {
      typedef std::map<std::string, json_spirit::Value>::iterator iterator;
      json_spirit::Value jval(val);
      std::pair<iterator, bool> it_inserted = obj_.insert(std::make_pair(key, jval));
      if (!it_inserted.second)
        it_inserted.first->second = jval;
    }

    void
    attach(const std::string& attachment_name, std::istream& stream, const std::string& content_type)
    {
      reader binary_reader(stream);
      curl_.reset();
      curl_.setReader(&binary_reader);
      json_writer_stream_.str("");
      curl_.setWriter(&json_writer_);
      curl_.setHeader("Content-Type: " + content_type);
      curl_.setURL(url_id() + "/" + attachment_name + "?rev=" + rev_);
      curl_.PUT();
      curl_.perform();
      getid();
    }

    void
    get_attachment_stream(const std::string& attachment_name, std::ostream& stream)
    {
      writer binary_writer(stream);
      curl_.reset();
      json_writer_stream_.str("");
      curl_.setWriter(&binary_writer);
      curl_.setURL(url_id() + "/" + attachment_name);
      curl_.GET();
      curl_.perform();
    }
  private:

    void
    get(json_spirit::Value& val)
    {
      json_writer_stream_.seekg(0);
      json_spirit::read(json_writer_stream_, val);
    }
    bool
    getid(const std::string& prefix = "")
    {
      json_spirit::Value val;
      get(val);
      if(val.is_null()) return false;
      const json_spirit::Object& obj = val.get_obj();
      json_spirit::Object::const_iterator it = std::find_if(obj.begin(), obj.end(),
                                                            value_type_find(prefix + "id", json_spirit::str_type));
      json_spirit::Object::const_iterator rit = std::find_if(obj.begin(), obj.end(),
                                                             value_type_find(prefix + "rev", json_spirit::str_type));
      if (it != obj.end() && rit != obj.end())
      {
        setIdRev(it->value_.get_str(), rit->value_.get_str());
        return true;
      }
      else
      {
        return false;
      }

    }

    void
    setIdRev(const std::string& id, const std::string& rev)
    {
      id_ = id;
      rev_ = rev;
      set_value("_id", id_);
      set_value("_rev", rev_);
    }
    void
    precondition_id()
    {
      if (id_.empty())
        throw std::runtime_error("The document's id must be initialized.");

    }

    void
    precondition_rev()
    {
      if (rev_.empty())
        throw std::runtime_error("The document must have a valid revision.");
    }
    void
    upload_json(const std::string& url, const std::string& request)
    {
      curl_.reset();
      json_writer_stream_.str("");
      json_reader_stream_.str("");
      json_spirit::Object obj;
      json_spirit::map_to_obj(obj_, obj);
      json_spirit::write(obj, json_reader_stream_);
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
    inline std::string
    url_id()
    {
      return db_url_ + (id_.empty() ? "" : "/" + id_);
    }
    inline std::string
    url_id_rev()
    {
      return url_id() + "?rev=" + rev_;
    }

    std::map<std::string, json_spirit::Value> obj_;
    std::string id_, rev_;
    std::string db_url_;
    std::stringstream json_writer_stream_, json_reader_stream_;
    writer json_writer_;
    reader json_reader_;
    cURL curl_;
  };
}

void
couch::Db::all_dbs(const std::string& couch_db_root_url, std::vector<std::string> & dbs)
{
  dbs.clear();
  cURL curl;
  std::stringstream stream;
  writer json_writer(stream);
  curl.setURL(couch_db_root_url + "/_all_dbs");
  curl.setWriter(&json_writer);
  curl.GET();
  curl.perform();
  if (curl.get_response_code() == cURL::OK)
  {
    json_spirit::Value val;
    json_spirit::read(stream, val);
    json_spirit::Array array = val.get_array();
    dbs.reserve(array.size());
    std::transform(array.begin(), array.end(), std::back_inserter(dbs), val2<std::string>());
  }
  else
    throw std::runtime_error("Could not connect to couchdb at : " + couch_db_root_url);
}

bool
couch::Db::create()
{
  return impl_->create();
}

void
couch::Db::update_info()
{
  impl_->update_info();
}

void
couch::Db::print_info(std::ostream & out)
{
  impl_->print_info(out);
}

std::string
couch::Db::url() const
{
  return impl_->url_;
}

couch::Db::Db(const std::string& url)
    :
      impl_(new couch::Db::impl(url))
{
}

void
couch::Db::print()
{
  impl_->print();
}

bool
couch::Db::delete_()
{
  return impl_->delete_();
}

couch::Document::Document(const Db & db, const std::string& id)
    :
      impl_(new impl(db, id))
{
}

couch::Document::Document(const Document& rhs)
    :
      impl_(new impl(*rhs.impl_))
{

}
couch::Document&
couch::Document::operator=(const Document& rhs)
{
  if (&rhs == this)
    return *this;
  if (impl_.get() == 0)
    impl_.reset(new impl(*rhs.impl_));
  else
    *impl_ = *rhs.impl_;
  return *this;
}

couch::Db::Db(const Db& rhs)
    :
      impl_(new impl(*rhs.impl_))
{

}
couch::Db&
couch::Db::operator=(const Db& rhs)
{
  if (&rhs == this)
    return *this;
  if (impl_.get() == 0)
    impl_.reset(new impl(*rhs.impl_));
  else
    *impl_ = *rhs.impl_;
  return *this;
}

void
couch::Document::whoami()
{
  impl_->whoami();
}

std::string
couch::Document::toString()
{
  return impl_->toString();
}

void
couch::Document::print_result(std::ostream & out)
{
  return impl_->print_result(out);
}

void
couch::Document::update()
{
  impl_->update();
}

void
couch::Document::create()
{
  impl_->create();
}

void
couch::Document::commit()
{
  impl_->commit();
}

void
couch::Document::clear_all()
{
  impl_->clear_all();
}

void
couch::Document::clear_value(const std::string& key)
{
  impl_->clear_value(key);
}

void
couch::Document::attach(const std::string& attachment_name, std::istream & stream, const std::string& content_type)
{
  impl_->attach(attachment_name, stream, content_type);
}

void
couch::Document::get_attachment_stream(const std::string& attachment_name, std::ostream & stream)
{
  impl_->get_attachment_stream(attachment_name, stream);
}

template<>
bool
couch::Document::get_value<bool>(const std::string& key)
{
  return impl_->get_value<bool>(key);
}
template<>
int
couch::Document::get_value<int>(const std::string& key)
{
  return impl_->get_value<int>(key);
}
template<>
double
couch::Document::get_value<double>(const std::string& key)
{
  return impl_->get_value<double>(key);
}
template<>
std::string
couch::Document::get_value<std::string>(const std::string& key)
{
  return impl_->get_value<std::string>(key);
}

template<>
void
couch::Document::set_value<bool>(const std::string& key, const bool& val)
{
  impl_->set_value(key, val);
}
template<>
void
couch::Document::set_value<int>(const std::string& key, const int& val)
{
  impl_->set_value(key, val);
}
template<>
void
couch::Document::set_value<double>(const std::string& key, const double& val)
{
  impl_->set_value(key, val);
}

template<>
void
couch::Document::set_value<std::string>(const std::string& key, const std::string& val)
{
  impl_->set_value(key, val);
}

void
couch::Document::set_value(const std::string& key, const char* val)
{
  impl_->set_value(key, val);
}

template<>
void
couch::Document::set_value<std::vector<std::string> >(const std::string& key, const std::vector<std::string>& val)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  json_spirit::Array array;
  for (size_t i = 0; i < val.size(); i++)
  {
    std::cout << val[i] << std::endl;
    json_spirit::Value v(val[i]);
    array.push_back(v);
  }
  impl_->set_value(key, array);
}

template<>
std::vector<std::string>
couch::Document::get_value<std::vector<std::string> >(const std::string& key)
{

  json_spirit::Array array = impl_->get_value<json_spirit::Array>(key);
  std::vector<std::string> vec;
  for (size_t i = 0; i < array.size(); i++)
  {
    vec.push_back(array[i].get_str());
  }
  return vec;
}
template<>
bool
couch::Db::get_info_item<bool>(const std::string& item)
{
  return impl_->get_info_item<bool>(item);
}
template<>
int
couch::Db::get_info_item<int>(const std::string& item)
{
  return impl_->get_info_item<int>(item);
}
template<>
double
couch::Db::get_info_item<double>(const std::string& item)
{
  return impl_->get_info_item<double>(item);
}
template<>
std::string
couch::Db::get_info_item<std::string>(const std::string& item)
{
  return impl_->get_info_item<std::string>(item);
}

couch::View::View()
{
}

void
couch::View::add_map(const std::string & key, const std::string & function)
{
  map[key] = function;
}

void
couch::View::add_reduce(const std::string & key, const std::string & function)
{
  reduce[key] = function;
}

void
couch::Db::run_view(View & v, int limit_rows, int start_offset, int& total_rows, int& offset,
                    std::vector<couch::Document>& docs)
{
  impl_->run_view(v, limit_rows, start_offset, total_rows, offset, docs);
}

