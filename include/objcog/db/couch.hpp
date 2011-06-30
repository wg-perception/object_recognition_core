#pragma once
#include <map>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#define DEFAULT_COUCHDB_URL "http://localhost:5984"
namespace couch
{
  class Document;
  struct View
  {
    View();
    void add_map(const std::string& key, const std::string& function);
    void add_reduce(const std::string& key, const std::string& function);
    std::map<std::string, std::string> map, reduce;
  };

  class Db
  {
  public:
    static void all_dbs(const std::string& couch_db_root_url,
                        std::vector<std::string>& dbs);
    explicit Db(const std::string& url);
    Db(const Db& rhs);
    Db(){}
    Db& operator=(const Db& rhs);
    bool create();
    void update_info();
    void print_info(std::ostream& out);
    template<typename T>
    T get_info_item(const std::string& item);
    bool delete_();
    std::string url() const;
    void print();
    void run_view(View& v, int limit_rows, int start_offset, int& total_rows,
                  int& offset, std::vector<couch::Document>& docs);
  private:
    class impl;
    std::auto_ptr<impl> impl_;
  };

  class Document
  {
  public:
    Document(){}
    explicit Document(const Db& db, const std::string& id = "");
    Document(const Document& rhs);
    Document& operator=(const Document& rhs);
    void create();

    void whoami();
    std::string toString();

    void print_result(std::ostream& out);

    void update();

    void commit();

    void clear_all();

    void clear_value(const std::string& key);

    template<typename T>
    T get_value(const std::string& key);

    template<typename T>
    void set_value(const std::string& key, const T& val);

    void set_value(const std::string& key, const char* val);

    template<typename T>
    void attach(const std::string& attachment_name, const T& value)
    {
      typedef boost::archive::binary_oarchive OutputArchive;
      std::stringstream ss;
      OutputArchive ar(ss);
      ar & value;
      attach(attachment_name, ss, "application/octet-stream");
    }

    void attach(const std::string& attachment_name, std::istream& stream,
                const std::string& content_type);

    template<typename T>
    void get_attachment(const std::string& attachment_name, T& value)
    {
      typedef boost::archive::binary_iarchive InputArchive;
      std::stringstream stream;
      get_attachment_stream(attachment_name, stream);
      stream.seekg(0);
      InputArchive ar(stream);
      ar & value;
    }
    void get_attachment_stream(const std::string& attachment_name,
                               std::ostream& stream);
  private:
    class impl;
    std::auto_ptr<impl> impl_;
  };

  template<>
  bool Document::get_value<bool>(const std::string& key);
  template<>
  int Document::get_value<int>(const std::string& key);
  template<>
  double Document::get_value<double>(const std::string& key);
  template<>
  std::string Document::get_value<std::string>(const std::string& key);

  template<>
  void Document::set_value<bool>(const std::string& key, const bool& val);
  template<>
  void Document::set_value<int>(const std::string& key, const int& val);
  template<>
  void Document::set_value<double>(const std::string& key, const double& val);

  template<>
  void Document::set_value<std::string>(const std::string& key,
                                        const std::string& val);

  template<>
  bool Db::get_info_item<bool>(const std::string& item);
  template<>
  int Db::get_info_item<int>(const std::string& item);
  template<>
  double Db::get_info_item<double>(const std::string& item);
  template<>
  std::string Db::get_info_item<std::string>(const std::string& item);

}
