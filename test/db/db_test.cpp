#include <gtest/gtest.h>

#include <object_recognition/db/db.h>
#include <boost/property_tree/json_parser.hpp>

const char* db_url = "http://localhost:5984";
using namespace object_recognition::db_future;

using boost::property_tree::ptree;
using boost::property_tree::json_parser::write_json;
using boost::property_tree::json_parser::read_json;

std::string
couchdb_params(const std::string& url = db_url)
{
  ptree db_p;
  db_p.add("type", "CouchDB");
  db_p.add("url", url);
  std::stringstream ss;
  write_json(ss, db_p);
  return ss.str();
}

ptree
parse_status(const std::string& status)
{

  ptree p;
  std::stringstream ss(status);
  boost::property_tree::json_parser::read_json(ss, p);
  return p;
}

void
expect_not_found(ObjectDb& db, const std::string& collection)
{
  std::string status;
  db.Status(collection, status);
  ptree ps = parse_status(status);
  EXPECT_EQ(ps.get<std::string>("error"), "not_found");
}

void
delete_c(ObjectDb& db, const std::string& collection)
{
  db.DeleteCollection(collection);
  expect_not_found(db, collection);
}

TEST(OR_db, Status)
{
  ObjectDb db(couchdb_params());
  std::string status;
  db.Status(status);
  ptree ps = parse_status(status);
  EXPECT_EQ(ps.count("couchdb"), 1);
}

TEST(OR_db, CreateDelete)
{
  {
    ObjectDb db(couchdb_params());
    db.CreateCollection("test_it");
    std::string status;
    db.Status("test_it", status);
    ptree ps = parse_status(status);
    EXPECT_EQ(ps.get<std::string>("db_name"), "test_it");
  }
  {
    ObjectDb db(couchdb_params());
    delete_c(db, "test_it");
  }
}

TEST(OR_db, DocumentPesistLoad)
{
  ObjectDb db(couchdb_params());
  delete_c(db, "test_it");
  std::string id;
  {
    Document doc(db, "test_it");
    doc.set_value("x", 1.0);
    doc.set_value("foo", "UuU");
    doc.Persist();
    id = doc.id();
  }
  {
    Document doc(db, "test_it", id);
    EXPECT_EQ(doc.get_value<double>("x"), 1.0);
    EXPECT_EQ(doc.get_value<std::string>("foo"), "UuU");
  }
  delete_c(db, "test_it");
}

TEST(OR_db, NonExistantCouch)
{
  ObjectDb db(couchdb_params("http://foo:12323"));
  try
  {
    std::string status;
    db.Status(status);
  } catch (std::runtime_error& e)
  {
    EXPECT_EQ(std::string(e.what()), "No response from server. : http://foo:12323");
  }
}
