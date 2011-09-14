#include <gtest/gtest.h>

#include <object_recognition/db/db.h>
#include <boost/property_tree/json_parser.hpp>

const char* db_url = "http://localhost:5984";
using namespace object_recognition::db_future;

using boost::property_tree::ptree;
using boost::property_tree::json_parser::write_json;
using boost::property_tree::json_parser::read_json;

std::string
params_couchdb(const std::string& url = db_url)
{
  ptree db_p;
  db_p.add("type", "CouchDB");
  db_p.add("url", url);
  std::stringstream ss;
  write_json(ss, db_p);
  return ss.str();
}
std::string
params_bogus(const std::string& url = db_url)
{
  ptree db_p;
  db_p.add("urld", url);
  std::stringstream ss;
  write_json(ss, db_p);
  return ss.str();
}

std::string
params_garbage(const std::string& url = db_url)
{
  return "{\ndfkja:dkfj, dfkjak, dfkjalksf.dfj ---\ndfjkasdf";
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
  ObjectDb db(params_couchdb());
  std::string status;
  db.Status(status);
  ptree ps = parse_status(status);
  EXPECT_EQ(ps.count("couchdb"), 1);
}

TEST(OR_db, CreateDelete)
{
  {
    ObjectDb db(params_couchdb());
    db.CreateCollection("test_it");
    std::string status;
    db.Status("test_it", status);
    ptree ps = parse_status(status);
    EXPECT_EQ(ps.get<std::string>("db_name"), "test_it");
  }
  {
    ObjectDb db(params_couchdb());
    delete_c(db, "test_it");
  }
}
TEST(OR_db, DeleteNonexistant)
{
  ObjectDb db(params_couchdb());
  db.DeleteCollection("dgadf");
}

TEST(OR_db, DocumentPesistLoad)
{
  ObjectDb db(params_couchdb());
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
  ObjectDb db(params_couchdb("http://foo:12323"));
  try
  {
    std::string status;
    db.Status(status);
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    EXPECT_EQ(std::string(e.what()), "No response from server. : http://foo:12323");
  }

}

TEST(OR_db, StatusCollectionNonExistantDb)
{
  ObjectDb db(params_couchdb("http://foo:12323"));
  try
  {
    std::string status;
    db.Status("test_it", status);
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    EXPECT_EQ(std::string(e.what()), "No response from server. : http://foo:12323/test_it");
  }
}

TEST(OR_db, DeleteBogus)
{
  ObjectDb db(params_couchdb("http://foo:12323"));
  try
  {
    db.DeleteCollection("test_it");
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    EXPECT_EQ(std::string(e.what()), "No response from server. : http://foo:12323/test_it");
  }
}
TEST(OR_db, StatusCollectionNonExistant)
{
  ObjectDb db(params_couchdb());
  db.DeleteCollection("test_it");

  std::string status;
  db.Status("test_it", status);
  ptree ps = parse_status(status);
  EXPECT_EQ(ps.get<std::string>("error"), "not_found");
  EXPECT_EQ(ps.get<std::string>("reason"), "no_db_file");
}

TEST(OR_db, StatusCollectionExistant)
{
  ObjectDb db(params_couchdb());
  db.DeleteCollection("test_it");
  std::string status;
  db.CreateCollection("test_it");
  db.Status("test_it", status);
  ptree ps = parse_status(status);
  EXPECT_EQ(ps.get<std::string>("db_name"), "test_it");
  db.DeleteCollection("test_it");
}

TEST(OR_db, DocumentBadId)
{
  ObjectDb db(params_couchdb());
  try
  {
    Document doc(db, "test_it", "bogus_id");
    ASSERT_FALSE(true);

  } catch (std::runtime_error& e)
  {
    EXPECT_EQ(std::string(e.what()), "Object Not Found : http://localhost:5984/test_it/bogus_id");
  }
}

TEST(OR_db, DocumentUrl)
{
  ObjectDb db(params_couchdb("http://foo:12323"));
  try
  {
    Document doc(db, "test_it", "bogus_id");
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    EXPECT_EQ(std::string(e.what()), "No response from server. : http://foo:12323/test_it/bogus_id");
  }
}

TEST(OR_db, DoubleCreate)
{
  ObjectDb db(params_couchdb());
  db.CreateCollection("aa");
  db.CreateCollection("aa");
  db.DeleteCollection("aa");
}

TEST(OR_db, DoubleDelete)
{
  ObjectDb db(params_couchdb());
  db.CreateCollection("aa");
  db.DeleteCollection("aa");
  db.DeleteCollection("aa");
}

TEST(OR_db, ParamsBogus)
{
  try
  {
    ObjectDb db(params_bogus());
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    EXPECT_EQ(std::string("You must supply a database type. e.g. CouchDB"), e.what());
  }
}

TEST(OR_db, ParamsGarbage)
{
  try
  {
    ObjectDb db(params_garbage());
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    std::string error(e.what());
    EXPECT_EQ(std::string("Failed to parse json --- <unspecified file>(2): expected object name"), error);
  }
}

TEST(OR_db, NonArgsDbCreate)
{
  ObjectDb db;
  try
  {
    db.CreateCollection("aa");
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    std::string expect = "This ObjectDb instance is uninitialized.";
    EXPECT_EQ(e.what(), expect);
  }
}
TEST(OR_db, NonArgsDbInsert)
{
  ObjectDb db;

  std::string id;
  {
    Document doc(db, "test_it");
    doc.set_value("x", 1.0);
    doc.set_value("foo", "UuU");
    try
    {
      doc.Persist();
      ASSERT_FALSE(true);
    } catch (std::runtime_error& e)
    {
      std::string expect = "This ObjectDb instance is uninitialized.";
      EXPECT_EQ(e.what(), expect);
    }
  }
}

TEST(OR_db, InitSeperatelyChangeURL)
{
  ObjectDb db;
  db.set_params(params_couchdb());
  std::string status;
  db.Status(status);
  ptree ps = parse_status(status);
  EXPECT_EQ(ps.count("couchdb"), 1);
  db.set_params(params_couchdb("foo"));
  try
  {
    db.Status(status);
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    std::string expect = "No response from server. : http://foo";
    EXPECT_EQ(e.what(), expect);
  }
}

TEST(OR_db, ObjectDbCopy)
{
  ObjectDb db(params_couchdb()), db2;
  db2 = db;
  std::string s1, s2;
  db.Status(s1);
  db2.Status(s2);
  EXPECT_EQ(s1, s2);
}
