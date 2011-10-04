#include <string>

#include <gtest/gtest.h>

#include <object_recognition/db/db.h>
#include <object_recognition/db/parameters/couch.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

const char* db_url = "http://localhost:5984";
using namespace object_recognition::db_future;

using boost::property_tree::ptree;
using boost::property_tree::json_parser::write_json;
using boost::property_tree::json_parser::read_json;
using parameters::CouchDB;

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
  ObjectDb db(CouchDB());
  std::string status;
  db.Status(status);
  ptree ps = parse_status(status);
  EXPECT_EQ(ps.count("couchdb"), 1);
}

TEST(OR_db, CreateDelete)
{
  {
    ObjectDb db(CouchDB());
    db.CreateCollection("test_it");
    std::string status;
    db.Status("test_it", status);
    ptree ps = parse_status(status);
    EXPECT_EQ(ps.get<std::string>("db_name"), "test_it");
  }
  {
    ObjectDb db(CouchDB());
    delete_c(db, "test_it");
  }
}
TEST(OR_db, DeleteNonexistant)
{
  ObjectDb db(CouchDB());
  db.DeleteCollection("dgadf");
}

TEST(OR_db, DocumentPesistLoad)
{
  ObjectDb db(CouchDB());
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
  ObjectDb db(CouchDB("http://foo:12323"));
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
  ObjectDb db(CouchDB("http://foo:12323"));
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
  ObjectDb db(CouchDB("http://foo:12323"));
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
  ObjectDb db(CouchDB());
  db.DeleteCollection("test_it");

  std::string status;
  db.Status("test_it", status);
  ptree ps = parse_status(status);
  EXPECT_EQ(ps.get<std::string>("error"), "not_found");
  EXPECT_EQ(ps.get<std::string>("reason"), "no_db_file");
}

TEST(OR_db, StatusCollectionExistant)
{
  ObjectDb db(CouchDB());
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
  ObjectDb db(CouchDB());
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
  ObjectDb db(CouchDB("http://foo:12323"));
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
  ObjectDb db(CouchDB());
  db.CreateCollection("aa");
  db.CreateCollection("aa");
  db.DeleteCollection("aa");
}

TEST(OR_db, DoubleDelete)
{
  ObjectDb db(CouchDB());
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
  db.set_params(CouchDB());
  std::string status;
  db.Status(status);
  ptree ps = parse_status(status);
  EXPECT_EQ(ps.count("couchdb"), 1);
  db.set_params(CouchDB("http://abc"));
  try
  {
    db.Status(status);
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
//error messages seem to be bit platform dependent.
//    std::string expect = "No response from server. : http://abc";
//    EXPECT_EQ(e.what(), expect);
  }
}

TEST(OR_db, ObjectDbCopy)
{
  ObjectDb db(CouchDB()), db2;
  db2 = db;
  std::string s1, s2;
  db.Status(s1);
  db2.Status(s2);
  EXPECT_EQ(s1, s2);
}

TEST(OR_db, JSONReadWrite)
{
  boost::property_tree::ptree params1, params2;
  std::stringstream ssparams1, ssparams2, ssparams3;
  ssparams1 << "{\"num1\":2, \"num2\":3.5, \"str\":\"foo\"}";
  boost::property_tree::read_json(ssparams1, params1);

  // Write it to a JSON string and make sure numbers are persisted as numbers
  boost::property_tree::write_json(ssparams2, params1);
  std::string new_json = ssparams2.str();
  EXPECT_GE(new_json.find("\"2\""), new_json.size());
  EXPECT_GE(new_json.find("\"3.5\""), new_json.size());
  EXPECT_LT(new_json.find("\"foo\""), new_json.size());

  // Make sure we can read it back
  ssparams3 << ssparams2.str();
  boost::property_tree::read_json(ssparams3, params2);
  EXPECT_EQ(params1.get<std::string>("num1"), params2.get<std::string>("num1"));
  EXPECT_EQ(params1.get<std::string>("num2"), params2.get<std::string>("num2"));
  EXPECT_EQ(params1.get<std::string>("str"), params2.get<std::string>("str"));
}
