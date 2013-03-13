#include <string>
#include <vector>

#include <boost/foreach.hpp>

#include <gtest/gtest.h>

#include <object_recognition_core/common/json_spirit/json_spirit.h>
#include <object_recognition_core/db/db.h>

const char* db_url = "http://localhost:5984";

using namespace object_recognition_core::db;
using object_recognition_core::db::ObjectDbParameters;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions related to parameters

std::vector<std::string>
db_types()
{
  const char* types[] =
  { "CouchDB", "filesystem" };
  return std::vector<std::string>(types, types + 1);
}

ObjectDbParameters
params_bogus(const std::string& url = db_url)
{
  or_json::mObject params;
  params["urld"] = url;
  return ObjectDbParameters(params);
}

std::string
params_garbage(const std::string& url = db_url)
{
  return "{\ndfkja:dkfj, dfkjak, dfkjalksf.dfj ---\ndfjkasdf";
}

ObjectDbParameters
params_test(const std::string &db_type)
{
  ObjectDbParameters params;
  if (db_type == "CouchDB")
  {
    params = ObjectDbParameters("CouchDB");
    params.set_parameter("root", "http://foo:12323");
    params.set_parameter("collection", "test_it");
  }
  else if (db_type == "filesystem")
  {
    params = ObjectDbParameters("filesystem");
    params.set_parameter("path", "/bogus/path/for/testing");
    params.set_parameter("collection", "test_it");
  }
  return params;
}

ObjectDbParameters
params_valid(const std::string &db_type)
{
  ObjectDbParameters params;
  if (db_type == "CouchDB")
  {
    params = ObjectDbParameters("CouchDB");
    params.set_parameter("root", "http://localhost:5984");
    params.set_parameter("collection", "test_it");
  }
  else if (db_type == "filesystem")
  {
    params = ObjectDbParameters("filesystem");
    params.set_parameter("path", "/tmp");
    params.set_parameter("collection", "test_it");
  }
  return params;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utils
or_json::mObject
parse_status(const std::string& status)
{
  or_json::mValue value;
  std::stringstream ss(status);
  or_json::read(ss, value);
  return value.get_obj();
}

void
expect_not_found(ObjectDbPtr& db, const std::string& collection)
{
  std::string status = db->Status(collection);
  or_json::mObject ps = parse_status(status);
  EXPECT_EQ(ps["error"].get_str(), std::string("not_found"));
}

void
delete_c(ObjectDbPtr& db, const std::string& collection)
{
  db->DeleteCollection(collection);
  expect_not_found(db, collection);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Error handlers

void
handling_invalid_server(const std::string & db_type, const std::runtime_error& e, const std::string &collection,
                        const std::string &id)
{
  switch (ObjectDbParameters::StringToType(db_type))
  {
    case ObjectDbParameters::COUCHDB:
    {
      std::string url = "http://foo:12323";
      if (!collection.empty())
      {
        url.append("/" + collection);
        if (!id.empty())
          url.append("/" + id);
      };EXPECT_EQ(std::string(e.what()), std::string("No response from server. : ") + url);
      break;
    }
    case ObjectDbParameters::FILESYSTEM:
      EXPECT_EQ(std::string(e.what()), std::string("Path /bogus/path/for/testing does not exist. Please create."));
      break;
    default:
      throw std::runtime_error("Status test not implemented for " + db_type);
  }
}

void
handling_valid_server(const std::string & db_type, const std::string & status)
{
  or_json::mObject ps = parse_status(status);
  switch (ObjectDbParameters::StringToType(db_type))
  {
    case ObjectDbParameters::COUCHDB:
      EXPECT_EQ(ps.count("couchdb"), 1);
      break;
    case ObjectDbParameters::FILESYSTEM:
      EXPECT_EQ(ps.count("filesystem"), 1);
      break;
    default:
      throw std::runtime_error("Status test not implemented for " + db_type);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(OR_db, Status) {
  BOOST_FOREACH(const std::string &db_type, db_types()) {
    ObjectDbParameters db_params(db_type);
    ObjectDbPtr db = db_params.generateDb();
    std::string status = db->Status();
    handling_valid_server(db_type, status);
  }
}

TEST(OR_db, CreateDelete)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params(db_type);
        {
          ObjectDbPtr db = db_params.generateDb();
          db->CreateCollection("test_it");
          std::string status = db->Status("test_it");
          or_json::mObject ps = parse_status(status);
          EXPECT_EQ(ps["db_name"].get_str(), std::string("test_it"));
        }
        {
          ObjectDbPtr db = db_params.generateDb();
          delete_c(db, "test_it");
        }
      }
}
TEST(OR_db, DeleteNonexistant)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params(db_type);
        ObjectDbPtr db = db_params.generateDb();
        db->DeleteCollection("dgadf");
      }
}

TEST(OR_db, DocumentPersistLoad)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbPtr db = params_valid(db_type).generateDb();
        delete_c(db, "test_it");
        std::string id;
        std::string original_attachment = "Ecto is awesome";
        {
          Document doc;
          doc.set_db(db);
          doc.set_field("x", 1.0);
          doc.set_field("foo", "UuU");
          doc.Persist();
          id = doc.id();
        }
        {
          Document doc;
          doc.set_db(db);
          doc.set_document_id(id);
          EXPECT_EQ(doc.get_field<double>("x"), 1.0);
          EXPECT_EQ(doc.get_field<std::string>("foo"), std::string("UuU"));
        }
        delete_c(db, "test_it");
      }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(OR_db, NonExistantCouch)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params = params_test(db_type);
        ObjectDbPtr db = db_params.generateDb();
        try
        {
          std::string status = db->Status();
          ASSERT_FALSE(true);
        } catch (std::runtime_error& e)
        {
          handling_invalid_server(db_type, e, "", "");
        }
      }
}

TEST(OR_db, StatusCollectionNonExistantDb)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params = params_test(db_type);
        ObjectDbPtr db = db_params.generateDb();
        std::string collection = "test_it";
        try
        {
          std::string status = db->Status(collection);
          ASSERT_FALSE(true);
        } catch (std::runtime_error& e)
        {
          handling_invalid_server(db_type, e, collection, "");
        }
      }
}

TEST(OR_db, DeleteBogus)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params = params_test(db_type);
        ObjectDbPtr db = db_params.generateDb();
        std::string collection = "test_it";
        try
        {
          db->DeleteCollection(collection);
          ASSERT_FALSE(true);
        } catch (std::runtime_error& e)
        {
          handling_invalid_server(db_type, e, collection, "");
        }
      }
}

TEST(OR_db, StatusCollectionNonExistant)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params(db_type);
        ObjectDbPtr db = db_params.generateDb();
        db->DeleteCollection("test_it");

        std::string status = db->Status("test_it");
        or_json::mObject ps = parse_status(status);
        EXPECT_EQ(ps["error"].get_str(), std::string("not_found"));
        EXPECT_EQ(ps["reason"].get_str(), std::string("no_db_file"));
      }
}

TEST(OR_db, StatusCollectionExistant)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params(db_type);
        ObjectDbPtr db = db_params.generateDb();
        db->DeleteCollection("test_it");
        db->CreateCollection("test_it");
        std::string status = db->Status("test_it");
        or_json::mObject ps = parse_status(status);
        EXPECT_EQ(ps["db_name"].get_str(), std::string("test_it"));
        db->DeleteCollection("test_it");
      }
}

TEST(OR_db, DocumentBadId)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbPtr db = params_valid(db_type).generateDb();
        try
        {
          Document doc;
          doc.set_db(db);
          doc.set_document_id("bogus_id");
          ASSERT_FALSE(true);
        } catch (std::runtime_error& e)
        {
          switch (ObjectDbParameters::StringToType(db_type))
          {
            case ObjectDbParameters::COUCHDB:
            {
              EXPECT_EQ(std::string(e.what()), std::string("Object Not Found : http://localhost:5984/test_it/bogus_id"));
              break;
            }
            case ObjectDbParameters::FILESYSTEM:
              EXPECT_EQ(std::string(e.what()), std::string("Object Not Found : /tmp/test_it/all_docs/bogus_id/value"));
              break;
            default:
              throw std::runtime_error("Status test not implemented for " + db_type);
          }
        }
      }
}

TEST(OR_db, DocumentUrl)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params = params_test(db_type);
        db_params.set_parameter("collection", "test_it");
        ObjectDbPtr db = db_params.generateDb();
        std::string bogus_id = "bogus_id";
        try
        {
          Document doc;
          doc.set_db(db);
          doc.set_document_id(bogus_id);
          ASSERT_FALSE(true);
        } catch (std::runtime_error& e)
        {
          handling_invalid_server(db_type, e, db_params.at("collection").get_str(), bogus_id);
        }
      }
}

TEST(OR_db, DoubleCreate)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params(db_type);
        ObjectDbPtr db = db_params.generateDb();
        db->CreateCollection("aa");
        db->CreateCollection("aa");
        db->DeleteCollection("aa");
      }
}

TEST(OR_db, DoubleDelete)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params(db_type);
        ObjectDbPtr db = db_params.generateDb();
        db->CreateCollection("aa");
        db->DeleteCollection("aa");
        db->DeleteCollection("aa");
      }
}

TEST(OR_db, ParamsBogus)
{
  try
  {
    ObjectDbPtr db = params_bogus().generateDb();
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    EXPECT_EQ(std::string("You must supply a database type. e.g. CouchDB"), std::string(e.what()));
  }
}

TEST(OR_db, ParamsGarbage)
{
  try
  {
    ObjectDbParameters db_params(params_garbage());
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    std::string error(e.what());
    EXPECT_TRUE(error.find(std::string("No implementation for that db type.")) != std::string::npos);
  }
}

TEST(OR_db, NonArgsDbCreate)
{
  ObjectDbPtr db;
  try
  {
    db->CreateCollection("aa");
    ASSERT_FALSE(true);
  } catch (std::runtime_error& e)
  {
    std::string expect = "This ObjectDb instance is uninitialized.";
    EXPECT_EQ(std::string(e.what()), expect);
  }
}
TEST(OR_db, NonArgsDbInsert)
{
  ObjectDbPtr db;

  std::string id;
  {
    Document doc;
    doc.set_db(db);
    doc.set_field("x", 1.0);
    doc.set_field("foo", "UuU");
    try
    {
      doc.Persist();
      ASSERT_FALSE(true);
    } catch (std::runtime_error& e)
    {
      std::string expect = "This ObjectDb instance is uninitialized.";
      EXPECT_EQ(std::string(e.what()), expect);
    }
  }
}

TEST(OR_db, InitSeperatelyChangeURL)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params(db_type);
        ObjectDbPtr db = db_params.generateDb();
        std::string status = db->Status();
        handling_valid_server(db_type, status);
        ObjectDbParameters params = params_test(db_type);
        db = params.generateDb();
        try
        {
          db->Status(status);
          ASSERT_FALSE(true);
        } catch (std::runtime_error& e)
        {
//error messages seem to be bit platform dependent.
//    std::string expect = "No response from server. : http://abc";
//    EXPECT_EQ(e.what(), expect);
        }
      }
}

TEST(OR_db, ObjectDbCopy)
{
  BOOST_FOREACH(const std::string &db_type, db_types())
      {
        ObjectDbParameters db_params(db_type);
        ObjectDbPtr db = db_params.generateDb(), db2;
        db2 = db;
        std::string s1, s2;
        db->Status(s1);
        db2->Status(s2);
        EXPECT_EQ(s1, s2);
      }
}

TEST(OR_db, JSONReadWrite)
{
  or_json::mObject params1, params2;
  std::stringstream ssparams1, ssparams2, ssparams3;
  ssparams1 << "{\"num1\":2, \"num2\":3.5, \"str\":\"foo\"}";
  or_json::mValue value;
  or_json::read(ssparams1, value);
  params1 = value.get_obj();

  // Write it to a JSON string and make sure numbers are persisted as numbers
  value = or_json::mValue(params1);
  or_json::write(value, ssparams2);
  std::string new_json = ssparams2.str();
  EXPECT_GE(new_json.find("\"2\""), new_json.size());
  EXPECT_GE(new_json.find("\"3.5\""), new_json.size());
  EXPECT_LT(new_json.find("\"foo\""), new_json.size());

  // Make sure we can read it back
  ssparams3 << ssparams2.str();
  or_json::read(ssparams3, value);
  params2 = value.get_obj();
  EXPECT_EQ(params1["num1"].get_uint64(), params2["num1"].get_uint64());
  EXPECT_EQ(params1["num2"].get_real(), params2["num2"].get_real());
  EXPECT_EQ(params1["str"].get_str(), params2["str"].get_str());
}

TEST(OR_db, JSONReadBigInteger)
{
  or_json::mObject params1, params2;
  std::stringstream ssparams1, ssparams2, ssparams3;
  ssparams1 << "{\"num\":3372036854775808  }";
  or_json::mValue value;
  or_json::read(ssparams1, value);
  params1 = value.get_obj();
}
