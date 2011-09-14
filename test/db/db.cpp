#include <gtest/gtest.h>

#include <object_recognition/db/db.h>

const char* db_url = "http://localhost:5984";
using namespace object_recognition::db_future;
TEST(OR_db, Connect)
{

  ObjectDb db;
  boost::property_tree::ptree db_p;
  db_p.add("type", "CouchDB");
  db_p.add("url", db_url);
  db.set_params(db_p);
  Document doc;
  doc.set_value("x", 10);
  doc.set_value("bar", std::string("uUuUuU"));
  doc.Persist(db, "test_it");
}

