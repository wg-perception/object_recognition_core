#pragma once
#include <object_recognition/db/utils.h>
#include <boost/format.hpp>
namespace object_recognition
{
  namespace db_future
  {
    namespace parameters
    {
      inline std::string
      CouchDB(const std::string& url = "http://localhost:5984")
      {
        return boost::str(boost::format("{\"type\": \"CouchDB\",\"url\": \"%s\"}")%url);
      }
    }

  }
}
