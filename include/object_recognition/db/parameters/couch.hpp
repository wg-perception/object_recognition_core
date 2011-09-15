#pragma once
#include <object_recognition/db/utils.h>

namespace object_recognition
{
  namespace db_future
  {
    namespace parameters
    {
      static std::string
      CouchDB(const std::string& url = db_url)
      {
        ptree db_p;
        db_p.add("type", "CouchDB");
        db_p.add("url", url);
        std::stringstream ss;
        write_json(ss, db_p);
        return ss.str();
      }
    }

  }
}
