#pragma once
#include <ecto/ecto.hpp>
#include <string>

#include <opencv2/core/core.hpp>
#include <object_recognition/db/db.h>

namespace object_recognition
{
  namespace capture
  {
    struct Observation
    {
      //fields
      std::string object_id, session_id;
      int frame_number;

      //attachments
      cv::Mat K, R, T; //yaml files
      cv::Mat image, depth, mask; //png images

      static void
      declare(ecto::tendrils& t, bool required);
    };

    void
    operator>>(Observation& o, db_future::Document& doc);
    void
    operator<<(Observation& o, db_future::Document& doc);
    void
    operator>>(Observation& obs, const ecto::tendrils& o);
    void
    operator<<(Observation& obs, const ecto::tendrils& i);
  }
}
