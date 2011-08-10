#pragma once
#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <string>

#include "object_recognition/db/couch.hpp"

namespace object_recognition
{
  namespace capture
  {
    struct Observation
    {
      cv::Mat image, depth, mask;
      cv::Mat R, T, K;
      std::string object_id, session_id;
      int frame_number;
      void
      operator>>(couch::Document& doc);
      void
      operator<<(couch::Document& doc);
      void
      operator>>(const ecto::tendrils& o);
      void
      operator<<(const ecto::tendrils& i);
    };
  }
}

