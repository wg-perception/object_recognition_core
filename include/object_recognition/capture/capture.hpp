#pragma once
#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <objcog/db/couch.hpp>
#include <string>
namespace objcog
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
      operator>>(ecto::tendrils& o);
      void
      operator<<(ecto::tendrils& i);
    };
  }
}

