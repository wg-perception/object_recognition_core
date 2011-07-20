#pragma once
#include <opencv2/core/core.hpp>
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
    };
  }
}

