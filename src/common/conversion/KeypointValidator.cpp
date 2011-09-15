/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <limits.h>
#include <vector>

#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using ecto::tendrils;

namespace
{
  inline unsigned int
  roundWithinBounds(float xy, int xy_min, int xy_max)
  {
    return std::min(std::max(cvRound(xy), xy_min), xy_max);
  }
}
namespace
{
  /** Ecto module that makes sure keypoints are part of a mask
   */
  struct KeypointsValidator
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The keypoints").required(true);
      inputs.declare<cv::Mat>("image", "The mask keypoint have to belong to").required(true);
      outputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The adjusted keypoints");
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      std::vector<cv::KeyPoint> keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
      const cv::Mat & in_mask = inputs.get<cv::Mat>("image");
      cv::Mat_<uchar> mask(in_mask.size());
      if (in_mask.depth() == (CV_8U))
        mask = in_mask;
      else
        in_mask.convertTo(mask, CV_8U);

      int width = mask.cols, height = mask.rows;
      BOOST_FOREACH(cv::KeyPoint & keypoint, keypoints)
          {
            unsigned int x = roundWithinBounds(keypoint.pt.x, 0, width), y = roundWithinBounds(keypoint.pt.y, 0,
                                                                                               height);
            if (mask(y, x))
            {
              keypoint.pt.x = x;
              keypoint.pt.y = y;
              continue;
            }
            bool is_good = false;
            int window_size = 1;
            float min_dist_sq = std::numeric_limits<float>::max();
            // Look into neighborhoods of different sizes to see if we have a point in the mask
            while (!is_good)
            {
              for (unsigned int i = roundWithinBounds(x - window_size, 0, width);
                  i <= roundWithinBounds(x + window_size, 0, width); ++i)
                for (unsigned int j = roundWithinBounds(y - window_size, 0, height);
                    j <= roundWithinBounds(y + window_size, 0, height); ++j)
                  if (mask(j, i))
                  {
                    float dist_sq = (i - keypoint.pt.x) * (i - keypoint.pt.x)
                                    + (j - keypoint.pt.y) * (j - keypoint.pt.y);
                    if (dist_sq < min_dist_sq)
                    {
                      // If the point is in the mask and the closest from the keypoint
                      keypoint.pt.x = i;
                      keypoint.pt.y = j;
                      min_dist_sq = dist_sq;
                      is_good = true;
                    }
                  }
              ++window_size;
            }
          }
      outputs.get<std::vector<cv::KeyPoint> >("keypoints") = keypoints;

      return ecto::OK;
    }
  };
}

ECTO_CELL(conversion, KeypointsValidator, "KeypointsValidator",
          "Given keypoints and a mask, make sure they belong to the mask by rounding their coordinates.");
