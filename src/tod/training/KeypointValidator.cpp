
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
#include <opencv2/highgui/highgui.hpp>
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
    declare_params(ecto::tendrils& params)
    {
      params.declare<double>("baseline", "the baseline in meters", 0.075).required();
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The keypoints").required(true);
      inputs.declare<cv::Mat>("descriptors", "The descriptors").required(true);
      inputs.declare<cv::Mat>("mask", "The mask keypoint have to belong to").required(true);
      inputs.declare<cv::Mat>("K", "The calibration matrix").required(true);
      inputs.declare<cv::Mat>("depth", "The depth image (with a size similar to the mask one).").required(true);

      outputs.declare<cv::Mat>(
          "points", "The valid keypoints: 1 x n_points x 3 channels (x in pixels, y in pixels, disparity in pixels)");
      outputs.declare<cv::Mat>("descriptors", "The matching descriptors, n_points x feature_length");
    }

    void
    configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      baseline_ = params["baseline"];
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      const std::vector<cv::KeyPoint> & in_keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
      cv::Mat in_mask, depth, descriptors, in_K, K;
      inputs["mask"] >> in_mask;
      inputs["depth"] >> depth;
      inputs["K"] >> in_K;
      in_K.convertTo(K, CV_32FC1);
      inputs["descriptors"] >> descriptors;
      size_t n_points = descriptors.rows;
      cv::Mat clean_descriptors = cv::Mat(descriptors.size(), descriptors.type());
      cv::Mat clean_points = cv::Mat(1, n_points, CV_32FC3);

      cv::Mat_<uchar> mask;
      in_mask.convertTo(mask, CV_8U);
      // Erode just because of the possible rescaling
      cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 4);

      int width = mask.cols, height = mask.rows;
      size_t clean_row_index = 0;
      for (size_t keypoint_index = 0; keypoint_index < n_points; ++keypoint_index)
      {
        // First, make sure that the keypoint belongs to the mask
        const cv::KeyPoint & in_keypoint = in_keypoints[keypoint_index];
        unsigned int x = roundWithinBounds(in_keypoint.pt.x, 0, width), y = roundWithinBounds(in_keypoint.pt.y, 0,
                                                                                              height);
        float z;
        bool is_good = false;
        if (mask(y, x))
          is_good = true;
        else
        {
          // If the keypoint does not belong to the mask, look in a slightly bigger neighborhood
          int window_size = 2;
          float min_dist_sq = std::numeric_limits<float>::max();
          // Look into neighborhoods of different sizes to see if we have a point in the mask
          for (unsigned int i = roundWithinBounds(x - window_size, 0, width);
              i <= roundWithinBounds(x + window_size, 0, width); ++i)
            for (unsigned int j = roundWithinBounds(y - window_size, 0, height);
                j <= roundWithinBounds(y + window_size, 0, height); ++j)
              if (mask(j, i))
              {
                float dist_sq = (i - in_keypoint.pt.x) * (i - in_keypoint.pt.x)
                                + (j - in_keypoint.pt.y) * (j - in_keypoint.pt.y);
                if (dist_sq < min_dist_sq)
                {
                  // If the point is in the mask and the closest from the keypoint
                  x = i;
                  y = j;
                  min_dist_sq = dist_sq;
                  is_good = true;
                }
              }
        }
        if (!is_good)
          continue;

        // Now, check that the depth of the keypoint is valid
        if (depth.depth() == CV_16U)
        {
          z = depth.at<uint16_t>(y, x);
          if ((z == std::numeric_limits<uint16_t>::min()) || (z == std::numeric_limits<uint16_t>::max()))
            is_good = false;
          z /= 1000;
        }
        else
        {
          z = depth.at<float>(y, x);
          if ((z != z) || (z == std::numeric_limits<float>::max()))
            is_good = false;
        }

        if (!is_good)
          continue;

        // Store the keypoint and descriptor
        clean_points.at<cv::Vec3f>(0, clean_row_index) = cv::Vec3f(x, y, (*baseline_) * K.at<float>(0, 0) / z);
        cv::Mat clean_descriptor_row = clean_descriptors.row(clean_row_index++);
        descriptors.row(keypoint_index).copyTo(clean_descriptor_row);
      }

      cv::Mat final_points, final_descriptors;
      if (clean_row_index > 0)
      {
        clean_points.colRange(0, clean_row_index).copyTo(final_points);
        clean_descriptors.rowRange(0, clean_row_index).copyTo(final_descriptors);
      }
      outputs["points"] << final_points;
      outputs.get<cv::Mat>("descriptors") = final_descriptors;

      return ecto::OK;
    }
  private:
    ecto::spore<double> baseline_;
  }
  ;
}

ECTO_CELL(tod_training, KeypointsValidator, "KeypointsValidator",
          "Given keypoints and a mask, make sure they belong to the mask by rounding their coordinates.");
