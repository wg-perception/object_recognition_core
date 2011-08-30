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

#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using ecto::tendrils;

/** Ecto module that transforms 3d points from camera coordinates to world coordinates
 */
struct CameraToWorld
{
  static void
  declare_params(tendrils& p)
  {
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("points", "The points (n by 3 matrix)");
    inputs.declare<cv::Mat>("R", "The rotation matrix");
    inputs.declare<cv::Mat>("T", "The translation vector");
    outputs.declare<cv::Mat>("points", "The points in the world frame");
  }

  void
  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
  }

  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat_<float> R, T, in_points;
    inputs.get<cv::Mat>("R").convertTo(R,CV_32F);
    inputs.get<cv::Mat>("T").convertTo(T,CV_32F);
    inputs.get<cv::Mat>("points").convertTo(in_points,CV_32F);

    cv::Mat points;
    if (T.rows == 3)
      cv::repeat(T.t(), in_points.rows, 1, points);
    else
      cv::repeat(T, in_points.rows, 1, points);

    // Apply the inverse translation
    points = (in_points - points) * R;

    outputs.get<cv::Mat>("points") = points;

    return 0;
  }
};

ECTO_CELL(tod, CameraToWorld, "CameraToWorld",
          "Given 3d points in the camera frame and the (R,T) of the camera in world frame, give world points.");
