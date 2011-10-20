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

#include <list>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "object_recognition/common/types_eigen.h"

namespace object_recognition
{
  namespace tod
  {
    /** cell storing the 3d points and descriptors while a model is being computed
     */
    struct TodModelStacker
    {
    public:
      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        inputs.declare<cv::Mat>("K", "The camera intrinsics matrix.").required(true);
        inputs.declare<cv::Mat>("R", "The rotation matrix of the camera.").required(true);
        inputs.declare<cv::Mat>("T", "The translation vector of the camera.").required(true);
        inputs.declare<cv::Mat>("points", "The 2d position of the points, with their disparity.").required(true);
        inputs.declare<cv::Mat>("points3d", "The 3d position of the points.").required(true);
        inputs.declare<cv::Mat>("descriptors", "The descriptors.").required(true);

        outputs.declare<Eigen::Matrix3d>("K", "The intrinsic parameter matrix.");
        outputs.declare<VectorQuaterniond>("quaternions", "The initial estimates of the camera rotations.");
        outputs.declare<VectorVector3d>("Ts", "The initial estimates of the camera translations.");
        outputs.declare<std::vector<cv::Mat> >("points3d",
                                               "The measured 2d positions and disparity (3-channel matrices).");
        outputs.declare<std::vector<cv::Mat> >("points",
                                               "The estimated 3d position of the points (3-channel matrices).");
        outputs.declare<std::vector<cv::Mat> >("descriptors", "The stacked descriptors.");
      }

      void
      configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        K_ = outputs["K"];
        quaternions_ = outputs["quaternions"];
        Ts_ = outputs["Ts"];
        points_ = outputs["points"];
        points3d_ = outputs["points3d"];
        descriptors_ = outputs["descriptors"];
      }

      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        // Add the points to the stack of points
        cv::Mat points;
        inputs["points"] >> points;
        if (points.empty())
          return ecto::OK;

        // Clean the input points
        cv::Mat descriptors, points3d;
        inputs["descriptors"] >> descriptors;
        inputs["points3d"] >> points3d;
        descriptors_->push_back(descriptors);
        points_->push_back(points);
        points3d_->push_back(points3d);

        // Add the new camera parameters to the list of camera parameters
        {
          cv::cv2eigen(inputs.get<cv::Mat>("K"), *K_);
          Eigen::Matrix3d R;
          cv::cv2eigen(inputs.get<cv::Mat>("R"), R);
          quaternions_->push_back(Eigen::Quaterniond(R));
          Eigen::Vector3d T;
          cv::cv2eigen(inputs.get<cv::Mat>("T"), T);
          Ts_->push_back(T);
        }

        return ecto::OK;
      }
    private:
      ecto::spore<Eigen::Matrix3d> K_;
      ecto::spore<VectorQuaterniond> quaternions_;
      ecto::spore<VectorVector3d> Ts_;
      ecto::spore<std::vector<cv::Mat> > points3d_;
      ecto::spore<std::vector<cv::Mat> > points_;
      ecto::spore<std::vector<cv::Mat> > descriptors_;
    };
  }
}

ECTO_CELL(tod_training, object_recognition::tod::TodModelStacker, "TodModelStacker", "Stack 3d points and descriptors")
