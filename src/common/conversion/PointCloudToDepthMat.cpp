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

#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using ecto::tendrils;

namespace object_recognition
{
  namespace conversion
  {
    /** Ecto implementation of a module that takes a point cloud as an input and stacks it in a matrix of floats:
     * - if the point cloud is organized, the return a matrix is width by height with 3 channels (for x, y and z)
     * - if the point cloud is unorganized, the return a matrix is n_point by 1 with 3 channels (for x, y and z)
     */
    struct PointCloudToDepthMat
    {
      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> const> >("point_cloud", "The point cloud");
        inputs.declare<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const> >("point_cloud_rgb",
                                                                                    "The RGB point cloud");
        outputs.declare<cv::Mat>("depth", "The depth image");
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        // In DepthTo3 we get the 3d point of a (u,v) point this way:
        // float fx = K.at<float>(0, 0);
        // float fy = K.at<float>(1, 1);
        // float cx = K.at<float>(0, 2);
        // float cy = K.at<float>(1, 2);
        // *(sp_begin++) = (u - cx) * z / fx;
        // *(sp_begin++) = (v - cy) * z / fy;
        // *(sp_begin++) = z;
        // So we just need to recover the z

        // Convert the point cloud to a width by height 3 channel matrix
        cv::Mat depth;
        {
          boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const> point_cloud_rgb = inputs.get<
              boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const> >("point_cloud_rgb");
          if (!point_cloud_rgb->points.empty())
          {
            depth = cv::Mat_<float>(point_cloud_rgb->height, point_cloud_rgb->width);
            float * data_points = reinterpret_cast<float*>(depth.data);
            const float * data_pcd = reinterpret_cast<const float*>(point_cloud_rgb->points.data()) + 2;
            for (unsigned int i = 0; i < point_cloud_rgb->size(); ++i, data_pcd += 3, ++data_points)
              *data_points = *data_pcd;
          }
        }
        {
          boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>const> point_cloud = inputs.get<
              boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> const> >("point_cloud");
          if (!point_cloud->points.empty())
          {
            depth = cv::Mat_<float>(point_cloud->height, point_cloud->width);
            float * data_points = reinterpret_cast<float*>(depth.data);
            const float * data_pcd = reinterpret_cast<const float*>(point_cloud->points.data()) + 2;
            for (unsigned int i = 0; i < point_cloud->size(); ++i, data_pcd += 3, ++data_points)
              *data_points = *data_pcd;
          }
        }

        outputs.get<cv::Mat>("depth") = depth;

        return ecto::OK;
      }
    }
    ;
  }
}

ECTO_CELL(conversion, object_recognition::conversion::PointCloudToDepthMat, "PointCloudToDepthMat",
          "Given a point cloud returns a depth cv::Mat (just the z of the points)");
