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

#include <fstream>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <object_recognition/common/conversions.hpp>
using ecto::tendrils;

namespace object_recognition
{
  namespace conversion
  {
    /** Ecto implementation of a module that takes a point cloud as an input and stacks it in a matrix of floats:
     * - if the point cloud is organized, the return a matrix is width by height with 3 channels (for x, y and z)
     * - if the point cloud is unorganized, the return a matrix is n_point by 1 with 3 channels (for x, y and z)
     */
    struct MatToPointCloudXYZ
    {
      // Get the original keypoints and point cloud
      typedef pcl::PointXYZ PointType;
      typedef pcl::PointCloud<PointType> CloudType;

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<cv::Mat>("points", "The width by height by 3 channels (x, y and z)");
        outputs.declare<CloudType::ConstPtr>("point_cloud", "The XYZ point cloud");
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        CloudType::Ptr point_cloud(new CloudType);
        cvToCloud(inputs.get<cv::Mat>("points"), *point_cloud);
        outputs["point_cloud"] << CloudType::ConstPtr(point_cloud);
        return 0;
      }
    };

    struct MatToPointCloudXYZRGB
    {
      typedef pcl::PointXYZRGB PointType;
      // Get the original keypoints and point cloud
      typedef pcl::PointCloud<PointType> CloudType;

      static void
      declare_params(tendrils& p)
      {

      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare(&MatToPointCloudXYZRGB::image,"image", "The rgb image.").required(true);
        inputs.declare(&MatToPointCloudXYZRGB::mask, "mask", "The binary mask for valid points.").required(true);
        inputs.declare(&MatToPointCloudXYZRGB::cloud_out,"points3d", "The 3d points.").required(true);
        outputs.declare("point_cloud", "The XYZRGB point cloud");
      }

      void
      configure(const tendrils&p, const tendrils&i, const tendrils&o)
      {
        R = i["R"];
        T = i["T"];
        image = i["image"];
        mask = i["mask"];
        points3d = i["points3d"];
        view = o["view"];

      }

      int
      process(const tendrils& i, const tendrils& o)
      {

        typedef pcl::PointCloud<Point> CloudT;
        typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudNormalT;

        //extract the cloud
        CloudT::Ptr cloud(new CloudT);
        cvToCloudXYZRGB(*points3d, *cloud, *image, *mask, false);
        pcl::KdTree<Point>::Ptr tree_;
        pcl::NormalEstimation<Point, pcl::Normal> impl;
        pcl::PointCloud<pcl::Normal> normals;
        tree_.reset(new pcl::KdTreeFLANN<Point>);
        impl.setSearchMethod(tree_);

        impl.setInputCloud(cloud);
        impl.setKSearch(50);
        impl.compute(normals);

        CloudNormalT::Ptr cloud_with_normals(new CloudNormalT);
        pcl::concatenateFields(*cloud, normals, *cloud_with_normals);
        {
          CloudNormalT::Ptr tempc(new CloudNormalT);
          bool inverse = true;
          Eigen::Affine3f transform = RT2Transform(*R, *T, inverse); //compute the inverse transform
          pcl::transformPointCloudWithNormals(*cloud_with_normals, *tempc, transform);
          cloud_with_normals.swap(tempc);
        }
        *view = cloud_with_normals;
        return ecto::OK;
      }
      ecto::spore<cv::Mat> R, T, mask, image, points3d;
      ecto::spore<CloudOutT> cloud_out;

    };
  }
}

ECTO_CELL( conversion, object_recognition::conversion::MatToPointCloudXYZ, "MatToPointCloudXYZ",
          "Given a cv::Mat, convert it to pcl::PointCloud<pcl::PointXYZ>.");
