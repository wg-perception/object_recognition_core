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

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

#include "opencv_candidate/PoseRT.h"

typedef unsigned int ObjectId;

using ecto::tendrils;

namespace object_recognition
{
  namespace tod
  {

    /** Ecto implementation of a module that takes
     *
     */
    struct PointCloudToMat
    {
      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const> >("point_cloud_rgb",
                                                                                    "The RGB point cloud");
        outputs.declare<cv::Mat>("points", "The width by height by 3 channels (x, y and z)");
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        // Get the original keypoints and point cloud
        typedef pcl::PointXYZRGB PointType;
        boost::shared_ptr<pcl::PointCloud<PointType> const> point_cloud = inputs.get<
            boost::shared_ptr<pcl::PointCloud<PointType> const> >("point_cloud_rgb");

        cv::Mat points;
        if (point_cloud->height != 1) //isOrganized() is not const...
          points = cv::Mat(point_cloud->height, point_cloud->width, CV_32FC3);
        else
          points = cv::Mat(point_cloud->size(), 3, CV_32F);

        float * data = reinterpret_cast<float*>(points.data);
        BOOST_FOREACH(const PointType & point, point_cloud->points)
            {
              *(data++) = point.x;
            }
        outputs.get<cv::Mat>("points") = points;

        return 0;
      }
    };
  }
}

ECTO_CELL(tod_detection, object_recognition::tod::PointCloudToMat, "PointCloudToMat",
          "Given a point cloud, convert it to a width by height matrix with 3 channels (x,y and z).");
