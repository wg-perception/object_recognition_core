/*
 * cameraToWorld.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */
#include <iostream>
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
  static void declare_params(tendrils& p)
  {
    p.declare<int>("n_features", "The number of desired features", 1000);
    p.declare<int>("n_levels", "The number of scales", 3);
    p.declare<float>("scale_factor", "The factor between scales", 1.2);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<std::vector<cv::Point3f> >("points", "The keypoints");
    inputs.declare<cv::Mat>("R", "The rotation matrix");
    inputs.declare<cv::Mat>("T", "The translation vector");
    outputs.declare<cv::Mat>("points", "The keypoints");
  }

  void configure(const tendrils& params, const tendrils& inputs,const tendrils& outputs)
  {
  }

  int process(const tendrils& inputs,const tendrils& outputs)
  {
    cv::Mat_<float> R = inputs.get<cv::Mat>("R");
    const cv::Mat & T = inputs.get<cv::Mat>("T");
    const std::vector<cv::Point3f> in_points = inputs.get<std::vector<cv::Point3f> >("points");
    cv::Mat_<float> points(3, in_points.size());
    unsigned int i = 0;

    // Create the centered points
    BOOST_FOREACH(const cv::Point3f & point, in_points)
        {
          points(0, i) = point.x - T.at<float>(0);
          points(1, i) = point.y - T.at<float>(1);
          points(2, i) = point.z - T.at<float>(2);
        }

    // Apply the inverse rotation
    outputs.get<cv::Mat>("points") = R.t() * points;

    return 0;
  }
};

ECTO_CELL(tod, CameraToWorld, "CameraToWorld",
          "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
