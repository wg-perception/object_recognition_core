/*
 * twoDToThreeD.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

#include "tod/detecting/GuessGenerator.h""

using ecto::tendrils;

/** Ecto implementation of a module that takes
 *
 */
struct GuessGenerator
{
  static void declare_params(tendrils& p)
  {
    p.declare<int>("n_features", "The number of desired features", 1000);
    p.declare<int>("n_levels", "The number of scales", 3);
    p.declare<float>("scale_factor", "The factor between scales", 1.2);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>("point_cloud", "The point cloud");
    inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The depth image");
    inputs.declare<cv::Mat>("descriptors", "The depth image");
    outputs.declare<tod::Guess>("guesses", "The output 3d points");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int process(const tendrils& inputs, tendrils& outputs)
  {
    // We have lam (x,y,1) = K (X,Y,Z), hence lam=Z
    const std::vector<cv::KeyPoint> &keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
    const cv::Mat & depth_image = inputs.get<cv::Mat>("depth");
    cv::Mat_<float> K = inputs.get<cv::Mat>("K");

    unsigned int n_points = keypoints.size();
    cv::Mat_<float> scaled_points(3, n_points);

    // Create the scaled keypoints
    unsigned int i = 0;
    BOOST_FOREACH(const cv::KeyPoint & keypoint, keypoints)
        {
          float depth = depth_image.at<short int>(keypoint.pt.y, keypoint.pt.x);
          scaled_points(0, i) = keypoint.pt.x * depth;
          scaled_points(1, i) = keypoint.pt.y * depth;
          scaled_points(2, i) = depth;
          ++i;
        }

    // Figure out the original points
    cv::Mat_<float> points;
    cv::solve(K, scaled_points, points);

    // Fill out the output
    std::vector<cv::Point3f> ouput;
    for (i = 0; i < n_points; ++i)
      ouput.push_back(cv::Point3f(points(0, i), points(1, i), points(2, i)));
    outputs.get<std::vector<cv::Point3f> >("points") = ouput;

    return 0;
  }
};

void wrap_GuessGenerator()
{
  ecto::wrap<GuessGenerator>("GuessGenerator", "Given ORB descriptors and 3D positions, compute object guesses.");
}
