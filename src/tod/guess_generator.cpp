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

#include "tod/detecting/GuessGenerator.h"

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
    inputs.declare<pcl::PointCloud<pcl::PointXYZRGB> >("point_cloud", "The point cloud");
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
    const std::vector<cv::KeyPoint> &keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");

    return 0;
  }
};

void wrap_GuessGenerator()
{
  ecto::wrap<GuessGenerator>("GuessGenerator", "Given ORB descriptors and 3D positions, compute object guesses.");
}
