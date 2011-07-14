/*
 * twoDToThreeD.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */

#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

using ecto::tendrils;

/** Ecto implementation of a module that takes
 *
 */
struct TwoDToThreeD
{
  static void declare_params(tendrils& p)
  {
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The keypoints we want to 3d-fy");
    inputs.declare<std::vector<cv::Point2f> >("points", "The points we want to 3d-fy (an aternative to the keypoints)");
    inputs.declare<cv::Mat>("K", "The calibration matrix");
    inputs.declare<cv::Mat>("depth", "The depth image");
    outputs.declare<std::vector<cv::Point3f> >("points", "The output 3d points");
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

    cv::Mat_<float> scaled_points;

    // Create the scaled keypoints
    int i = 0;
    if (keypoints.empty())
    {
      const std::vector<cv::Point2f> &points = inputs.get<std::vector<cv::Point2f> >("points");
      scaled_points = cv::Mat_<float>(3, points.size());
      BOOST_FOREACH(const cv::Point2f & point, points)
          {
            float depth = depth_image.at<short int>(point.y, point.x);
            scaled_points(0, i) = point.x * depth;
            scaled_points(1, i) = point.y * depth;
            scaled_points(2, i) = depth;
            ++i;
          }
    }
    else
    {
      scaled_points = cv::Mat_<float>(3, keypoints.size());
      BOOST_FOREACH(const cv::KeyPoint & keypoint, keypoints)
          {
            float depth = depth_image.at<short int>(keypoint.pt.y, keypoint.pt.x);
            scaled_points(0, i) = keypoint.pt.x * depth;
            scaled_points(1, i) = keypoint.pt.y * depth;
            scaled_points(2, i) = depth;
            ++i;
          }
    }

    // Figure out the original points
    cv::Mat_<float> points;
    cv::Mat_<float> K = inputs.get<cv::Mat>("K");
    cv::solve(K, scaled_points, points);

    // Fill out the output
    std::vector<cv::Point3f> ouput;
    for (i = 0; i < points.cols; ++i)
      ouput.push_back(cv::Point3f(points(0, i), points(1, i), points(2, i)));
    outputs.get<std::vector<cv::Point3f> >("points") = ouput;

    return 0;
  }
};

void wrap_TwoDToThreeD()
{
  ecto::wrap<TwoDToThreeD>(
      "TwoDToThreeD",
      "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
}
