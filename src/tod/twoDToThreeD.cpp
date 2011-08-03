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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

using ecto::tendrils;

/** Ecto implementation of a module that takes
 *
 */
struct TwoDToThreeD
{
  static void
  declare_params(tendrils& p)
  {
    p.declare<bool>("do_points", "outputs a vector of Point3f").required();
    p.declare<bool>("do_point_cloud", "output a PCL point cloud").required();
  }

  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The keypoints we want to 3d-fy");
    inputs.declare<std::vector<cv::Point2f> >("points", "The points we want to 3d-fy (an aternative to the keypoints)");
    inputs.declare<cv::Mat>("K", "The calibration matrix");
    inputs.declare<cv::Mat>("depth", "The depth image");
    outputs.declare<std::vector<cv::Point3f> >("points", "The output 3d points");
    outputs.declare<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> const> >("point_cloud", "The output point cloud");
  }

  void
  configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    do_points_ = params.get<bool>("do_points");
    do_point_cloud_ = params.get<bool>("do_point_cloud");
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int
  process(const tendrils& inputs, tendrils& outputs)
  {
    // We have lam (x,y,1) = K (X,Y,Z), hence lam=Z
    const cv::Mat & depth_image = inputs.get<cv::Mat>("depth");

    cv::Mat_<float> scaled_points;

    // Create the scaled keypoints if any lit is provided
    bool do_organized = false;
    {
      int i = 0;
      const std::vector<cv::KeyPoint> &keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
      const std::vector<cv::Point2f> &points = inputs.get<std::vector<cv::Point2f> >("points");
      if (!points.empty())
      {
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
      else if (!keypoints.empty())
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
      else
      {
        // Process the whole image
        do_organized = true;
        const int width = depth_image.cols;
        const int height = depth_image.rows;
        scaled_points = cv::Mat_<float>(3, width * height);

        for (int v = 0; v < height; ++v)
          for (int u = 0; u < width; ++u)
          {
            float depth = depth_image.at<short int>(v, u);
            scaled_points(0, i) = v * depth;
            scaled_points(1, i) = u * depth;
            scaled_points(2, i) = depth;
            ++i;
          }
      }
    }

    // Figure out the original points
    cv::Mat_<float> points;
    cv::Mat_<float> K = inputs.get<cv::Mat>("K");
    cv::solve(K, scaled_points, points);

    // Fill out the output
    if (do_points_)
    {
      std::vector<cv::Point3f> & ouput = outputs.get<std::vector<cv::Point3f> >("points");
      for (int i = 0; i < points.cols; ++i)
        ouput.push_back(cv::Point3f(points(0, i), points(1, i), points(2, i)));
    }

    if (do_point_cloud_)
    {
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(
          new pcl::PointCloud<pcl::PointXYZ>());
      if (do_organized)
      {
        cloud->width = depth_image.cols;
        cloud->height = depth_image.rows;
      }

      // Copy the cloud over
      cloud->points.reserve(scaled_points.cols);
      for (int i = 0; i < scaled_points.cols; ++i)
        cloud->points.push_back(pcl::PointXYZ(scaled_points(0, i), scaled_points(1, i), scaled_points(2, i)));

      outputs.get<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> const> >("point_cloud") = cloud;
    }

    return 0;
  }
private:
  bool do_points_;
  bool do_point_cloud_;
};

ECTO_CELL(tod, TwoDToThreeD, "TwoDToThreeD",
          "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
