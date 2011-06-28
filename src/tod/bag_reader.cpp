/*
 * bag_reader.cpp
 *
 *  Created on: Jun 23, 2011
 *      Author: vrabaud
 */

#include <iostream>
#include <string>

#include <cv_bridge/CvBridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/core/core.hpp>

#include <ecto/ecto.hpp>

#include "tod_stub/tod_stub.h"
#include "opencv_candidate/PoseRT.h"

struct BagReader
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud_t;

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<std::string>("path", "The path of the bag");
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    outputs.declare<cv::Mat>("img", "The image that will be analyzed");
    outputs.declare<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>("pcd", "The point cloud");
    outputs.declare<cv::Mat>("K", "K for the camera");
    outputs.declare<cv::Mat>("D", "D for the camera");
  }

  void configure(ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    bag_.open(params.get<std::string>("path"), rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back("image_mono");
    topics.push_back("camera_info");
    topics.push_back("points");

    for (size_t i = 0; i < topics.size(); i++)
      std::cout << "looking at topic:" << topics[i] << std::endl;

    view_.addQuery(bag_, rosbag::TopicQuery(topics));
    message_ = view_.begin();
  }

  int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    tod_stub::ImagePointsCamera ipc_package;

    while ((!ipc_package.full()) && (message_ != view_.end()))
    {
      sensor_msgs::ImageConstPtr img = message_->instantiate<sensor_msgs::Image>();
      if (img != NULL)
      {
        ipc_package.img = img;
      }
      sensor_msgs::CameraInfoConstPtr camera_info = message_->instantiate<sensor_msgs::CameraInfo>();
      if (camera_info != NULL)
      {
        ipc_package.camera_info = camera_info;
      }

      /*
       Cloud_t::ConstPtr points2 = message_->instantiate<Cloud_t>();
       if (points2 != NULL)
       {
       ipc_package.points2 = points2;
       }
       */

      ++message_;
    }

    if (ipc_package.full())
    {
      sensor_msgs::CvBridge bridge;
      outputs.get<cv::Mat>("img") = cv::Mat(bridge.imgMsgToCv(ipc_package.img, "bgr8"));
      outputs.get<pcl::PointCloud<pcl::PointXYZRGB> >("pcd") = *(ipc_package.points2);
      outputs.get<cv::Mat>("K") = cv::Mat(3, 3, CV_64F, (void*)ipc_package.camera_info->K.elems).clone();
      outputs.get<cv::Mat>("D") = cv::Mat(ipc_package.camera_info->D).clone();
      ipc_package.clear();
    }
    else
    {
      std::cerr << "toto" << std::endl;
      return ecto::QUIT;
    }

    ++message_;

    if (message_ == view_.end())
      return ecto::QUIT;

    return 0;
  }
  rosbag::Bag bag_;
  rosbag::View view_;
  rosbag::View::iterator message_;
};

void wrap_BagReader()
{
  ecto::wrap<BagReader>("BagReader", "Reads a TOD bag");
}
