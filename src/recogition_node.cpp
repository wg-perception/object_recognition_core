/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 * Author: Ethan Rublee
 * run with:
 rosrun tod_stub image_pcl_recorder \
 camera:=/camera/rgb points2:=/camera/depth/points2 time_interval:=0.5
 */


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/ros/conversions.h>

using std::string;
using namespace sensor_msgs;

namespace
{
namespace po = boost::program_options;
namespace enc = sensor_msgs::image_encodings;

struct options
{
  int x;
};
int options(int ac, char ** av, pe_options& opts)
{
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "show help");
  desc.add_options()("x", po::value<int>(&opts.x)->default_value(0), "An optional int...");

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);
  if (vm.count("help"))
  {
    std::cerr << desc << std::endl;
    return 1;
  }
  return 0;
}

class RecognitionNode
{
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraInfoSubscriber;
  typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproxSyncPolicy;
  typedef message_filters::Synchronizer<ApproxSyncPolicy> SynchronizerImageDepthCamera;

  ImageSubscriber image_sub_, depth_sub_;
  CameraInfoSubscriber camera_info_sub_, depth_camera_info_sub_;
  SynchronizerImageDepthCamera sync_sub_;
  ros::NodeHandle nh_;
  string camera_topic_, depth_camera_topic_;
  ros::Time prev_;
  options opts_;
public:

  RecognitionNode(options opts) :
    sync_sub_(10),opts_(opts)
  {
    onInit();
  }

  void setupPubs()
  {

  }
  void setupSubs()
  {
    //may be remapped to : /camera/rgb
    camera_topic_ = nh_.resolveName("camera", true);
    depth_camera_topic_ = nh_.resolveName("depth_camera", true);

    ROS_INFO_STREAM("camera topic is set to " << camera_topic_);
    ROS_INFO_STREAM("depth camera topic is set to " <<depth_camera_topic_ );

    //subscribe to rgb camera topics
    image_sub_.subscribe(nh_, camera_topic_ + "/image_color", 2);
    camera_info_sub_.subscribe(nh_, camera_topic_ + "/camera_info", 2);
    
    //subscribe to depth image topics
    depth_sub_.subscribe(nh_, depth_camera_topic_ + "/image", 2);
    depth_camera_info_sub_.subscribe(nh_,  depth_camera_topic_ + "/camera_info", 2);

    //setup the approxiamate time sync
    sync_sub_.connectInput(image_sub_, camera_info_sub_, depth_sub_,depth_camera_info_sub_);
    sync_sub_.registerCallback(&RecognitionNode::dataCallback, this);
  }
  void onInit()
  {

    prev_ = ros::Time::now();

    setupSubs();
    setupPubs();

    ROS_INFO("init done");
  }

  void dataCallback(const ImageConstPtr& image, const CameraInfoConstPtr& camera_info,
                   const ImageConstPtr& depth, const CameraInfoConstPtr& depth_camera_info)
  {
    ros::Time n = image->header.stamp;
    float dt = (n - prev_).toSec();
    ROS_INFO_STREAM("Processing frame bundle. dt=" << dt);
  }
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition");

  options opts;
  if (options(argc, argv, opts))
    return 1;

  RecognitionNode recog_node(opts);
  ros::spin();
  return 0;
}

