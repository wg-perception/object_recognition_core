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

#include <vector>

#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

// ROS includes
#include <ros/publisher.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <opencv2/core/core.hpp>

#include <object_recognition_core/common/pose_result.h>
#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/db.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

namespace bp = boost::python;
using object_recognition_core::db::ObjectId;
using object_recognition_core::common::PoseResult;

namespace object_recognition_core
{
  /** Cell that takes the results of object recognition and fills the official ROS message
   */
  struct MsgAssembler
  {
    typedef geometry_msgs::PoseArrayConstPtr PoseArrayMsgPtr;
    typedef geometry_msgs::PoseArray PoseArrayMsg;
    typedef std_msgs::StringConstPtr ObjectIdsMsgPtr;
    typedef std_msgs::String ObjectIdsMsg;

    static void
    declare_params(ecto::tendrils& params)
    {
    }

    static void
    declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare < sensor_msgs::ImageConstPtr > ("image_message", "the image message to get the header");
      inputs.declare(&MsgAssembler::point3d_clusters_, "point3d_clusters", "The sets of 3d points for each object");
      inputs.declare(&MsgAssembler::pose_results_, "pose_results", "The results of object recognition");

      outputs.declare < object_recognition_msgs::RecognizedObjectArrayConstPtr > ("msg", "The poses");
    }

    void
    configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      ecto::py::scoped_call_back_to_python scb;

      image_message_ = inputs["image_message"];
      bp::object mapping;
    }

    int
    process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      // Publish the info
      ros::Time time = ros::Time::now();
      object_recognition_msgs::RecognizedObjectArrayPtr msg(new object_recognition_msgs::RecognizedObjectArray());

      std::string frame_id;
      if ((*image_message_))
        frame_id = (*image_message_)->header.frame_id;

      msg->objects.resize(pose_results_->size());
      {
        size_t object_id = 0;
        BOOST_FOREACH (const object_recognition_core::common::PoseResult & pose_result, *pose_results_)
        {
          object_recognition_msgs::RecognizedObject & object = msg->objects[object_id];

          // Deal with the id
          object.id.id = pose_result.object_id();
          object.id.db = or_json::write(or_json::mValue(pose_result.db().parameters().raw()));

          // Deal with the confidence
          object.confidence = pose_result.confidence();

          // Deal with the pose
          object.pose.header.frame_id = frame_id;
          object.pose.header.stamp = time;

          cv::Mat_<float> T = pose_result.T<cv::Mat_<float> >(), R = pose_result.R<cv::Mat_<float> >();

          geometry_msgs::Pose & msg_pose = object.pose.pose.pose;

          Eigen::Matrix3f rotation_matrix;
          for (unsigned int j = 0; j < 3; ++j)
            for (unsigned int i = 0; i < 3; ++i)
              rotation_matrix(j, i) = R(j, i);

          Eigen::Quaternion<float> quaternion(rotation_matrix);

          msg_pose.position.x = T(0);
          msg_pose.position.y = T(1);
          msg_pose.position.z = T(2);
          msg_pose.orientation.x = quaternion.x();
          msg_pose.orientation.y = quaternion.y();
          msg_pose.orientation.z = quaternion.z();
          msg_pose.orientation.w = quaternion.w();

          // Deal with the header
          object.header.frame_id = frame_id;

          // Deal with the partial point clouds
          object.point_clouds.resize(point3d_clusters_->size());
          for (size_t i = 0; i < point3d_clusters_->size(); ++i)
            pcl::toROSMsg((*point3d_clusters_)[i], object.point_clouds[i]);

          ++object_id;
        }
      }

      // Export the message as final
      outputs["msg"] << object_recognition_msgs::RecognizedObjectArrayConstPtr(msg);
      return ecto::OK;
    }
  private:
    ecto::spore<std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > > point3d_clusters_;
    ecto::spore<std::vector<common::PoseResult> > pose_results_;
    ecto::spore<sensor_msgs::ImageConstPtr> image_message_;
  };
}

ECTO_CELL(io_ros, object_recognition_core::MsgAssembler, "MsgAssembler",
    "Given object ids and poses, fill the object recognition message.");
