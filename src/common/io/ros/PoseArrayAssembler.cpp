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
#include <boost/property_tree/json_parser.hpp>

#include <ecto/ecto.hpp>

#include <Eigen/Geometry>

// ROS includes
#include <ros/publisher.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core/core.hpp>

#include "object_recognition/common/types.h"

namespace bp = boost::python;

namespace
{
  // see
  // http://en.wikipedia.org/wiki/HSL_and_HSV#Converting_to_RGB
  // for points on a dark background you want somewhat lightened
  // colors generally... back off the saturation (s)
  static void
  hsv2rgb(float h, float s, float v, float& r, float& g, float& b)
  {
    float c = v * s;
    float hprime = h / 60.0;
    float x = c * (1.0 - fabs(fmodf(hprime, 2.0f) - 1));

    r = g = b = 0;

    if (hprime < 1)
    {
      r = c;
      g = x;
    }
    else if (hprime < 2)
    {
      r = x;
      g = c;
    }
    else if (hprime < 3)
    {
      g = c;
      b = x;
    }
    else if (hprime < 4)
    {
      g = x;
      b = c;
    }
    else if (hprime < 5)
    {
      r = x;
      b = c;
    }
    else if (hprime < 6)
    {
      r = c;
      b = x;
    }

    float m = v - c;
    r += m;
    g += m;
    b += m;
  }
}

namespace object_recognition
{
  struct PoseArrayAssembler
  {
    typedef geometry_msgs::PoseArrayConstPtr PoseArrayMsgPtr;
    typedef visualization_msgs::MarkerArrayConstPtr MarkerArrayMsgPtr;
    typedef visualization_msgs::MarkerArray MarkerArrayMsg;
    typedef geometry_msgs::PoseArray PoseArrayMsg;
    typedef std_msgs::StringConstPtr ObjectIdsMsgPtr;
    typedef std_msgs::String ObjectIdsMsg;

    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare<bp::object>("mapping", "Mapping from object ids to mesh ids.").required(true);
    }

    static void
    declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<std::vector<ObjectId> >("object_ids", "the id's of the found objects");
      inputs.declare<sensor_msgs::ImageConstPtr>("image_message", "the image message to get the header");
      inputs.declare<std::vector<cv::Mat> >("Rs", "The rotations of the poses of the found objects");
      inputs.declare<std::vector<cv::Mat> >("Ts", "The translations of the poses of the found objects");

      outputs.declare<PoseArrayMsgPtr>("pose_message", "The poses");
      outputs.declare<ObjectIdsMsgPtr>("object_ids_message", "The poses");
      outputs.declare<MarkerArrayMsgPtr>("marker_message", "Visualization markers for ROS.");
    }

    void
    configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      Rs_ = inputs["Rs"];
      Ts_ = inputs["Ts"];
      image_message_ = inputs["image_message"];
      object_ids_ = inputs["object_ids"];
      bp::object mapping;
      params["mapping"] >> mapping;
      bp::list l = bp::dict(mapping).items();
      for (int j = 0, end = bp::len(l); j < end; ++j)
      {
        bp::object key = l[j][0];
        bp::object value = l[j][1];
        std::string object_id = bp::extract<std::string>(key);
        std::string mesh_id = bp::extract<std::string>(value);
        mapping_[object_id] = mesh_id;
      }
    }

    std::string
    get_mesh_id(const std::string& object_id)
    {
      return mapping_[object_id];
    }

    int
    process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      PoseArrayMsg pose_array_msg;
      ObjectIdsMsg object_ids_msg;
      // Publish the info
      ros::Time time = ros::Time::now();
      std::string frame_id = (*image_message_)->header.frame_id;
      pose_array_msg.header.stamp = time;
      pose_array_msg.header.frame_id = frame_id;
      MarkerArrayMsg marker_array;

      BOOST_FOREACH(const ObjectId &object_id, *object_ids_)
            if (object_id_to_index_.find(object_id) == object_id_to_index_.end())
              object_id_to_index_[object_id] = object_id_to_index_.size();

      // Create poses and fill them in the message
      {
        std::vector<geometry_msgs::Pose> &poses = pose_array_msg.poses;
        poses.resize(Rs_->size());

        unsigned int i;
        for (i = 0; i < Rs_->size(); ++i)
        {
          cv::Mat_<float> T, R;
          (*Ts_)[i].convertTo(T, CV_32F);
          (*Rs_)[i].convertTo(R, CV_32F);

          geometry_msgs::Pose & msg_pose = poses[i];

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

          visualization_msgs::Marker marker;
          marker.pose = msg_pose;
          marker.type = visualization_msgs::Marker::MESH_RESOURCE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.lifetime = ros::Duration(30);
          marker.header = pose_array_msg.header;
          marker.scale.x = 1;
          marker.scale.y = 1;
          marker.scale.z = 1;

          float hue = (360.0 / object_id_to_index_.size()) * object_id_to_index_[(*object_ids_)[i]];

          float r, g, b;
          hsv2rgb(hue, 0.7, 1, r, g, b);

          marker.color.a = 0.75;
          marker.color.g = g;
          marker.color.b = b;
          marker.color.r = r;
          marker.id = i;
          //http://localhost:5984/object_recognition/_design/models/_view/by_object_id_and_mesh?key=%2212a1e6eb663a41f8a4fb9baa060f191c%22
          marker.mesh_resource = "http://localhost:5984/object_recognition/" + get_mesh_id((*object_ids_)[i])
                                 + "/mesh.stl";
          marker_array.markers.push_back(marker);
        }
      }

      // Add the object ids to the message
      {
        boost::property_tree::ptree object_ids_param_tree;
        boost::property_tree::ptree object_ids_array;
        BOOST_FOREACH(const ObjectId & object_id, *object_ids_)
              object_ids_array.push_back(std::make_pair("", object_id));
        object_ids_param_tree.push_back(std::make_pair("object_ids", object_ids_array));

        std::stringstream ssparams;
        boost::property_tree::write_json(ssparams, object_ids_param_tree);

        object_ids_msg.data = ssparams.str();
      }

      outputs["pose_message"] << PoseArrayMsgPtr(new PoseArrayMsg(pose_array_msg));
      outputs["object_ids_message"] << ObjectIdsMsgPtr(new ObjectIdsMsg(object_ids_msg));
      outputs["marker_message"] << MarkerArrayMsgPtr(new MarkerArrayMsg(marker_array));
      return 0;
    }
  private:
    ecto::spore<std::vector<cv::Mat> > Rs_;
    ecto::spore<std::vector<cv::Mat> > Ts_;
    ecto::spore<std::vector<std::string> > object_ids_;
    ecto::spore<sensor_msgs::ImageConstPtr> image_message_;

    std::map<std::string, std::string> mapping_;
    static std::map<ObjectId, unsigned int> object_id_to_index_;
  };
  std::map<ObjectId, unsigned int> PoseArrayAssembler::object_id_to_index_;
}

ECTO_CELL(io_ros, object_recognition::PoseArrayAssembler, "PoseArrayAssembler",
          "Given object ids and poses, create PoseArray message.");
