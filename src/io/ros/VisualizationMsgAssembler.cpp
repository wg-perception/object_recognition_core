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

// ROS includes
#include <std_msgs/String.h>
// TODO: use the following one and delete the one after
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core/core.hpp>

#include <object_recognition_core/common/pose_result.h>
#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/prototypes/object_info.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

namespace bp = boost::python;
using object_recognition_core::db::ObjectId;
using object_recognition_core::common::PoseResult;

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

namespace object_recognition_core
{
  /** Cell that takes the output message of object recognition and extract from it what be visualized
   */
  struct VisualizationMsgAssembler
  {
    // TODO, switch to the one below or simply remove it from here, that should only be doing visualization
    //typedef std::vector<geometry_msgs::PoseWithCovarianceStamped> PoseArrayMsg;
    typedef geometry_msgs::PoseArray PoseArrayMsg;
    typedef boost::shared_ptr<PoseArrayMsg> PoseArrayMsgPtr;
    typedef boost::shared_ptr<const PoseArrayMsg> PoseArrayMsgConstPtr;
    typedef visualization_msgs::MarkerArrayConstPtr MarkerArrayMsgPtr;
    typedef visualization_msgs::MarkerArray MarkerArrayMsg;
    typedef std_msgs::StringConstPtr ObjectIdsMsgPtr;
    typedef std_msgs::String ObjectIdsMsg;

    static void
    declare_params(ecto::tendrils& params)
    {
    }

    static void
    declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare(&VisualizationMsgAssembler::recognized_objects_, "msg", "The object recognition array msg");

      outputs.declare < PoseArrayMsgConstPtr > ("pose_message", "The poses");
      outputs.declare < ObjectIdsMsgPtr > ("object_ids_message", "The poses");
      outputs.declare < MarkerArrayMsgPtr > ("marker_message", "Visualization markers for ROS.");
    }

    void
    configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      ecto::py::scoped_call_back_to_python scb;

      bp::object mapping;
    }

    int
    process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      PoseArrayMsg pose_array_msg;
      ObjectIdsMsg object_ids_msg;
      MarkerArrayMsg marker_array;
      std::vector<or_json::mValue> object_ids_array;

      // Create poses and fill them in the message
      {
        pose_array_msg.poses.resize((*recognized_objects_)->objects.size());

        unsigned int marker_id = 0;
        BOOST_FOREACH (const object_recognition_msgs::RecognizedObject & recognized_object, (*recognized_objects_)->objects){
        // Deal with the color index
        size_t object_index;
        {
          std::string hash = recognized_object.id.db + recognized_object.id.id;
          if (object_id_to_index_.find(hash) == object_id_to_index_.end())
          object_id_to_index_[hash] = object_id_to_index_.size();
          object_index = object_id_to_index_[hash];
        }

        // Deal with the pose
        pose_array_msg.poses[marker_id] = recognized_object.pose.pose.pose;
        // For now, we assume that all the poses are in the same frame
        pose_array_msg.header = recognized_object.pose.header;

        or_json::mValue db_params;
        or_json::read(recognized_object.id.db, db_params);
        object_recognition_core::db::ObjectDb db = object_recognition_core::db::ObjectDb(
            object_recognition_core::db::ObjectDbParameters(db_params.get_obj()));
        object_recognition_core::prototypes::ObjectInfo object_info(recognized_object.id.id, db);
        const or_json::mObject & attributes = object_info.attributes();

        // Deal with the marker
        {
          visualization_msgs::Marker marker;
          marker.pose = recognized_object.pose.pose.pose;
          marker.type = visualization_msgs::Marker::MESH_RESOURCE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.lifetime = ros::Duration(10);
          marker.header = recognized_object.pose.header;

          marker.scale.x = 1;
          marker.scale.y = 1;
          marker.scale.z = 1;

          float hue = (360.0 / object_id_to_index_.size()) * object_index;

          float r, g, b;
          hsv2rgb(hue, 0.7, 1, r, g, b);

          marker.color.a = 0.75;
          marker.color.g = g;
          marker.color.b = b;
          marker.color.r = r;
          marker.id = 2*marker_id;

          if (attributes.find("mesh_uri") != attributes.end())
          marker.mesh_resource = attributes.find("mesh_uri")->second.get_str();

          marker_array.markers.push_back(marker);
        }
        {
          visualization_msgs::Marker marker;
          marker.pose = recognized_object.pose.pose.pose;
          marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          marker.action = visualization_msgs::Marker::ADD;
          marker.lifetime = ros::Duration(10);
          marker.header = recognized_object.pose.header;

          if (attributes.find("name") != attributes.end())
          marker.text = attributes.find("name")->second.get_str();
          marker.color.a = 1;
          marker.color.g = 1;
          marker.color.b = 1;
          marker.color.r = 1;
          marker.scale.z = 0.03;
          marker.id = 2*marker_id+1;

          marker_array.markers.push_back(marker);
        }
        ++marker_id;

        // Deal with the object_id
        object_ids_array.push_back(or_json::mValue(recognized_object.id.id));
      }
    }

    // Add the object ids to the message
      {
        or_json::mObject object_ids_param_tree;
        object_ids_param_tree["object_ids"] = or_json::mValue(object_ids_array);

        std::stringstream ssparams;

        or_json::mValue value(object_ids_param_tree);
        or_json::write(value, ssparams);
        object_ids_msg.data = ssparams.str();
      }

      outputs["pose_message"] << PoseArrayMsgConstPtr(new PoseArrayMsg(pose_array_msg));
      outputs["object_ids_message"] << ObjectIdsMsgPtr(new ObjectIdsMsg(object_ids_msg));
      outputs["marker_message"] << MarkerArrayMsgPtr(new MarkerArrayMsg(marker_array));
      return ecto::OK;
    }
  private:
    ecto::spore<object_recognition_msgs::RecognizedObjectArrayConstPtr> recognized_objects_;

    /** This structure is used to keep an index for each found obejct, only so that the color of the mesh does not
     * keep changing
     */
    static std::map<ObjectId, unsigned int> object_id_to_index_;
  };
  std::map<ObjectId, unsigned int> VisualizationMsgAssembler::object_id_to_index_;
}

ECTO_CELL(io_ros, object_recognition_core::VisualizationMsgAssembler, "VisualizationMsgAssembler",
    "Given an existing message, create visualization messages from it.");
