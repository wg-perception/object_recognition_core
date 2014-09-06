/*                                                                                                  
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <ecto/ecto.hpp>

#include <stdlib.h>
#include <string>

#include <Eigen/Core>

#include <object_recognition_core/common/pose_result.h>
#include <object_recognition_core/db/db.h>

using ecto::tendrils;
using ecto::spore;
using object_recognition_core::db::ObjectId;
using object_recognition_core::common::PoseResult;
using object_recognition_core::db::ObjectDbPtr;

struct ConstantSource {
  static void declare_io(const tendrils& params, tendrils& inputs,
      tendrils& outputs) {
    outputs.declare(&ConstantSource::frame_id_, "frame_id",
        "The frame in which everything is computed");
  }

  int process(const tendrils& inputs, const tendrils& outputs) {
    *frame_id_ = "/bogus_frame_id";

    return ecto::OK;
  }

  spore<std::string> frame_id_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ConstantDetector {
  static void declare_params(tendrils& params) {
  }

  static void declare_io(const tendrils& params, tendrils& inputs,
      tendrils& outputs) {
    outputs.declare(&ConstantDetector::pose_results_, "pose_results",
        "The results of object recognition");
  }

  void configure(const tendrils& params, const tendrils& inputs,
      const tendrils& outputs) {
    object_db_ = object_recognition_core::db::ObjectDbParameters(
        object_recognition_core::db::ObjectDbParameters::EMPTY).generateDb();
  }

  int process(const tendrils& inputs, const tendrils& outputs) {
    pose_results_->clear();

    PoseResult result;

    // no db for now, only one model
    result.set_object_id(object_db_, "bogus_name");
    result.set_confidence(1.0);

    // set the clustered cloud's center as a center...
    result.set_T(Eigen::Vector3f(float(std::rand())/RAND_MAX, float(std::rand())/RAND_MAX, float(std::rand())/RAND_MAX));

    // Only one point of view for this object...
    /*sensor_msgs::PointCloud2Ptr cluster_cloud(new sensor_msgs::PointCloud2());
     std::vector<sensor_msgs::PointCloud2ConstPtr> ros_clouds(1);
     pcl::toROSMsg(clusters[object_it], *(cluster_cloud));
     ros_clouds[0] = cluster_cloud;
     result.set_clouds(ros_clouds);*/

    pose_results_->push_back(result);

    return ecto::OK;
  }
  ObjectDbPtr object_db_;

  spore<std::vector<PoseResult> > pose_results_;
};

// register the ECTO cells
ECTO_CELL(pipelines, ConstantSource, "ConstantSource",
          "A source that only spits a frame id.")
ECTO_CELL(pipelines, ConstantDetector, "ConstantDetector",
    "A pipelines that always spits out the same output. Means for testing.")
