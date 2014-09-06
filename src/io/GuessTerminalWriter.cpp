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

#include <ecto/ecto.hpp>

#include <iostream>

#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>

#include <object_recognition_core/common/pose_result.h>

using ecto::tendrils;
using object_recognition_core::common::PoseResult;
using object_recognition_core::db::ObjectId;

namespace object_recognition_core
{
  namespace io
  {
    /** Ecto implementation of a module that takes object id and poses and that
     * outputs them to the command line
     */
    struct GuessTerminalWriter
    {
      static void
      declare_params(tendrils& p)
      {
        p.declare<std::string>("base_directory", "Base directory");
        p.declare<std::string>("config_file", "Configuration file");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare(&GuessTerminalWriter::pose_results_, "pose_results", "The results of object recognition");
      }

      /** Get the 2d keypoints and figure out their 3D position from the depth map
       * @param inputs
       * @param outputs
       * @return
       */
      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        // match to our objects
        BOOST_FOREACH(const common::PoseResult & pose_result, *pose_results_)
            {
              const ObjectId & object_id = pose_result.object_id();
              cv::Matx33f R = pose_result.R<cv::Matx33f>();
              cv::Vec3f T = pose_result.T<cv::Vec3f>();

              //poseInfo.frame = point_cloud.header.seq;
              std::cout << "Found object " << object_id << " with pose (R,t) = " << std::endl << cv::Mat(R) << " " << cv::Mat(T)
                        << std::endl;
            }

        return 0;
      }
    private:
      /** The object recognition results */
      ecto::spore<std::vector<common::PoseResult> > pose_results_;
    };
  }
}

ECTO_CELL(io, object_recognition_core::io::GuessTerminalWriter, "GuessTerminalWriter",
          "Given guesses, writes them to the terminal.")
