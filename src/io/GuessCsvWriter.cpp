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
 */

#include <ecto/ecto.hpp>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>

#include <object_recognition_core/common/pose_result.h>
#include "csv.h"

using ecto::tendrils;
using object_recognition_core::db::ObjectId;

namespace object_recognition_core
{
  namespace io
  {
    /** Ecto implementation of a module that takes object recognition results and writes them to a CSV file
     */
    struct GuessCsvWriter
    {
      static void
      declare_params(tendrils& p)
      {
        p.declare<std::string>("team_name", "The name of the team to consider");
        p.declare<int>("run_number", "The run number");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare(&GuessCsvWriter::pose_results_, "pose_results", "The results of object recognition");
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        team_name_ = params.get<std::string>("team_name");
        run_number_ = params.get<int>("run_number");
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
        RunInfo run_info;
        run_info.ts.set();
        run_info.runID = run_number_;
        run_info.name = team_name_;
        CSVOutput csv_out = openCSV(run_info);
        int dID = 0; //detection id
        BOOST_FOREACH(const common::PoseResult & pose_result, *pose_results_)
            {
              const ObjectId & object_id = pose_result.object_id();
              cv::Matx33f R = pose_result.R<cv::Matx33f>();
              cv::Vec3f T = pose_result.T<cv::Vec3f>();

              PoseInfo poseInfo;
              for (int i = 0; i < 9; i++)
                poseInfo.Rot[i] = R(i % 3, i / 3);

              poseInfo.Tx = T(0);
              poseInfo.Ty = T(1);
              poseInfo.Tz = T(2);
              poseInfo.ts.set();
              //poseInfo.frame = point_cloud.header.seq;
              poseInfo.oID = object_id;
              poseInfo.dID = dID++; //training (only one detection per frame)
              writeCSV(csv_out, poseInfo);
            }

        return 0;
      }
    private:
      int run_number_;
      std::string team_name_;
      ecto::spore<std::vector<common::PoseResult> > pose_results_;
    };
  }
}

ECTO_CELL(io, object_recognition_core::io::GuessCsvWriter, "GuessCsvWriter",
          "Given guesses, writes them to a CSV in the NIST format.")
