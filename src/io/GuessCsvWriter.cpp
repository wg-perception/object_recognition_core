/*
 * twoDToThreeD.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>

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
              cv::Mat_<float> R = pose_result.R<cv::Mat_<float> >(), T = pose_result.T<cv::Mat_<float> >();

              PoseInfo poseInfo;
              for (int i = 0; i < 9; i++)
                poseInfo.Rot[i] = R.at<float>(i % 3, i / 3);

              poseInfo.Tx = T.at<float>(0);
              poseInfo.Ty = T.at<float>(1);
              poseInfo.Tz = T.at<float>(2);
              poseInfo.ts.set();
              //poseInfo.frame = point_cloud.header.seq;
              poseInfo.oID = object_id;
              std::cout << "Found object " << object_id << " with pose (R,t) = " << std::endl << R << " " << T
                        << std::endl;
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
