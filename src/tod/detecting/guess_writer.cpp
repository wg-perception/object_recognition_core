/*
 * twoDToThreeD.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

#include "tod/detecting/GuessGenerator.h"
#include "tod/detecting/Loader.h"
#include "tod/detecting/Recognizer.h"
#include "tod_stub/tod_stub.h"
#include "tod_stub/tod_stub_impl.h"
#include "tod_stub/csv.h"

namespace po = boost::program_options;

using ecto::tendrils;

struct DetectorOptions
{
  /** The path to a ROS bag file
   */
  std::string bag_file_;
  std::string imageFile;
  std::string baseDirectory;
  std::string config;
  tod::TODParameters params;
  int verbose;
  int mode;
};

/** Ecto implementation of a module that takes
 *
 */
struct GuessWriter
{
  static void declare_params(tendrils& p)
  {
    p.declare<std::string>("base_directory", "Base directory");
    p.declare<std::string>("config_file", "Configuration file");
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<std::vector<tod::Guess> >("guesses", "The guesse about the objects and their positions.");
    inputs.declare<pcl::PointCloud<pcl::PointXYZRGB> >("point_cloud", "The original point cloud");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int process(const tendrils& inputs, tendrils& outputs)
  {
    const pcl::PointCloud<pcl::PointXYZRGB> & point_cloud = inputs.get<pcl::PointCloud<pcl::PointXYZRGB> >(
        "point_cloud");

    // match to our objects
    const std::vector<tod::Guess> &guesses  = inputs.get<std::vector<tod::Guess> >("guesses");
    tod_stub::RunInfo run_info;
    tod_stub::Options opts;
    run_info.ts.set();
    run_info.runID = opts.run_number;
    run_info.name = opts.team_name;
    tod_stub::CSVOutput csv_out = openCSV(run_info);
    int dID = 0; //detection id
    BOOST_FOREACH(const tod::Guess &g, guesses)
    {
      tod::PoseRT pose = g.aligned_pose();
      pose.rvec.clone().convertTo(pose.rvec, CV_64F);
      pose.tvec.clone().convertTo(pose.tvec, CV_64F);
      tod_stub::Result r(cv::Mat(), pose.tvec, g.getObject()->name);
      cv::Rodrigues(pose.rvec, r.R);

      tod_stub::PoseInfo poseInfo;
      for (int i = 0; i < 9; i++)
      {
        poseInfo.Rot[i] = r.R.at<double>(i % 3, i / 3);
      }

      poseInfo.Tx = r.T.at<double>(0);
      poseInfo.Ty = r.T.at<double>(1);
      poseInfo.Tz = r.T.at<double>(2);
      poseInfo.ts.set();
      poseInfo.frame = point_cloud.header.seq;
      poseInfo.oID = r.object_id;
      poseInfo.dID = dID++; //training (only one detection per frame)
      writeCSV(csv_out, poseInfo);
    }

    return 0;
  }
};

void wrap_GuessWriter()
{
  ecto::wrap<GuessWriter>("GuessWriter", "Given guesses, writes them to a CSV.");
}
