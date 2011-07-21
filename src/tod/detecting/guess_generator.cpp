/*
 * twoDToThreeD.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

#include "opencv_candidate/PoseRT.h"

namespace po = boost::program_options;

typedef unsigned int ObjectId;

struct DetectorOptions
{
  /** The path to a ROS bag file
   */
  std::string bag_file_;
  std::string imageFile;
  std::string baseDirectory;
  std::string config;
  int verbose;
  int mode;
};

/** Ecto implementation of a module that takes
 *
 */
struct GuessGenerator
{
  static void declare_params(ecto::tendrils& p)
  {
    p.declare<std::string>("base_directory", "Base directory");
    p.declare<std::string>("config_file", "Configuration file");
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    inputs.declare<pcl::PointCloud<pcl::PointXYZRGB> >("point_cloud", "The point cloud");
    inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The depth image");
    inputs.declare<std::vector<std::vector<cv::DMatch> > >("matches", "The list of OpenCV DMatch");
    inputs.declare<std::vector<std::vector<cv::Point3f> > >("matches_3d",
                                                            "The corresponding 3d position of those matches");
    outputs.declare<std::vector<ObjectId> >("object_ids", "the id's of the found objects");
    outputs.declare<std::vector<opencv_candidate::Pose> >("poses", "The poses of the found objects");
  }

  void configure(ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    std::string config_file = params.get<std::string>("config_file");
    DetectorOptions opts;

    po::variables_map vm;

    po::options_description desc("Allowed options");
    desc.add_options()("base,B", po::value<std::string>(&opts.baseDirectory)->default_value("./"),
                       "The directory that the training base is in.");
    desc.add_options()("verbose,V", po::value<int>(&opts.verbose)->default_value(0), "Verbosity level.");

    boost::filesystem::path cf(config_file);
    if (boost::filesystem::exists(cf))
    {
      std::ifstream cf_is(cf.string().c_str());

      po::store(po::parse_config_file(cf_is, desc, false), vm);
      po::notify(vm);
    }
    else
    {
      std::cerr << cf << " does not exist!" << std::endl;
      desc.print(std::cout);
      throw std::runtime_error("Bad options");
    }

    if (!vm.count("base"))
    {
      std::cout << "Must supply training base directory." << "\n";
      std::cout << desc << std::endl;
      throw std::runtime_error("Bad options");
    }
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    // Get the different matches
    const std::vector<std::vector<cv::DMatch> > & matches = inputs.get<std::vector<std::vector<cv::DMatch> > >(
        "matches");
    const std::vector<std::vector<cv::Point3f> > & matches_3d = inputs.get<std::vector<std::vector<cv::Point3f> > >(
        "matches_3d");

    // Get the original keypoints and point cloud
    const pcl::PointCloud<pcl::PointXYZRGB> & point_cloud = inputs.get<pcl::PointCloud<pcl::PointXYZRGB> >(
        "point_cloud");

    // Get the outputs
    std::vector<ObjectId> &object_ids = outputs.get<std::vector<ObjectId> >("object_ids");
    std::vector<opencv_candidate::Pose> &poses = outputs.get<std::vector<opencv_candidate::Pose> >("poses");

    if (point_cloud.points.empty())
    {
      // Only use 2d to 3d matching
      // TODO
      //const std::vector<cv::KeyPoint> &keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
    }
    else
    {
      // Use 3d to 3d matching

      // Cluster the matches per object ID
      std::map<ObjectId, pcl::PointCloud<pcl::PointXYZ> > query_point_clouds;
      std::map<ObjectId, pcl::PointCloud<pcl::PointXYZ> > training_point_clouds;
      for (unsigned int matches_index = 0; matches_index < matches.size(); ++matches_index)
      {
        const std::vector<cv::DMatch> &local_matches = matches[matches_index];
        const std::vector<cv::Point3f> &local_matches_3d = matches_3d[matches_index];
        for (unsigned int match_index = 0; match_index < local_matches.size(); ++match_index)
        {
          pcl::PointXYZRGB query_point = point_cloud[local_matches[match_index].trainIdx];

          // TODO: replace this by doing 3d to 3d with an unknown depth for that point
          if ((query_point.x != query_point.x) || (query_point.y != query_point.y) || (query_point.z != query_point.z))
            continue;

          ObjectId object_id = local_matches[match_index].imgIdx;
          // Fill in the training cloud
          const cv::Point3d & training_point3f = local_matches_3d[match_index];
          training_point_clouds[object_id].push_back(
              pcl::PointXYZ(training_point3f.x, training_point3f.y, training_point3f.z));
          // Fill in the query cloud
          query_point_clouds[object_id].push_back(pcl::PointXYZ(query_point.x, query_point.y, query_point.z));
        }
      }

      // For each object, perform 3d to 3d matching
      for (std::map<ObjectId, pcl::PointCloud<pcl::PointXYZ> >::const_iterator query_iterator;
          query_iterator != query_point_clouds.end(); ++query_iterator)
          {
        ObjectId object_id = query_iterator->first;
        unsigned int n_points = query_iterator->second.size();
        if (n_points < min_inliers_)
          continue;

        std::vector<int> good_indices;
        for (unsigned int i = 0; i < n_points; ++i)
          good_indices.push_back(i);

        pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model(
            new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(training_point_clouds[object_id].makeShared(),
                                                                     good_indices));
        //pcl::RandomSampleConsensus<pcl::PointXYZ> sample_consensus(model);
        pcl::ProgressiveSampleConsensus<pcl::PointXYZ> sample_consensus(model);

        model->setInputTarget(query_point_clouds[object_id].makeShared(), good_indices);
        sample_consensus.setDistanceThreshold(0.01);
        sample_consensus.setMaxIterations(n_ransac_iterations_);
        sample_consensus.computeModel();
        std::vector<int> inliers;
        sample_consensus.getInliers(inliers);
        if (inliers.size() >= min_inliers_)
        {
          // Create a pose object
          Eigen::VectorXf coefficients;
          sample_consensus.getModelCoefficients(coefficients);

          cv::Mat_<float> R_mat(3, 3), tvec(3, 1);
          for (unsigned int j = 0; j < 3; ++j)
          {
            for (unsigned int i = 0; i < 3; ++i)
              R_mat(j, i) = coefficients[4 * j + i];
            tvec(j, 0) = coefficients[4 * j + 3];
          }
          opencv_candidate::Pose pose;
          pose.setR(R_mat);
          pose.setT(tvec);
          // And store it in the outputs
          poses.push_back(pose);
          object_ids.push_back(object_id);
        }
      }
    }

    return 0;
  }
private:
  /** The minimum number of inliers in order to do pose matching */
  unsigned int min_inliers_;
  /** The number of RANSAC iterations to perform */
  unsigned int n_ransac_iterations_;
}
;

void wrap_GuessGenerator()
{
  ecto::wrap<GuessGenerator>("GuessGenerator", "Given ORB descriptors and 3D positions, compute object guesses.");
}
