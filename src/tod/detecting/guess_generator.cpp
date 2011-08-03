/*
 * twoDToThreeD.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

#include "opencv_candidate/PoseRT.h"

typedef unsigned int ObjectId;

namespace object_recognition
{
  namespace tod
  {

    /** Ecto implementation of a module that takes
     *
     */
    struct GuessGenerator
    {
      static void
      declare_params(ecto::tendrils& p)
      {
        p.declare<std::string>(
            "json_params", "The parameters, as a JSON string.\n\"min_inliers\": "
            "Minimum number of inliers. \n\"n_ransac_iterations\": Number of RANSAC iterations.\n");
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        inputs.declare<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const> >("point_cloud", "The point cloud");
        inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The depth image");
        inputs.declare<std::vector<std::vector<cv::DMatch> > >("matches", "The list of OpenCV DMatch");
        inputs.declare<std::vector<std::vector<cv::Point3f> > >("matches_3d",
                                                                "The corresponding 3d position of those matches");
        outputs.declare<std::vector<ObjectId> >("object_ids", "the id's of the found objects");
        outputs.declare<std::vector<opencv_candidate::Pose> >("poses", "The poses of the found objects");
      }

      void
      configure(ecto::tendrils& json_params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        boost::property_tree::ptree param_tree;
        std::stringstream ssparams;
        ssparams << json_params.get<std::string>("json_params");
        boost::property_tree::read_json(ssparams, param_tree);

        min_inliers_ = param_tree.get<unsigned int>("min_inliers");
        n_ransac_iterations_ = param_tree.get<unsigned int>("n_ransac_iterations");
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
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const> point_cloud = inputs.get<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const> >(
        "point_cloud");

    // Get the outputs
    std::vector<ObjectId> &object_ids = outputs.get<std::vector<ObjectId> >("object_ids");
    std::vector<opencv_candidate::Pose> &poses = outputs.get<std::vector<opencv_candidate::Pose> >("poses");

    if (point_cloud->points.empty())
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
          pcl::PointXYZRGB query_point = point_cloud->at(size_t(local_matches[match_index].trainIdx),0);

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
      for (std::map<ObjectId, pcl::PointCloud<pcl::PointXYZ> >::const_iterator query_iterator = query_point_clouds.begin();
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
};
}
}

ECTO_CELL(tod, object_recognition::tod::GuessGenerator, "GuessGenerator",
          "Given descriptors and 3D positions, compute object guesses.");
