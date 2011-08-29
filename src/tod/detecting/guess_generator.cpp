/*
 * twoDToThreeD.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

#include "opencv_candidate/PoseRT.h"

typedef unsigned int ObjectId;

using ecto::tendrils;

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
      declare_params(tendrils& p)
      {
        p.declare<std::string>(
            "json_params", "The parameters, as a JSON string.\n\"min_inliers\": "
            "Minimum number of inliers. \n\"n_ransac_iterations\": Number of RANSAC iterations.\n");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> const> >("point_cloud", "The point cloud");
        inputs.declare<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const> >("point_cloud_rgb", "The RGB point cloud");
        inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The depth image");
        inputs.declare<std::vector<std::vector<cv::DMatch> > >("matches", "The list of OpenCV DMatch");
        inputs.declare<std::vector<std::vector<cv::Point3f> > >("matches_3d",
                                                                "The corresponding 3d position of those matches");
        outputs.declare<std::vector<ObjectId> >("object_ids", "the id's of the found objects");
        outputs.declare<std::vector<opencv_candidate::Pose> >("poses", "The poses of the found objects");
      }

      void
      configure(const tendrils& params, const tendrils& inputs,  const tendrils& outputs)
      {
        boost::property_tree::ptree param_tree;
        std::stringstream ssparams;
        ssparams << params.get<std::string>("json_params");
        boost::property_tree::read_json(ssparams, param_tree);

        min_inliers_ = param_tree.get<unsigned int>("min_inliers");
        n_ransac_iterations_ = param_tree.get<unsigned int>("n_ransac_iterations");
      }

      /** Get the 2d keypoints and figure out their 3D position from the depth map
       * @param inputs
       * @param outputs
       * @return
       */
      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        // Get the different matches
        const std::vector<std::vector<cv::DMatch> > & matches = inputs.get<std::vector<std::vector<cv::DMatch> > >(
            "matches");
        const std::vector<std::vector<cv::Point3f> > & matches_3d = inputs.get<std::vector<std::vector<cv::Point3f> > >(
            "matches_3d");

        // Get the original keypoints and point cloud
        const std::vector<cv::KeyPoint> & keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> const> point_cloud = inputs.get<
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> const> >("point_cloud");

        if (point_cloud->points.empty())
        {
          boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > new_point_cloud = boost::shared_ptr<
              pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());

          boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const> point_cloud_rgb = inputs.get<
              boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const> >("point_cloud_rgb");
          new_point_cloud->points.reserve(point_cloud_rgb->points.size());
          BOOST_FOREACH(const pcl::PointXYZRGB & point, point_cloud_rgb->points)
                new_point_cloud->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
          point_cloud = new_point_cloud;
        }

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
          for (unsigned int descriptor_index = 0; descriptor_index < matches.size(); ++descriptor_index)
          {
            const cv::KeyPoint & keypoint = keypoints[descriptor_index];
            pcl::PointXYZ query_point = point_cloud->at(keypoint.pt.x, keypoint.pt.y);
            // Check if we have NaN's
            if ((query_point.x != query_point.x) || (query_point.y != query_point.y)
                || (query_point.z != query_point.z))
              continue;

            const std::vector<cv::DMatch> &local_matches = matches[descriptor_index];
            const std::vector<cv::Point3f> &local_matches_3d = matches_3d[descriptor_index];
            // Get the matches for that point
            for (unsigned int match_index = 0; match_index < local_matches.size(); ++match_index)
            {
              const cv::Point3f & training_point3f = local_matches_3d[match_index];

              // Check if we have NaN's
              if ((training_point3f.x != training_point3f.x) || (training_point3f.y != training_point3f.y)
                  || (training_point3f.z != training_point3f.z))
                continue;

              // Check that the dimensions are correct
              if ((std::abs(training_point3f.x) > 1e10) || (std::abs(training_point3f.y) > 1e10)
                  || (std::abs(training_point3f.z) > 1e10))
                continue;

              // Fill in the clouds
              ObjectId object_id = local_matches[match_index].imgIdx;
              training_point_clouds[object_id].push_back(
                  pcl::PointXYZ(training_point3f.x, training_point3f.y, training_point3f.z));
              query_point_clouds[object_id].push_back(query_point);
            }
          }

      // For each object, perform 3d to 3d matching
      for (std::map<ObjectId, pcl::PointCloud<pcl::PointXYZ> >::const_iterator query_iterator = query_point_clouds.begin();
          query_iterator != query_point_clouds.end(); ++query_iterator)
          {
        ObjectId object_id = query_iterator->first;
        unsigned int n_points = query_iterator->second.size();
        if ((n_points < min_inliers_) || (training_point_clouds[object_id].points.size() < 5))
              continue;

        std::vector<int> good_indices;
        for (unsigned int i = 0; i < n_points; ++i)
          good_indices.push_back(i);

        pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model(
            new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(training_point_clouds[object_id].makeShared(),
                                                                     good_indices));
        pcl::RandomSampleConsensus<pcl::PointXYZ> sample_consensus(model);
        //pcl::ProgressiveSampleConsensus<pcl::PointXYZ> sample_consensus(model);
            std::cout << "Object id " << object_id << " has " << query_point_clouds[object_id].points.size()
                      << " possible matches with " << n_ransac_iterations_ << " iterations " << std::endl;

            std::cout << "[ ";
            BOOST_FOREACH(const pcl::PointXYZ &point, training_point_clouds[object_id].points)
            {
              std ::cout << point.x << " " << point.y << " " << point.z << ";";
            }
            std::cout << " ]" << std::endl;
            std::cout << "[ ";
            BOOST_FOREACH(const pcl::PointXYZ &point, query_point_clouds[object_id].points)
            {
              std ::cout << point.x << " " << point.y << " " << point.z << ";";
            }
            std::cout << " ]" << std::endl;
        model->setInputTarget(query_point_clouds[object_id].makeShared(), good_indices);
        sample_consensus.setDistanceThreshold(0.1);
        sample_consensus.setMaxIterations(n_ransac_iterations_);
        sample_consensus.computeModel();
        std::vector<int> inliers;
        sample_consensus.getInliers(inliers);
        std::cout << "n inliers " << inliers.size() << std::endl;
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

ECTO_CELL(tod_detection, object_recognition::tod::GuessGenerator, "GuessGenerator",
          "Given descriptors and 3D positions, compute object guesses.");
