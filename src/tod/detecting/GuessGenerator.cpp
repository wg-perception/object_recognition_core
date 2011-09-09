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
        p.declare<std::string>("json_params", "The parameters, as a JSON string.\n\"min_inliers\": "
                               "Minimum number of inliers. \n\"n_ransac_iterations\": Number of RANSAC iterations.\n");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<cv::Mat>("points3d", "The height by width 3 channel point cloud");
        inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The interesting keypoints");
        inputs.declare<std::vector<std::vector<cv::DMatch> > >("matches", "The list of OpenCV DMatch");
        inputs.declare<std::vector<cv::Mat> >(
            "matches_3d",
            "The corresponding 3d position of those matches. For each point, a 1 by n 3 channel matrix (for x,y and z)");
        outputs.declare<std::vector<ObjectId> >("object_ids", "the id's of the found objects");
        outputs.declare<std::vector<cv::Mat> >("Rs", "The rotations of the poses of the found objects");
        outputs.declare<std::vector<cv::Mat> >("Ts", "The translations of the poses of the found objects");
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
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
        const std::vector<cv::Mat> & matches_3d = inputs.get<std::vector<cv::Mat> >("matches_3d");

        // Get the original keypoints and point cloud
        const std::vector<cv::KeyPoint> & keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
        cv::Mat point_cloud = inputs.get<cv::Mat>("points3d");

        // Get the outputs
        std::vector<ObjectId> &object_ids = outputs.get<std::vector<ObjectId> >("object_ids");
        object_ids.clear();
        std::vector<cv::Mat> &Rs = outputs.get<std::vector<cv::Mat> >("Rs");
        std::vector<cv::Mat> &Ts = outputs.get<std::vector<cv::Mat> >("Ts");
        Rs.clear();
        Ts.clear();

        if (point_cloud.empty())
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
          std::vector<int> histo_count;
          for (unsigned int descriptor_index = 0; descriptor_index < matches.size(); ++descriptor_index)
          {
            const cv::KeyPoint & keypoint = keypoints[descriptor_index];
            cv::Vec3f point3f = point_cloud.at<cv::Vec3f>(keypoint.pt.y, keypoint.pt.x);
            pcl::PointXYZ query_point = pcl::PointXYZ(point3f.val[0], point3f.val[1], point3f.val[2]);

            const std::vector<cv::DMatch> &local_matches = matches[descriptor_index];
            const cv::Mat &local_matches_3d = matches_3d[descriptor_index];
            // Get the matches for that point
            for (unsigned int match_index = 0; match_index < local_matches.size(); ++match_index)
            {
              const cv::Vec3f & val = local_matches_3d.at<cv::Vec3f>(0, match_index);
              pcl::PointXYZ training_point(val[0], val[1], val[2]);

              // Fill in the clouds
              ObjectId object_id = local_matches[match_index].imgIdx;
              training_point_clouds[object_id].push_back(training_point);
              query_point_clouds[object_id].push_back(query_point);
              break;
            }
          }

          // For each object, perform 3d to 3d matching
          for (std::map<ObjectId, pcl::PointCloud<pcl::PointXYZ> >::const_iterator query_iterator =
              query_point_clouds.begin(); query_iterator != query_point_clouds.end(); ++query_iterator)
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
            std::cout << "Object id " << object_id << " has " << query_iterator->second.points.size()
                      << " possible matches with " << n_ransac_iterations_ << " iterations " << std::endl;

            model->setInputTarget(query_point_clouds[object_id].makeShared(), good_indices);
            sample_consensus.setDistanceThreshold(0.01);
            sample_consensus.setMaxIterations(n_ransac_iterations_);
            bool success = sample_consensus.computeModel();
            std::vector<int> inliers;
            sample_consensus.getInliers(inliers);
            std::cout << "Success ? " << success << " Above inlier thresh: " << ( inliers.size() >= min_inliers_ ) << std::endl;
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
              Rs.push_back(R_mat);
              Ts.push_back(tvec);
              // And store it in the outputs
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
