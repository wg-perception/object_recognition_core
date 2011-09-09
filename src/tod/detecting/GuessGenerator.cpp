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

    namespace
    {
      bool
      compare_first(const std::pair<int, int>& lhs, const std::pair<int, int>& rhs)
      {
        return lhs.first < rhs.first;
      }
    }
    /** Ecto implementation of a module that takes
     *
     */
    struct GuessGenerator
    {
      static void
      declare_params(tendrils& p)
      {
        p.declare<std::string> ("json_params", "The parameters, as a JSON string.\n\"min_inliers\": "
          "Minimum number of inliers. \n\"n_ransac_iterations\": Number of RANSAC iterations.\n");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<cv::Mat> ("points3d", "The height by width 3 channel point cloud");
        inputs.declare<std::vector<cv::KeyPoint> > ("keypoints", "The interesting keypoints");
        inputs.declare<std::vector<std::vector<cv::DMatch> > > ("matches", "The list of OpenCV DMatch");
        inputs.declare<std::vector<cv::Mat> > ("matches_3d",
                                               "The corresponding 3d position of those matches. For each point, a 1 by n 3 channel matrix (for x,y and z)");
        outputs.declare<std::vector<ObjectId> > ("object_ids", "the id's of the found objects");
        outputs.declare<std::vector<cv::Mat> > ("Rs", "The rotations of the poses of the found objects");
        outputs.declare<std::vector<cv::Mat> > ("Ts", "The translations of the poses of the found objects");
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        boost::property_tree::ptree param_tree;
        std::stringstream ssparams;
        ssparams << params.get<std::string> ("json_params");
        boost::property_tree::read_json(ssparams, param_tree);

        min_inliers_ = param_tree.get<unsigned int> ("min_inliers");
        n_ransac_iterations_ = param_tree.get<unsigned int> ("n_ransac_iterations");
      }

      void
      filter_clouds(pcl::PointCloud<pcl::PointXYZ> & training_point_cloud,
                    pcl::PointCloud<pcl::PointXYZ> & query_point_cloud)
      {
        pcl::PointCloud<pcl::PointXYZ> training, query;
        float max_dist = 0.005;
        for (unsigned int i = 0; i < training_point_cloud.size(); ++i)
        {
          pcl::PointXYZ & training_point_1 = training_point_cloud.points[i];
          pcl::PointXYZ & query_point_1 = query_point_cloud.points[i];
          unsigned int count = 0;
          for (unsigned int j = 0; j < training_point_cloud.size(); ++j)
          {
            if (j == i)
              continue;

            pcl::PointXYZ & training_point_2 = training_point_cloud.points[j];
            pcl::PointXYZ & query_point_2 = query_point_cloud.points[j];

            float distsq_1 = pcl::euclideanDistance(training_point_1, training_point_2);
            float distsq_2 = pcl::euclideanDistance(query_point_1, query_point_2);
            if (std::abs(distsq_1 - distsq_2) < max_dist)
              ++count;
          }
          if (count >= 3)
          {
            std::cout << count << " ";
            training.push_back(training_point_1);
            query.push_back(query_point_1);
          }
        }
        std::cout << std::endl;

        training_point_cloud = training;
        query_point_cloud = query;
      }

      Eigen::VectorXf
      ransacy(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& training_point_cloud,
              const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & query_point_cloud, const std::vector<int>& good_indices,
              std::vector<int>& inliers)
      {
        pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model(
                                                                        new pcl::SampleConsensusModelRegistration<
                                                                            pcl::PointXYZ>(training_point_cloud,
                                                                                           good_indices));
        pcl::RandomSampleConsensus<pcl::PointXYZ> sample_consensus(model);
        Eigen::VectorXf coefficients;
        model->setInputTarget(query_point_cloud, good_indices);
        sample_consensus.setDistanceThreshold(0.01);
        sample_consensus.setMaxIterations(n_ransac_iterations_);
        if (sample_consensus.computeModel())
        {
          sample_consensus.getInliers(inliers);
          std::sort(inliers.begin(), inliers.end());
          sample_consensus.getModelCoefficients(coefficients);
        }
        else
        {
          inliers.clear();
        }
        return coefficients;
      }
      void
      cluster_clouds(const pcl::PointCloud<pcl::PointXYZ> & training_point_cloud,
                     const pcl::PointCloud<pcl::PointXYZ> & query_point_cloud, std::vector<cv::Mat> &Rs,
                     std::vector<cv::Mat> &Ts)
      {
        pcl::PointCloud<pcl::PointXYZ> training, query;
        std::vector<std::vector<int> > clusters(training_point_cloud.size());
        float max_dist = 0.005;
        float min_span = 0.05;
        float span_query = 0;
        float span_training = 0;
        std::cout << "training_point_cloud size " << training_point_cloud.size() << std::endl;
        for (unsigned int i = 0; i < training_point_cloud.size(); ++i)
        {
          const pcl::PointXYZ & training_point_1 = training_point_cloud.points[i];
          const pcl::PointXYZ & query_point_1 = query_point_cloud.points[i];
          std::vector<int>& cluster = clusters[i];
          cluster.push_back(i);
          for (unsigned int j = 0; j < training_point_cloud.size(); ++j)
          {
            if (j == i)
              continue;

            const pcl::PointXYZ & training_point_2 = training_point_cloud.points[j];
            const pcl::PointXYZ & query_point_2 = query_point_cloud.points[j];

            float dist1 = pcl::euclideanDistance(training_point_1, training_point_2);
            float dist2 = pcl::euclideanDistance(query_point_1, query_point_2);
            if (std::abs(dist1 - dist2) < max_dist)
            {
              cluster.push_back(j);
              span_training = std::max(span_training, dist1);
              span_query = std::max(span_query, dist2);
            }
          }
          //sort the small cluster before exiting. for the set difference.
          if ((span_query < min_span) && (span_training < min_span))
            cluster.clear();
          std::sort(cluster.begin(), cluster.end());
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr
            training_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(training_point_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr query_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(query_point_cloud));
        std::vector<int> inliers;
        bool is_done = false;
        while (!is_done)
        {
          std::vector<std::pair<int, int> > count_clusteridx;
          count_clusteridx.reserve(clusters.size());

          count_clusteridx.clear();
          for (unsigned int i = 0; i < clusters.size(); ++i)
            count_clusteridx.push_back(std::make_pair(clusters[i].size(), i));
          std::sort(count_clusteridx.begin(), count_clusteridx.end(), compare_first);

          is_done = true;
          for (int i = count_clusteridx.size() - 1; i >= 0; i--)
          {
            int count_i, index_i;
            boost::tie(count_i, index_i) = count_clusteridx[i];
            const std::vector<int>& cluster_i = clusters[index_i];
            if (cluster_i.size() < int(min_inliers_))
              break;
            //largest cluster
            inliers.clear();
            Eigen::VectorXf coefficients = ransacy(training_cloud_ptr, query_cloud_ptr, cluster_i, inliers);
            if (inliers.size() >= min_inliers_)
            {
              // Create a pose object
              cv::Mat_<float> R_mat(3, 3), tvec(3, 1);
              for (unsigned int j = 0; j < 3; ++j)
              {
                for (unsigned int i = 0; i < 3; ++i)
                  R_mat(j, i) = coefficients[4 * j + i];
                tvec(j, 0) = coefficients[4 * j + 3];
              }
              Rs.push_back(R_mat);
              Ts.push_back(tvec);

              std::vector<int> temp_cluster;
              // remove inliers from all clusters
              for (unsigned int index = 0; index < clusters.size(); ++index)
              {

                temp_cluster = clusters[index];
                int size = temp_cluster.size();
                clusters[index].resize(temp_cluster.size() + inliers.size());
                std::vector<int>::iterator it;
                it = std::set_difference(temp_cluster.begin(), temp_cluster.end(), inliers.begin(), inliers.end(),
                                         clusters[index].begin());
                //be sure to resize here.
                clusters[index].resize(it - clusters[index].begin());
                std::sort(clusters[index].begin(), clusters[index].end());
                std::cout << "post diff : " << clusters[index].size() << " pre: " << size << std::endl;
              }
              for (unsigned int j = 0; j < inliers.size(); ++j)
              {
                clusters[inliers[j]].clear();
              }
              cluster_i.clear();
              is_done = false;
              break;
            }
            else
            {
              cluster_i.clear();
            }
          }
          //break;//one cluster per object.
        }
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
        const std::vector<std::vector<cv::DMatch> > & matches =
            inputs.get<std::vector<std::vector<cv::DMatch> > > ("matches");
        const std::vector<cv::Mat> & matches_3d = inputs.get<std::vector<cv::Mat> > ("matches_3d");

        // Get the original keypoints and point cloud
        const std::vector<cv::KeyPoint> & keypoints = inputs.get<std::vector<cv::KeyPoint> > ("keypoints");
        cv::Mat point_cloud = inputs.get<cv::Mat> ("points3d");

        // Get the outputs
        std::vector<ObjectId> &object_ids = outputs.get<std::vector<ObjectId> > ("object_ids");
        object_ids.clear();
        std::vector<cv::Mat> &Rs = outputs.get<std::vector<cv::Mat> > ("Rs");
        std::vector<cv::Mat> &Ts = outputs.get<std::vector<cv::Mat> > ("Ts");
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
            cv::Vec3f point3f = point_cloud.at<cv::Vec3f> (keypoint.pt.y, keypoint.pt.x);
            pcl::PointXYZ query_point = pcl::PointXYZ(point3f.val[0], point3f.val[1], point3f.val[2]);

            const std::vector<cv::DMatch> &local_matches = matches[descriptor_index];
            const cv::Mat &local_matches_3d = matches_3d[descriptor_index];
            // Get the matches for that point
            for (unsigned int match_index = 0; match_index < local_matches.size(); ++match_index)
            {
              const cv::Vec3f & val = local_matches_3d.at<cv::Vec3f> (0, match_index);
              pcl::PointXYZ training_point(val[0], val[1], val[2]);

              // Fill in the clouds
              ObjectId object_id = local_matches[match_index].imgIdx;
              training_point_clouds[object_id].push_back(training_point);
              query_point_clouds[object_id].push_back(query_point);
            }
          }

          // For each object, perform 3d to 3d matching
          for (std::map<ObjectId, pcl::PointCloud<pcl::PointXYZ> >::const_iterator query_iterator =
              query_point_clouds.begin(); query_iterator != query_point_clouds.end(); ++query_iterator)
          {
            ObjectId object_id = query_iterator->first;
            cluster_clouds(training_point_clouds[object_id], query_point_clouds[object_id], Rs, Ts);
            object_ids.resize(Rs.size(), object_id); //fill in an object id for every new R and T found.
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
    "Given descriptors and 3D positions, compute object guesses.")
;
