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
#include <opencv2/highgui/highgui.hpp>

#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

#include "object_recognition/common/types.h"
#include "impl/maximum_clique.h"
#include "impl/sac_model_registration_graph.h"

using ecto::tendrils;

namespace
{
  void
  fill_connections(const pcl::PointCloud<pcl::PointXYZ> &training_point_cloud,
                   const pcl::PointCloud<pcl::PointXYZ> & query_point_cloud,
                   const std::vector<unsigned int> &query_indices, float object_span, float sensor_error,
                   object_recognition::maximum_clique::Graph &graph)
  {
    // The error the 3d sensor makes, distance wise
    cv::Mat_<float> distances(training_point_cloud.size(), training_point_cloud.size());
    for (unsigned int i = 0; i < training_point_cloud.size(); ++i)
    {
      const pcl::PointXYZ & training_point_1 = training_point_cloud.points[i], &query_point_1 =
          query_point_cloud.points[i];
      // For every other match that might end up in the same cluster
      for (unsigned int j = i + 1; j < training_point_cloud.size(); ++j)
      {
        // Two matches with the same query point cannot be connected
        if (query_indices[i] == query_indices[j])
          continue;
        // Two training points can be connected if they are within the span of an object
        const pcl::PointXYZ & query_point_2 = query_point_cloud.points[j];
        float dist_query = pcl::euclideanDistance(query_point_1, query_point_2);
        //distances(i, j) = dist2;
        //distances(j, i) = dist2;
        if (dist_query > (object_span + 2 * sensor_error))
          continue;

        const pcl::PointXYZ & training_point_2 = training_point_cloud.points[j];
        float dist_training = pcl::euclideanDistance(training_point_1, training_point_2);
        // Make sure the distance between two points is somewhat conserved
        if (std::abs(dist_training - dist_query) > 2 * sensor_error)
          continue;

        // If all those conditions are respected, those two matches are potentially part of the same cluster
        graph.addEdge(i, j);
      }
    }
  }
}

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
        p.declare<std::string>("json_params", "The parameters, as a JSON string.\n\"min_inliers\": "
                               "Minimum number of inliers. \n\"n_ransac_iterations\": Number of RANSAC iterations.\n");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<cv::Mat>("image", "The height by width 3 channel point cloud");
        inputs.declare<cv::Mat>("points3d", "The height by width 3 channel point cloud");
        inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The interesting keypoints");
        inputs.declare<std::vector<std::vector<cv::DMatch> > >("matches", "The list of OpenCV DMatch");
        inputs.declare<std::vector<cv::Mat> >(
            "matches_3d",
            "The corresponding 3d position of those matches. For each point, a 1 by n 3 channel matrix (for x,y and z)");
        inputs.declare<std::map<ObjectId, float> >("spans", "For each found object, its span based on known features.");
        inputs.declare<std::map<ObjectOpenCVId, ObjectId> >(
            "id_correspondences", "Correspondences from OpenCV integer id to the JSON object ids");

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

        debug_ = true;
        sensor_error_ = 0.015;
      }

      Eigen::VectorXf
      ransacy(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& training_point_cloud,
              const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & query_point_cloud, const std::vector<int>& good_indices,
              std::vector<int>& inliers)
      {
        pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model(
            new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(training_point_cloud, good_indices));
        pcl::RandomSampleConsensus<pcl::PointXYZ> sample_consensus(model);
        Eigen::VectorXf coefficients;
        model->setInputTarget(query_point_cloud, good_indices);
        sample_consensus.setDistanceThreshold(sensor_error_);
        sample_consensus.setMaxIterations(n_ransac_iterations_);

        /*std::cout << "a=[";
         BOOST_FOREACH(int i, good_indices) {
         const pcl::PointXYZ &training_point =training_point_cloud->points[i];
         std::cout << training_point.x << "," << training_point.y << "," << training_point.z << ";";
         }
         std::cout << "];"  << std::endl << "b=";
         std::cout << "[";
         BOOST_FOREACH(int i, good_indices) {
         const pcl::PointXYZ &query_point =query_point_cloud->points[i];
         std::cout << query_point.x << "," << query_point.y << "," << query_point.z << ";";
         }
         std::cout << "];";*/

        if (sample_consensus.computeModel())
        {
          inliers.clear();
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

      Eigen::VectorXf
      ransacy_graph(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& training_point_cloud,
                    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & query_point_cloud,
                    const std::vector<int>& good_indices, const object_recognition::maximum_clique::Graph &graph,
                    std::vector<int>& inliers)
      {
        SampleConsensusModelRegistrationGraph<pcl::PointXYZ>::Ptr model(
            new SampleConsensusModelRegistrationGraph<pcl::PointXYZ>(training_point_cloud, good_indices, graph));
        pcl::RandomSampleConsensus<pcl::PointXYZ> sample_consensus(model);
        Eigen::VectorXf coefficients;
        model->setInputTarget(query_point_cloud, good_indices);
        sample_consensus.setDistanceThreshold(sensor_error_);
        sample_consensus.setMaxIterations(n_ransac_iterations_);

        /*std::cout << "a=[";
         BOOST_FOREACH(int i, good_indices) {
         const pcl::PointXYZ &training_point =training_point_cloud->points[i];
         std::cout << training_point.x << "," << training_point.y << "," << training_point.z << ";";
         }
         std::cout << "];"  << std::endl << "b=";
         std::cout << "[";
         BOOST_FOREACH(int i, good_indices) {
         const pcl::PointXYZ &query_point =query_point_cloud->points[i];
         std::cout << query_point.x << "," << query_point.y << "," << query_point.z << ";";
         }
         std::cout << "];";*/

        if (sample_consensus.computeModel())
        {
          inliers.clear();
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
        const cv::Mat point_cloud = inputs.get<cv::Mat>("points3d");
        const std::map<ObjectOpenCVId, ObjectId> & id_correspondences = inputs.get<std::map<ObjectOpenCVId, ObjectId> >(
            "id_correspondences");
        const std::map<ObjectId, float> & spans = inputs.get<std::map<ObjectId, float> >("spans");

        const cv::Mat & initial_image = inputs.get<cv::Mat>("image");

        // Get the outputs
        std::vector<ObjectId> object_ids;
        std::vector<cv::Mat> Rs, Ts;

        if (point_cloud.empty())
        {
          // Only use 2d to 3d matching
          // TODO
          //const std::vector<cv::KeyPoint> &keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
        }
        else
        {
          // Cluster the matches per object ID
          typedef std::map<ObjectOpenCVId, pcl::PointCloud<pcl::PointXYZ> > OpenCVIdToPointCloud;
          OpenCVIdToPointCloud query_point_clouds;
          OpenCVIdToPointCloud training_point_clouds;
          std::map<ObjectOpenCVId, std::vector<unsigned int> > query_indices;
          std::vector<int> histo_count;
          std::vector<cv::KeyPoint> draw_keypoints_1;
          for (unsigned int query_index = 0; query_index < matches.size(); ++query_index)
          {
            // Figure out the 3d query point
            pcl::PointXYZ query_point;
            const cv::KeyPoint & keypoint = keypoints[query_index];
            const cv::Vec3f &point3f = point_cloud.at<cv::Vec3f>(keypoint.pt.y, keypoint.pt.x);
            query_point = pcl::PointXYZ(point3f.val[0], point3f.val[1], point3f.val[2]);
            // Make sure it does not contain any NaN's
            // We could have a solver that would consider Nan's as missing entries
            if ((query_point.x != query_point.x) || (query_point.y != query_point.y)
                || (query_point.z != query_point.z))
              continue;

            const std::vector<cv::DMatch> &local_matches = matches[query_index];
            const cv::Mat &local_matches_3d = matches_3d[query_index];
            if (!local_matches.empty())
              draw_keypoints_1.push_back(keypoint);
            // Get the matches for that point
            for (unsigned int match_index = 0; match_index < local_matches.size(); ++match_index)
            {
              const cv::Vec3f & val = local_matches_3d.at<cv::Vec3f>(0, match_index);
              pcl::PointXYZ training_point(val[0], val[1], val[2]);

              // Fill in the clouds
              ObjectOpenCVId opencv_object_id = local_matches[match_index].imgIdx;
              training_point_clouds[opencv_object_id].push_back(training_point);
              query_point_clouds[opencv_object_id].push_back(query_point);
              query_indices[opencv_object_id].push_back(query_index);
            }
          }

          if (debug_)
          {
            cv::Mat out_img;
            cv::drawKeypoints(initial_image, draw_keypoints_1, out_img);
            cv::namedWindow("keypoints from objects", 0);
            cv::imshow("keypoints from objects", out_img);
            cv::waitKey(1000);
          }

          // For each object, build the connectivity graph between the matches
          std::map<ObjectOpenCVId, std::vector<std::vector<int> > > matching_query_points;
          typedef std::map<ObjectOpenCVId, object_recognition::maximum_clique::Graph> OpenCVIdToGraph;
          OpenCVIdToGraph graphs;
          for (OpenCVIdToPointCloud::const_iterator query_iterator = query_point_clouds.begin();
              query_iterator != query_point_clouds.end(); ++query_iterator)
              {
            // Create a graph for that object
            ObjectOpenCVId opencv_object_id = query_iterator->first;
            std::cout << "Starting object: " << opencv_object_id << std::endl;
            object_recognition::maximum_clique::Graph graph(query_iterator->second.size());
            //graphs.insert(std::make_pair(opencv_object_id, graph));

            // Fill the connections for that graph
            ObjectId object_id = id_correspondences.find(opencv_object_id)->second;
            fill_connections(training_point_clouds[opencv_object_id], query_point_clouds[opencv_object_id],
                             query_indices[opencv_object_id], spans.find(object_id)->second, sensor_error_, graph);

            /*cluster_clouds(training_point_clouds[opencv_object_id],
             query_point_clouds[opencv_object_id], spans.find(object_id)->second,
             Rs, Ts);
             object_ids.resize(Rs.size(),object_id);
             continue;*/
            // Keep processing the graph until there is no maximum clique of the right size
            while (true)
            {
              // Compute the maximum of clique of that graph
              std::vector<unsigned int> maximum_clique;
              std::vector<int> int_maximum_clique(maximum_clique.size());
              std::vector<int> inliers;
              pcl::PointCloud<pcl::PointXYZ>::Ptr training_cloud_ptr =
                  training_point_clouds[opencv_object_id].makeShared();
              pcl::PointCloud<pcl::PointXYZ>::Ptr query_cloud_ptr = query_point_clouds[opencv_object_id].makeShared();
              Eigen::VectorXf coefficients;

              if (0)
              {
                graph.findMaximumClique(maximum_clique);
                std::cout << "done finding max clique, size: " << maximum_clique.size() << std::endl;

                if (maximum_clique.size() < min_inliers_)
                  break;

                //check_clique(training_point_clouds[opencv_object_id], query_point_clouds[opencv_object_id],
                //           query_indices[opencv_object_id], spans.find(object_id)->second, graph, maximum_clique);

                // Perform RANSAC on it to find the best possible pose
                std::copy(maximum_clique.begin(), maximum_clique.end(), int_maximum_clique.begin());
                std::cout << "*** starting RANSAC" << std::endl;
                coefficients = ransacy(training_cloud_ptr, query_cloud_ptr, int_maximum_clique, inliers);
              }
              else
              {
                for (unsigned int i = 0; i < training_point_clouds[opencv_object_id].points.size(); ++i)
                  int_maximum_clique.push_back(i);
                std::cout << "*** starting RANSAC" << std::endl;
                coefficients = ransacy_graph(training_cloud_ptr, query_cloud_ptr, int_maximum_clique, graph, inliers);
              }

              // If no pose was found, forget about all the connections in that clique
              std::cout << "*** n inliers: " << inliers.size() << " clique size " << int_maximum_clique.size()
                        << std::endl;

              if (inliers.size() < min_inliers_)
              {
                for (unsigned int i = 0; i < int_maximum_clique.size(); ++i)
                  for (unsigned int j = i + 1; j < int_maximum_clique.size(); ++j)
                    graph.deleteEdge(int_maximum_clique[i], int_maximum_clique[j]);
                break;
                continue;
              }

              // Store the matches for debug purpose
              if (debug_)
              {
                matching_query_points[opencv_object_id].push_back(inliers);

                std::vector<cv::KeyPoint> draw_keypoints;
                BOOST_FOREACH(int i, maximum_clique)
                      draw_keypoints.push_back(keypoints[query_indices[opencv_object_id][i]]);
                cv::Mat output_img;
                cv::drawKeypoints(initial_image, draw_keypoints, output_img);
                cv::namedWindow("max clique", 0);
                cv::imshow("max clique", output_img);
                cv::waitKey(1000);
              }

              // Check whether other matches could fit that model

              // Store the pose
              cv::Mat_<float> R_mat(3, 3), tvec(3, 1);
              for (unsigned int j = 0; j < 3; ++j)
              {
                for (unsigned int i = 0; i < 3; ++i)
                  R_mat(j, i) = coefficients[4 * j + i];
                tvec(j, 0) = coefficients[4 * j + 3];
              }
              Rs.push_back(R_mat);
              Ts.push_back(tvec);
              object_ids.push_back(object_id);
              std::cout << R_mat << std::endl;
              std::cout << tvec << std::endl;

              // Figure out the matches to remove
              std::vector<unsigned int> query_indices_to_delete;
              BOOST_FOREACH(unsigned int index, inliers)
                    query_indices_to_delete.push_back(query_indices[opencv_object_id][index]);

              std::sort(query_indices_to_delete.begin(), query_indices_to_delete.end());
              std::vector<unsigned int>::const_iterator iter = query_indices_to_delete.begin(), end =
                  query_indices_to_delete.end();
              int i = -1;
              BOOST_FOREACH(unsigned int query_index, query_indices[opencv_object_id])
                  {
                    ++i;
                    if (size_t(query_index) < *iter)
                      continue;
                    while ((iter != end) && (size_t(query_index) > *iter))
                      ++iter;
                    if (iter == end)
                      break;
                    // If the match has a keypoint in the inliers, remove the match
                    if (size_t(query_index) == *iter)
                      graph.deleteEdges(i);
                  }
            }
          }

          if (debug_)
          {
            // Draw the different inliers
            cv::Mat output_img = initial_image.clone();
            std::vector<cv::Scalar> colors;
            colors.push_back(cv::Scalar(255, 255, 0));
            colors.push_back(cv::Scalar(0, 255, 255));
            colors.push_back(cv::Scalar(255, 0, 255));
            colors.push_back(cv::Scalar(255, 0, 0));
            colors.push_back(cv::Scalar(0, 255, 0));
            colors.push_back(cv::Scalar(0, 0, 255));
            colors.push_back(cv::Scalar(0, 0, 0));
            colors.push_back(cv::Scalar(85, 85, 85));
            colors.push_back(cv::Scalar(170, 170, 170));
            colors.push_back(cv::Scalar(255, 255, 255));

            unsigned int i = 0;
            for (std::map<ObjectOpenCVId, std::vector<std::vector<int> > >::const_iterator query_iterator =
                matching_query_points.begin(); query_iterator != matching_query_points.end(); ++query_iterator)
            {
              ObjectOpenCVId object_opencv_id = query_iterator->first;

              BOOST_FOREACH(const std::vector<int> & indices, query_iterator->second)
                  {
                    std::vector<cv::KeyPoint> draw_keypoints;
                    draw_keypoints.clear();
                    BOOST_FOREACH(int index, indices)
                          draw_keypoints.push_back(keypoints[query_indices[object_opencv_id][index]]);
                    if (i < colors.size())
                    {
                      cv::drawKeypoints(output_img, draw_keypoints, output_img, colors[i]);
                      ++i;
                    }
                  }
            }
            cv::namedWindow("inliers", 0);
            cv::imshow("inliers", output_img);
          }

          outputs["Rs"] << Rs;
          outputs["Ts"] << Ts;
          outputs["object_ids"] << object_ids;
          std::cout << "********************* found " << object_ids.size() << "poses" << std::endl;
        }

        return 0;
      }
    private:
      /** How much can the sensor be wrong at most */
      float sensor_error_;
      /** flag indicating whether we run in edbug mode */
      bool debug_;
      /** The minimum number of inliers in order to do pose matching */
      unsigned int min_inliers_;
      /** The number of RANSAC iterations to perform */
      unsigned int n_ransac_iterations_;
    };
  }
}

ECTO_CELL(tod_detection, object_recognition::tod::GuessGenerator, "GuessGenerator",
          "Given descriptors and 3D positions, compute object guesses.");
