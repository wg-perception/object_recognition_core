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

#include "object_recognition/common/types.h"
#include "impl/guess_generator.h"
#include "impl/maximum_clique.h"
#include "impl/sac_model_registration_graph.h"

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
        sensor_error_ = 0.01;
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
        if (point_cloud.empty())
        {
          // Only use 2d to 3d matching
          // TODO
          //const std::vector<cv::KeyPoint> &keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
        }
        else
        {
          // Cluster the matches per object ID
          OpenCVIdToObjectPoints all_object_points;
          ClusterPerObject(keypoints, point_cloud, matches, matches_3d, debug_, initial_image, all_object_points);

          // For each object, build the connectivity graph between the matches
          std::vector<ObjectId> object_ids_final;
          std::vector<cv::Mat> Rs_final, Ts_final;
          std::map<ObjectOpenCVId, std::vector<std::vector<int> > > matching_query_points;
          typedef std::map<ObjectOpenCVId, object_recognition::maximum_clique::Graph> OpenCVIdToGraph;
          OpenCVIdToGraph graphs;
          for (OpenCVIdToObjectPoints::iterator query_iterator = all_object_points.begin();
              query_iterator != all_object_points.end(); ++query_iterator)
              {
            // Create a graph for that object
            ObjectPoints & object_points = query_iterator->second;
            ObjectOpenCVId opencv_object_id = query_iterator->first;
            ObjectId object_id = id_correspondences.find(opencv_object_id)->second;

            std::cout << "***Starting object: " << opencv_object_id << std::endl;

            object_points.FillAdjacency(keypoints, spans.find(object_id)->second, sensor_error_);

            // Keep processing the graph until there is no maximum clique of the right size
            std::vector<ObjectId> object_ids;
            std::vector<cv::Mat> Rs, Ts;

            while (true)
            {
              // Compute the maximum of clique of that graph
              std::vector<int> inliers;
              Eigen::VectorXf coefficients;

              std::cout << "* starting RANSAC" << std::endl;
              coefficients = RansacAdjacency(object_points, sensor_error_, n_ransac_iterations_, inliers);

              // If no pose was found, forget about all the connections in that clique
              std::cout << "* n inliers: " << inliers.size() << std::endl;

              if (inliers.size() < min_inliers_)
              {
                break;
                continue;
              }

              // Store the matches for debug purpose
              if (debug_)
              {
                matching_query_points[opencv_object_id].push_back(inliers);

                /*
                 std::vector<unsigned int> maximum_clique;
                 std::vector<cv::KeyPoint> draw_keypoints;
                 BOOST_FOREACH(int i, maximum_clique)
                 draw_keypoints.push_back(keypoints[query_indices[opencv_object_id][i]]);
                 cv::Mat output_img;
                 cv::drawKeypoints(initial_image, draw_keypoints, output_img);
                 cv::namedWindow("max clique", 0);
                 cv::imshow("max clique", output_img);
                 }

                 // Check whether other matches could fit that model
                 std::vector<std::vector<int> > neighbors;
                 const cv::Mat_<uchar> & adjacency = graph.adjacency();
                 {
                 neighbors.resize(adjacency.rows);
                 for (int j = 0; j < adjacency.rows; ++j)
                 {
                 const uchar * row = adjacency.ptr<uchar>(j);
                 for (int i = 0; i < adjacency.cols; ++i)
                 if (row[i])
                 neighbors[j].push_back(i);
                 }
                 }

                 std::vector<int> intersection = neighbors[inliers[0]];
                 std::sort(inliers.begin(), inliers.end());
                 std::vector<int>::iterator intersection_end = std::set_difference(neighbors[inliers[0]].begin(),
                 neighbors[inliers[0]].end(),
                 inliers.begin(), inliers.end(),
                 intersection.begin());
                 intersection.resize(intersection_end - intersection.begin());
                 BOOST_FOREACH(int inlier, inliers)
                 {
                 intersection_end = std::set_intersection(intersection.begin(), intersection.end(),
                 neighbors[inlier].begin(), neighbors[inlier].end(),
                 intersection.begin());
                 intersection.resize(intersection_end - intersection.begin());
                 }

                 // Check why those are not good
                 std::cout << "possible coherent ones: " << intersection.size();

                 object_recognition::maximum_clique::Graph graph_new(intersection.size());
                 for (unsigned int j = 0; j < intersection.size(); ++j)
                 for (unsigned int i = j + 1; i < intersection.size(); ++i)
                 if (adjacency(intersection[j], intersection[i]))
                 graph_new.addEdge(j, i);
                 std::vector<unsigned int> vertices;
                 graph_new.findMaximumClique(vertices);

                 std::cout << " witin themselves: " << vertices.size() << std::endl;*/
                {
                  double thresh = sensor_error_ * sensor_error_;

                  // Check if the model is valid given the user constraints
                  Eigen::Matrix4f transform;
                  transform.row(0) = coefficients.segment<4>(0);
                  transform.row(1) = coefficients.segment<4>(4);
                  transform.row(2) = coefficients.segment<4>(8);
                  transform.row(3) = coefficients.segment<4>(12);

                  std::vector<int> valid_indices = object_points.valid_indices();
                  std::vector<int>::iterator valid_end = std::set_difference(valid_indices.begin(), valid_indices.end(),
                                                                             inliers.begin(), inliers.end(),
                                                                             valid_indices.begin());
                  valid_indices.resize(valid_end - valid_indices.begin());

                  unsigned int count = 0;
                  BOOST_FOREACH(int index, valid_indices)
                      {
                        const Eigen::Map<Eigen::Vector4f, Eigen::Aligned> &pt_src =
                            object_points.query_points(index).getVector4fMap();
                        const Eigen::Map<Eigen::Vector4f, Eigen::Aligned> &pt_tgt =
                            object_points.training_points(index).getVector4fMap();
                        Eigen::Vector4f p_tr = transform * pt_src;
                        // Calculate the distance from the transformed point to its correspondence
                        if ((p_tr - pt_tgt).squaredNorm() < 2 * thresh)
                        {
                          inliers.push_back(index);
                          ++count;
                        }
                      }
                  std::cout << " added : " << count << std::endl;
                }

                // Get the span of the inliers
                float max_dist_training = 0, max_dist_training_xy = 0, max_dist_query = 0, max_dist_query_xy = 0;
                for (unsigned int i = 0; i < inliers.size(); ++i)
                {
                  const pcl::PointXYZ & training_point_1 = object_points.training_points(inliers[i]), &query_point_1 =
                      object_points.query_points(inliers[i]);
                  for (unsigned int j = i + 1; j < inliers.size(); ++j)
                  {
                    const pcl::PointXYZ & training_point_2 = object_points.training_points(inliers[j]), &query_point_2 =
                        object_points.query_points(inliers[j]);
                    float dist_training = pcl::euclideanDistance(training_point_1, training_point_2);
                    float dist_query = pcl::euclideanDistance(query_point_1, query_point_2);
                    max_dist_training = std::max(max_dist_training, dist_training);
                    max_dist_query = std::max(max_dist_query, dist_query);
                    max_dist_training_xy = std::max(
                        max_dist_training_xy,
                        std::sqrt(
                            (training_point_1.x - training_point_2.x) * (training_point_1.x - training_point_2.x)
                            + (training_point_1.y - training_point_2.y) * (training_point_1.y - training_point_2.y)));
                    max_dist_query_xy = std::max(
                        max_dist_query_xy,
                        std::sqrt(
                            (query_point_1.x - query_point_2.x) * (query_point_1.x - query_point_2.x)
                            + (query_point_1.y - query_point_2.y) * (query_point_1.y - query_point_2.y)));
                  }
                }
                std::cout << "span of training inliers: " << max_dist_training << " for xy: " << max_dist_training_xy
                          << std::endl;
                std::cout << "span of query inliers: " << max_dist_query << " for xy: " << max_dist_query_xy
                          << std::endl;

                // Go over all the matches that have not been checked
                // Store the pose
                cv::Mat_<float> R_mat(3, 3), tvec(3, 1);
                for (unsigned int j = 0; j < 3; ++j)
                {
                  for (unsigned int i = 0; i < 3; ++i)
                    R_mat(j, i) = coefficients[4 * j + i];
                  tvec(j, 0) = coefficients[4 * j + 3];
                }
                R_mat = R_mat.t();
                tvec = -R_mat * tvec;

                // Check whether the pose could be close to a previous

                Rs.push_back(R_mat);
                Ts.push_back(tvec);
                object_ids.push_back(object_id);
                std::cout << R_mat << std::endl;
                std::cout << tvec << std::endl;

                if (0)
                {
                  for (std::vector<int>::const_iterator iter = inliers.begin(), end = inliers.end() - 1; iter < end;
                      ++iter)
                      {
                    std::cout << int(object_points.physical_adjacency_(*iter, *(iter + 1))) << " ";
                    if (object_points.physical_adjacency_(*iter, *(iter + 1)))
                      continue;
                    int i = *iter, j = *(iter + 1);

                    const pcl::PointXYZ & training_point_1 = object_points.training_points()->points[i],
                        &query_point_1 = object_points.query_points()->points[i], &training_point_2 =
                            object_points.training_points()->points[j], &query_point_2 =
                            object_points.query_points()->points[j];

                    // Two matches with the same query point cannot be connected
                    std::cout << "-" << int(object_points.query_indices()[i] == object_points.query_indices()[j])
                              << " ";

                    float dist_query = pcl::euclideanDistance(query_point_1, query_point_2);

                    std::cout << int(dist_query > (spans.find(object_id)->second + 2 * sensor_error_)) << " ";

                    float dist_training = pcl::euclideanDistance(training_point_1, training_point_2);
                    // Make sure the distance between two points is somewhat conserved
                    std::cout << int(std::abs(dist_training - dist_query) > 2 * sensor_error_) << "-- ";
                  }
                }
                std::cout << std::endl;

                // Figure out the matches to remove
                {
                  std::vector<unsigned int> query_indices;
                  BOOST_FOREACH(unsigned int inlier, inliers)
                        query_indices.push_back(object_points.query_indices()[inlier]);

                  object_points.InvalidateQueryIndices(query_indices);
                  std::cout << query_indices.size() << " edges deleted" << std::endl;
                }
              }

              // Save all the poses;
              Rs_final.insert(Rs_final.end(), Rs.begin(), Rs.end());
              Ts_final.insert(Ts_final.end(), Ts.begin(), Ts.end());
              object_ids_final.insert(object_ids_final.end(), object_ids.begin(), object_ids.end());
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
                BOOST_FOREACH(const std::vector<int> & indices, query_iterator->second)
                    {
                      std::vector<cv::KeyPoint> draw_keypoints;
                      draw_keypoints.clear();
                      BOOST_FOREACH(int index, indices)
                            draw_keypoints.push_back(
                                keypoints[all_object_points[query_iterator->first].query_indices(index)]);
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
          }

          outputs["Rs"] << Rs_final;
          outputs["Ts"] << Ts_final;
          outputs["object_ids"] << object_ids_final;
          std::cout << "********************* found " << object_ids_final.size() << "poses" << std::endl;
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
    }
    ;
  }
}

ECTO_CELL(tod_detection, object_recognition::tod::GuessGenerator, "GuessGenerator",
          "Given descriptors and 3D positions, compute object guesses.");
