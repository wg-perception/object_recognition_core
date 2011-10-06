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
      declare_params(ecto::tendrils& params)
      {
        params.declare(&GuessGenerator::min_inliers_, "min_inliers", "Minimum number of inliers", 15);
        params.declare(&GuessGenerator::n_ransac_iterations_, "n_ransac_iterations", "Number of RANSAC iterations.",
                       1000);
        params.declare(&GuessGenerator::sensor_error_, "sensor_error", "The error (in meters) from the Kinect", 0.01);
        params.declare(&GuessGenerator::do_display_, "do_display", "If true, display temporary info through highgui",
                       false);
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
        if (*do_display_)
        {
          colors_.push_back(cv::Scalar(255, 255, 0));
          colors_.push_back(cv::Scalar(0, 255, 255));
          colors_.push_back(cv::Scalar(255, 0, 255));
          colors_.push_back(cv::Scalar(255, 0, 0));
          colors_.push_back(cv::Scalar(0, 255, 0));
          colors_.push_back(cv::Scalar(0, 0, 255));
          colors_.push_back(cv::Scalar(0, 0, 0));
          colors_.push_back(cv::Scalar(85, 85, 85));
          colors_.push_back(cv::Scalar(170, 170, 170));
          colors_.push_back(cv::Scalar(255, 255, 255));
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
          ClusterPerObject(keypoints, point_cloud, matches, matches_3d, *do_display_, colors_, initial_image,
                           all_object_points);

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

            {
              std::vector<unsigned int> query_indices = object_points.query_indices();
              std::sort(query_indices.begin(), query_indices.end());
              std::vector<unsigned int>::iterator end = std::unique(query_indices.begin(), query_indices.end());
              query_indices.resize(end - query_indices.begin());

              std::cout << query_indices.size() << " keypoints in " << object_points.query_indices().size()
                        << " matches" << std::endl;
            }

            object_points.FillAdjacency(keypoints, spans.find(object_id)->second, *sensor_error_);

            // Keep processing the graph until there is no maximum clique of the right size
            std::vector<ObjectId> object_ids;
            std::vector<cv::Mat> Rs, Ts;

            while (true)
            {
              // Compute the maximum of clique of that graph
              std::vector<int> inliers;
              Eigen::VectorXf coefficients;

              coefficients = RansacAdjacency(object_points, *sensor_error_, *n_ransac_iterations_, inliers);

              // If no pose was found, forget about all the connections in that clique
              std::cout << "RANSAC done with " << inliers.size() << " inliers" << std::endl;

              if (inliers.size() < *min_inliers_)
              {
                break;
                continue;
              }

              // Store the matches for debug purpose
              if (*do_display_)
              {
                matching_query_points[opencv_object_id].push_back(inliers);

                // Check whether other matches could fit that model
                std::vector<std::vector<int> > neighbors;
                const cv::Mat_<uchar> & adjacency = object_points.physical_adjacency_;
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
                std::cout << "possible coherent ones: " << intersection.size() << std::endl;

                /*object_recognition::maximum_clique::Graph graph_new(intersection.size());
                 for (unsigned int j = 0; j < intersection.size(); ++j)
                 for (unsigned int i = j + 1; i < intersection.size(); ++i)
                 if (adjacency(intersection[j], intersection[i]))
                 graph_new.addEdge(j, i);
                 std::vector<unsigned int> vertices;
                 graph_new.findMaximumClique(vertices);

                 std::cout << " witin themselves: " << vertices.size() << std::endl;*/
              }

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

              Rs.push_back(R_mat);
              Ts.push_back(tvec);
              object_ids.push_back(object_id);
              std::cout << R_mat << std::endl;
              std::cout << tvec << std::endl;

              // Figure out the matches to remove
              std::vector<unsigned int> query_indices;
              BOOST_FOREACH(unsigned int inlier, inliers)
                    query_indices.push_back(object_points.query_indices(inlier));

              object_points.InvalidateQueryIndices(query_indices);
            }

            // Save all the poses;
            Rs_final.insert(Rs_final.end(), Rs.begin(), Rs.end());
            Ts_final.insert(Ts_final.end(), Ts.begin(), Ts.end());
            object_ids_final.insert(object_ids_final.end(), object_ids.begin(), object_ids.end());
          }

          if (*do_display_)
          {
            // Draw the different inliers
            cv::Mat output_img = initial_image.clone();

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
                    if (i < colors_.size())
                    {
                      cv::drawKeypoints(output_img, draw_keypoints, output_img, colors_[i]);
                      ++i;
                    }
                  }
            }
            cv::namedWindow("inliers", 0);
            cv::imshow("inliers", output_img);
          }

          outputs["Rs"] << Rs_final;
          outputs["Ts"] << Ts_final;
          outputs["object_ids"] << object_ids_final;
          std::cout << "********************* found " << object_ids_final.size() << " poses" << std::endl;
        }

        return 0;
      }
    private:
      /** List of very different colors, for debugging purposes */
      std::vector<cv::Scalar> colors_;
      /** flag indicating whether we run in debug mode */
      ecto::spore<bool> do_display_;
      /** The minimum number of inliers in order to do pose matching */
      ecto::spore<unsigned int> min_inliers_;
      /** The number of RANSAC iterations to perform */
      ecto::spore<unsigned int> n_ransac_iterations_;
      /** How much can the sensor be wrong at most */
      ecto::spore<float> sensor_error_;
    }
    ;
  }
}

ECTO_CELL(tod_detection, object_recognition::tod::GuessGenerator, "GuessGenerator",
          "Given descriptors and 3D positions, compute object guesses.");
