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

#include <vector>

#include <boost/dynamic_bitset.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>

#include "opencv_candidate/lsh.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <Eigen/StdVector>

#include "adjacency_ransac.h"

using ecto::tendrils;

namespace
{
  struct IndexMatch
  {
    IndexMatch(unsigned int descriptor_index, unsigned int training_index)
        :
          query_index_(descriptor_index),
          training_index_(training_index)
    {
    }
    unsigned int query_index_;
    unsigned int training_index_;
  };
}

namespace object_recognition
{
  namespace tod
  {
    /** Given 3d points and descriptors, find correspondences for SBA
     */
    struct PrepareForG2O
    {
    public:

      static void
      declare_params(ecto::tendrils& p)
      {
        std::stringstream ss;
        ss << "JSON string that can contain the following fields: \"radius\" (for epsilon nearest neighbor search), "
           << "\"ratio\" when applying the ratio criterion like in SIFT";
        p.declare<std::string>("search_json_params", ss.str()).required(true);
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<std::vector<cv::Mat> >("points", "The measured points.").required(true);
        inputs.declare<std::vector<cv::Mat> >("points3d", "The 3d points in the world frame.").required(true);
        inputs.declare<std::vector<cv::Mat> >("descriptors", "The stacked descriptors.").required(true);

        outputs.declare<Eigen::SparseMatrix<int> >("x", "The measured x of the points.");
        outputs.declare<Eigen::SparseMatrix<int> >("y", "The measured y of the points.");
        outputs.declare<Eigen::SparseMatrix<int> >("disparity", "The disparity of the points.");
        outputs.declare<std::vector<Eigen::Vector3d> >("points", "The 3d points.");
        outputs.declare<std::vector<std::vector<size_t> > >(
            "ids", "The SBA id of each input point, the first vector is per image.");
      }

      void
      configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        boost::property_tree::ptree search_param_tree;
        std::stringstream ssparams;
        ssparams << params.get<std::string>("search_json_params");
        boost::property_tree::read_json(ssparams, search_param_tree);

        radius_ = search_param_tree.get<float>("radius");
        matcher_ = new lsh::LshMatcher(search_param_tree.get<unsigned int>("n_tables"),
                                       search_param_tree.get<unsigned int>("key_size"),
                                       search_param_tree.get<unsigned int>("multi_probe_level"));

        in_points_ = inputs["points"];
        in_points3d_ = inputs["points3d"];
        descriptors_ = inputs["descriptors"];

        x_ = outputs["x"];
        y_ = outputs["y"];
        disparity_ = outputs["disparity"];
        out_points_ = outputs["points"];
        ids_ = outputs["ids"];
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        std::vector<cv::Mat> descriptors_all;
        inputs["descriptors"] >> descriptors_all;
        size_t n_images = descriptors_all.size();

        matcher_->clear();
        matcher_->add(descriptors_all);
        matcher_->train();

        // Initialize the ids of each point, and whether they have been matched to others or not
        std::vector<boost::dynamic_bitset<> > is_matched_all(n_images);
        *ids_ = std::vector<std::vector<size_t> >(n_images);
        unsigned int n_points_all = 0;
        for (size_t image_id = 0; image_id < n_images; ++image_id)
        {
          const cv::Mat & descriptors = descriptors_all[image_id];
          (*ids_)[image_id] = std::vector<size_t>(descriptors.rows, 0);
          is_matched_all[image_id] = boost::dynamic_bitset<>(descriptors.rows);
          n_points_all += descriptors.rows;
        }
        // Figure out the physical matches of the input points
        size_t id = 0;
        typedef std::pair<size_t, size_t> ImageIdIndex;
        std::vector<std::vector<ImageIdIndex> > id_to_index_set(n_points_all);
        for (size_t query_image_id = 0; query_image_id < n_images; ++query_image_id)
        {
          // Add some verbosity as this step is slow
          /*if (image_id == 0)
           std::cout << "Prepare G2O: processing image ";
           else
           for (unsigned int i = 0; i < 2 * std::log10(n_images) + 1; ++i)
           std::cout << "\b";
           std::cout << std::setfill(' ') << std::setw(std::log10(n_images) + 1) << (image_id + 1) << "/" << n_images;
           std::flush(std::cout);*/

          // Find the nearest neighbors of all the points in the current image
          const cv::Mat & descriptors = descriptors_all[query_image_id];
          const cv::Mat_<cv::Vec3f> & points3d = (*in_points3d_)[query_image_id];
          std::vector<std::vector<cv::DMatch> > matches_all;

          matcher_->knnMatch(descriptors, matches_all, 2);

          // Cluster the matches per view
          typedef std::vector<IndexMatch> MatchesPerView;
          std::map<unsigned int, MatchesPerView> matches_per_view;
          size_t n_points = matches_all.size();

          for (size_t descriptor_id = 0; descriptor_id < n_points; ++descriptor_id)
          {
            BOOST_FOREACH(const cv::DMatch & match, matches_all[descriptor_id])
                {
                  if ((match.imgIdx == int(query_image_id)) && (match.trainIdx != int(descriptor_id)))
                    continue;
                  if (match.distance < radius_)
                    matches_per_view[match.imgIdx].push_back(IndexMatch(descriptor_id, match.trainIdx));
                }
          }

          // Find the best 5 views (no need to perform RANSAC on all the views)
#if 1
          {
            std::vector<std::pair<unsigned int, unsigned int> > lengths;
            lengths.reserve(n_images);
            std::map<unsigned int, std::set<unsigned int> > unique_ids;
            for (std::map<unsigned int, MatchesPerView>::const_iterator iter = matches_per_view.begin(), end =
                matches_per_view.end(); iter != end; ++iter)
              lengths.push_back(std::make_pair(iter->second.size(), iter->first));
            std::sort(lengths.begin(), lengths.end());
            for (std::vector<std::pair<unsigned int, unsigned int> >::const_iterator iter = std::max(lengths.begin(),
                                                                                                     lengths.end() - 5),
                end = lengths.end(); iter != end; ++iter)
            {
              unsigned int training_image_id = iter->second;
              MatchesPerView matches = matches_per_view[training_image_id];

              // Perform RANSAC for each of those matched views
              AdjacencyRansac adjacency_ransac;

              BOOST_FOREACH(const IndexMatch & index_match, matches_per_view[training_image_id ])
                  {
                    const cv::Vec3f & point_query = points3d.at<cv::Vec3f>(0, index_match.query_index_),
                        &point_training = points3d.at<cv::Vec3f>(0, index_match.training_index_);
                    pcl::PointXYZ query_point = pcl::PointXYZ(point_query.val[0], point_query.val[1],
                                                              point_query.val[2]);
                    pcl::PointXYZ training_point = pcl::PointXYZ(point_training.val[0], point_training.val[1],
                                                                 point_training.val[2]);
                    adjacency_ransac.AddPoints(training_point, query_point, index_match.query_index_);
                  }

              double span = 1;
              double sensor_error = 0.5;
              std::vector<cv::KeyPoint> keypoints(n_points);
              for (unsigned int i = 0; i < n_points; ++i)
              {
                const cv::Vec3f & vec = (*in_points_)[query_image_id].at<cv::Vec3f>(0, i);
                keypoints[i].pt = cv::Point2f(vec[0], vec[1]);
              }
              adjacency_ransac.FillAdjacency(keypoints, span, sensor_error);
              unsigned int n_ransac_iterations = 1000;
              std::vector<int> inliers;
              adjacency_ransac.Ransac(sensor_error, n_ransac_iterations, inliers);

              // Use the inliers to cluster the points
              BOOST_FOREACH(unsigned int inlier, inliers)
                  {
                    is_matched_all[training_image_id].set(matches[inlier].training_index_);
                    (*ids_)[training_image_id][matches[inlier].training_index_] = id;
                    id_to_index_set[id].push_back(ImageIdIndex(training_image_id, matches[inlier].training_index_));
                  }
              ++id;
            }
          }
#else
          for (size_t descriptor_id = 0; descriptor_id < n_points; ++descriptor_id)
          {
            (*ids_)[query_image_id][descriptor_id] = id;
            id_to_index_set[id].push_back(ImageIdIndex(query_image_id, descriptor_id));
            ++id;
          }
#endif
        }

        // Fill the point data
        size_t n_id = id - 1;
        Eigen::DynamicSparseMatrix<int> x(n_images, n_id);
        Eigen::DynamicSparseMatrix<int> y(n_images, n_id);
        Eigen::DynamicSparseMatrix<int> disparity(n_images, n_id);
        *out_points_ = std::vector<Eigen::Vector3d>(n_id);
        for (size_t id = 0; id < n_id; ++id)
        {
          Eigen::Vector3d average(0, 0, 0);
          BOOST_FOREACH(const ImageIdIndex & image_id_index, id_to_index_set[id])
              {
                size_t image_id = image_id_index.first;
                size_t index = image_id_index.second;
                {
                  const cv::Vec3f & vec = (*in_points_)[image_id].at<cv::Vec3f>(0, index);
                  x.insert(image_id, id) = vec.val[0];
                  y.insert(image_id, id) = vec.val[1];
                  disparity.insert(image_id, id) = vec.val[2];
                }
                {
                  const cv::Vec3f & vec = (*in_points3d_)[image_id].at<cv::Vec3f>(0, index);
                  average += Eigen::Vector3d(vec.val[0], vec.val[1], vec.val[2]);
                }
              }
          (*out_points_)[id] = average / id_to_index_set[id].size();
        }

        *x_ = Eigen::SparseMatrix<int>(x);
        *y_ = Eigen::SparseMatrix<int>(y);
        *disparity_ = Eigen::SparseMatrix<int>(disparity);

        return ecto::OK;
      }
    private:
      ecto::spore<std::vector<cv::Mat> > in_points_;
      ecto::spore<std::vector<cv::Mat> > in_points3d_;
      ecto::spore<std::vector<cv::Mat> > descriptors_;
      /** The object used to match descriptors to our DB of descriptors */
      cv::Ptr<cv::DescriptorMatcher> matcher_;
      /** The radius for the nearest neighbors (if not using ratio) */
      unsigned int radius_;

      ecto::spore<Eigen::SparseMatrix<int> > x_;
      ecto::spore<Eigen::SparseMatrix<int> > y_;
      ecto::spore<Eigen::SparseMatrix<int> > disparity_;
      ecto::spore<std::vector<Eigen::Vector3d> > out_points_;
      ecto::spore<std::vector<std::vector<size_t> > > ids_;
    }
    ;
  }
}

ECTO_CELL(tod_training, object_recognition::tod::PrepareForG2O, "PrepareForG2O",
          "Given 3d points and descriptors, find correspondences for SBA")
