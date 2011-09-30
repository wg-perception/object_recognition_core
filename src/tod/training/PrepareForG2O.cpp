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

using ecto::tendrils;

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
        std::cout << "G2O: " << n_images << " images " << in_points_->size() << " " << in_points3d_->size()
                  << std::endl;

        matcher_->clear();
        matcher_->add(descriptors_all);
        matcher_->train();

        // Initialize the ids of each point, ad whether they have been matched to others or not
        std::vector<boost::dynamic_bitset<> > is_done_all(n_images);
        *ids_ = std::vector<std::vector<size_t> >(n_images);
        unsigned int n_points_all = 0;
        for (size_t image_id = 0; image_id < n_images; ++image_id)
        {
          const cv::Mat & descriptors = descriptors_all[image_id];
          (*ids_)[image_id] = std::vector<size_t>(descriptors.rows);
          is_done_all[image_id] = boost::dynamic_bitset<>(descriptors.rows);
          n_points_all += descriptors.rows;
          std::cout << descriptors.rows << " " << (*in_points_)[image_id].cols << " " << (*in_points3d_)[image_id].cols
                    << std::endl;
        }
        // Figure out the physical matches of the input points
        size_t id = 0;
        typedef std::pair<size_t, size_t> ImageIdIndex;
        std::vector<std::vector<ImageIdIndex> > id_to_index_set(n_points_all);
        for (size_t image_id = 0; image_id < n_images; ++image_id)
        {
          // Find the nearest neighbors of all the points in the current image
          const cv::Mat & descriptors = descriptors_all[image_id];
          const cv::Mat_<cv::Vec3f> & points3d = (*in_points3d_)[image_id];

          std::vector<std::vector<cv::DMatch> > matches_all;
          matcher_->radiusMatch(descriptors, matches_all, radius_);
          size_t n_points = matches_all.size();
          std::cout << "G2O: matched " << n_points << " points with radius " << radius_ << std::endl;

          boost::dynamic_bitset<> & is_done = is_done_all[image_id];

          for (size_t descriptor_id = 0; descriptor_id < n_points; ++descriptor_id)
          {
            // If we've matched the point before, no need to do anything
            if (is_done[descriptor_id])
              continue;
            const cv::Vec3f & point1 = points3d.at<cv::Vec3f>(0, descriptor_id);
            BOOST_FOREACH(const cv::DMatch & match, matches_all[descriptor_id])
                {
                  if (match.imgIdx == int(image_id))
                  {
                    if (match.trainIdx != int(descriptor_id))
                      continue;
                  }
                  else
                  {
                    // make sure the matches are also close to each other physically
                    const cv::Vec3f & point2 = (*in_points3d_)[match.imgIdx].at<cv::Vec3f>(0, match.trainIdx);
                    cv::Vec3f diff = point1 - point2;
                    float norm = diff.val[0] * diff.val[0] + diff.val[1] * diff.val[1] + diff.val[2] * diff.val[2];
                    if (norm > 0.01 * 0.01)
                      continue;
                    continue;
                  }

                  is_done_all[match.imgIdx].set(match.trainIdx);
                  (*ids_)[match.imgIdx][match.trainIdx] = id;
                  id_to_index_set[id].push_back(ImageIdIndex(match.imgIdx, match.trainIdx));
                }
            ++id;
          }
        }

        // Fill the point data
        unsigned n_id = id - 1;
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
                  const cv::Vec3i & vec = (*in_points_)[image_id].at<cv::Vec3i>(0, index);
                  x.coeffRef(image_id_index.first, id) = vec.val[0];
                  y.coeffRef(image_id_index.first, id) = vec.val[1];
                  disparity.coeffRef(image_id_index.first, id) = vec.val[2];
                }
                {
                  const cv::Vec3f & vec = (*in_points_)[image_id].at<cv::Vec3f>(0, index);
                  average += Eigen::Vector3d(vec.val[0], vec.val[1], vec.val[2]);
                }
              }
          (*out_points_)[id] = average / id_to_index_set[id].size();
        }

        *x_ = Eigen::SparseMatrix<int>(x);
        *y_ = Eigen::SparseMatrix<int>(y);
        *disparity_ = Eigen::SparseMatrix<int>(disparity);
        outputs.get<std::vector<Eigen::Vector3d> >("points") = *out_points_;

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
