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

#include <boost/property_tree/json_parser.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>

#include "opencv_candidate/lsh.hpp"

using ecto::tendrils;

namespace object_recognition
{
  namespace tod
  {
    /** cell storing the 3d points and descriptors while a model is being computed
     */
    struct FilteredTodModelStacker
    {
    public:

      static void
      declare_params(ecto::tendrils& p)
      {
        std::stringstream ss;
        ss << "JSON string that can contain the following fields: \"radius\" (for epsilon nearest neighbor search), "
           << "\"ratio\" when applying the ratio criterion like in SIFT";
        p.declare<std::string>("search_json_params", ss.str()).required();
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<cv::Mat>("points", "The 3d position of the points.").required(true);
        inputs.declare<cv::Mat>("descriptors", "The descriptors.").required(true);
        outputs.declare<std::vector<cv::Mat> >("points", "The stacked 3d position of the points.");
        outputs.declare<std::vector<cv::Mat> >("descriptors", "The stacked descriptors.");
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
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        cv::Mat points;
        inputs["points"] >> points;
        if (!points.empty())
        {
          cv::Mat_<cv::Vec3f> points_filtered(1, points.cols);
          cv::Mat descriptors;
          inputs["descriptors"] >> descriptors;

          // Pre-Filter the descriptors
          std::vector<std::vector<cv::DMatch> > matches;
          matcher_->radiusMatch(descriptors, matches, radius_);

          cv::Mat descriptors_filtered(descriptors.rows, descriptors.cols, descriptors.type());

          cv::Mat_<cv::Vec3f>::iterator iter = points.begin<cv::Vec3f>(), end = points.end<cv::Vec3f>();
          cv::Mat_<cv::Vec3f>::iterator iter_filtered = points_filtered.begin();
          unsigned int row = 0, row_filtered = 0;
          for (unsigned int i = 0; iter != end; ++iter, ++row, ++i)
          {
            const cv::Vec3f & point = *iter;
            if ((point.val[0] == point.val[0]) && (point.val[1] == point.val[1]) && (point.val[2] == point.val[2]))
            {
              // Check descriptors for LSH closeness
              if ((!matches.empty()) && (!matches[i].empty()))
                continue;

              *(iter_filtered++) = point;
              cv::Mat row_filtered_mat = descriptors_filtered.row(row_filtered++);
              descriptors.row(row).copyTo(row_filtered_mat);
            }
          }

          cv::Mat final_points, final_descriptors;
          points_filtered.colRange(0, row_filtered).copyTo(final_points);
          descriptors_filtered.rowRange(0, row_filtered).copyTo(final_descriptors);
          points_.push_back(final_points);
          descriptors_.push_back(final_descriptors);

          std::vector<cv::Mat> filtered_descriptors_vec(1, final_descriptors);
          matcher_->add(filtered_descriptors_vec);
          matcher_->train();
        }

        outputs.get<std::vector<cv::Mat> >("points") = points_;
        outputs.get<std::vector<cv::Mat> >("descriptors") = descriptors_;

        return ecto::OK;
      }
    private:
      std::vector<cv::Mat> points_;
      std::vector<cv::Mat> descriptors_;
      /** The object used to match descriptors to our DB of descriptors */
      cv::Ptr<cv::DescriptorMatcher> matcher_;
      /** The radius for the nearest neighbors (if not using ratio) */
      unsigned int radius_;
    };
  }
}

ECTO_CELL(tod_training, object_recognition::tod::FilteredTodModelStacker, "FilteredTodModelStacker",
          "Stack 3d points and descriptors but by cleaning them")
