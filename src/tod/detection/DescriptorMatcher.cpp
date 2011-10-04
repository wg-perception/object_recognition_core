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
#include <ecto/ecto.hpp>
#include <string>
#include <map>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/flann/flann.hpp>

#include "object_recognition/common/types.h"
#include "object_recognition/db/db.h"
#include "object_recognition/db/opencv.h"
#include "opencv_candidate/lsh.hpp"
namespace object_recognition
{
  namespace tod
  {
    struct DescriptorMatcher
    {
      static void
      declare_params(ecto::tendrils& p)
      {
        // We can do radius and/or ratio test
        std::stringstream ss;
        ss << "JSON string that can contain the following fields: \"radius\" (for epsilon nearest neighbor search), "
           << "\"ratio\" when applying the ratio criterion like in SIFT";
        p.declare<std::string>("search_json_params", ss.str()).required();
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        inputs.declare<cv::Mat>("descriptors", "The descriptors to match to the database");
        inputs.declare<std::vector<cv::Mat> >("descriptors_db", "The descriptors from the database");
        inputs.declare<std::vector<cv::Mat> >("features3d_db", "The 3d position of the descriptors from the database");
        inputs.declare<bool>("do_update", "If true, forces the reload of the search structure");

        outputs.declare<std::vector<std::vector<cv::DMatch> > >("matches", "The matches for the input descriptors");
        outputs.declare<std::vector<cv::Mat> >(
            "matches_3d",
            "For each point, the 3d position of the matches, 1 by n matrix with 3 channels for, x, y, and z.");
      }

      void
      configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        // get some parameters
        {
          boost::property_tree::ptree search_param_tree;
          std::stringstream ssparams;
          ssparams << params.get<std::string>("search_json_params");
          boost::property_tree::read_json(ssparams, search_param_tree);

          radius_ = search_param_tree.get<float>("radius");
          ratio_ = search_param_tree.get<float>("ratio");

          // Create the matcher depending on the type of descriptors
          std::string search_type = search_param_tree.get<std::string>("type", "none");
          if (search_type == "LSH")
          {
            /*cv::flann::LshIndexParams lsh_params(search_param_tree.get<unsigned int>("n_tables"),
             search_param_tree.get<unsigned int>("key_size"),
             search_param_tree.get<unsigned int>("multi_probe_level"));
             matcher_ = new cv::FlannBasedMatcher(&lsh_params);*/
            matcher_ = new lsh::LshMatcher(search_param_tree.get<unsigned int>("n_tables"),
                                           search_param_tree.get<unsigned int>("key_size"),
                                           search_param_tree.get<unsigned int>("multi_probe_level"));
          }
          else
          {
            std::cerr << "Search not implemented for that type" << search_type;
            throw;
          }
        }

        do_update_ = inputs["do_update"];
        descriptors_db_ = inputs["descriptors_db"];
        features3d_db_ = inputs["features3d_db"];
      }

      /** Get the 2d keypoints and figure out their 3D position from the depth map
       * @param inputs
       * @param outputs
       * @return
       */
      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        std::vector<std::vector<cv::DMatch> > &matches = outputs.get<std::vector<std::vector<cv::DMatch> > >("matches");
        const cv::Mat & descriptors = inputs.get<cv::Mat>("descriptors");

        // Reload the search structure if necessary
        if (*do_update_)
        {
          matcher_->clear();
          matcher_->add(*descriptors_db_);
        }

        // Perform radius search
        if (radius_)
        {
          // Perform radius search
          matcher_->radiusMatch(descriptors, matches, radius_);
        }

        // TODO Perform ratio testing if necessary
        if (ratio_)
        {

        }

        // TODO remove matches that match the same (common descriptors)

        // Build the 3D positions of the matches
        std::vector<cv::Mat> matches_3d(descriptors.rows);

        for (int match_index = 0; match_index < descriptors.rows; ++match_index)
        {
          cv::Mat & local_matches_3d = matches_3d[match_index];
          local_matches_3d = cv::Mat(1, matches[match_index].size(), CV_32FC3);
          unsigned int i = 0;
          BOOST_FOREACH(const cv::DMatch & match, matches[match_index])
              {
                local_matches_3d.at<cv::Vec3f>(0, i) = (*features3d_db_)[match.imgIdx].at<cv::Vec3f>(0, match.trainIdx);
                ++i;
              }
        }

        outputs["matches_3d"] << matches_3d;

        return 0;
      }
    private:
      /** The object used to match descriptors to our DB of descriptors */
      cv::Ptr<cv::DescriptorMatcher> matcher_;
      /** The radius for the nearest neighbors (if not using ratio) */
      unsigned int radius_;
      /** The ratio used for k-nearest neighbors, if not using radius search */
      unsigned int ratio_;
      /** If true, forces the reload of the search structure */
      ecto::spore<bool> do_update_;
      /** The descriptors loaded from the DB */
      ecto::spore<std::vector<cv::Mat> > descriptors_db_;
      /** The 3d position of the descriptors loaded from the DB */
      ecto::spore<std::vector<cv::Mat> > features3d_db_;
    };
  }
}

ECTO_CELL(tod_detection, object_recognition::tod::DescriptorMatcher, "DescriptorMatcher",
          "Given descriptors, find matches, relating to objects.");
