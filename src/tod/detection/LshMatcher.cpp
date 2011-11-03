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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "opencv_candidate/lsh.hpp"

namespace object_recognition
{
  typedef std::vector<cv::DMatch> matches_t;
  namespace tod
  {
    struct LSHMatcher
    {

      static void
      declare_params(ecto::tendrils& p)
      {
        p.declare(&LSHMatcher::radius_,"radius", "", 55);
        p.declare(&LSHMatcher::key_size_,"key_size", "", 8);
        p.declare(&LSHMatcher::n_tables_,"n_tables", "", 4);
        p.declare(&LSHMatcher::multi_probe_level_,"multi_probe_level", "", 1);
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        inputs.declare<cv::Mat>("train", "Test descriptors.");
        inputs.declare<cv::Mat>("test", "Train descriptors.");
        outputs.declare<matches_t>("matches", "The descriptor matches.");
      }

      /** Get the 2d keypoints and figure out their 3D position from the depth map
       * @param inputs
       * @param outputs
       * @return
       */
      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        if (!matcher_)
        {
          // key_size: 24
          // multi_probe_level: 2
          // n_tables: 6
          // radius: 45
          // ratio: 0.8
          matcher_.reset(new lsh::LshMatcher(*n_tables_, *key_size_, *multi_probe_level_));
          cv::Mat train_desc;
          inputs["train"] >> train_desc;
          matcher_->add(std::vector<cv::Mat>(1, train_desc));
        }
        std::vector<std::vector<cv::DMatch> > matches;
        cv::Mat descriptors;
        inputs["test"] >> descriptors;

        // Perform radius search
        if (*radius_)
        {
          // Perform radius search
          matcher_->radiusMatch(descriptors, matches, *radius_);
        }
        std::vector<cv::DMatch> final_matches;
        final_matches.reserve(descriptors.rows);
        for (int match_index = 0; match_index < descriptors.rows; ++match_index)
        {
          if (!matches[match_index].empty())
          {
            final_matches.push_back(matches[match_index].front());
          }
        }
        outputs["matches"] << final_matches;
        return ecto::OK;
      }
    private:
      /** The object used to match descriptors to our DB of descriptors */
      boost::shared_ptr<lsh::LshMatcher> matcher_;
      /** The radius for the nearest neighbors (if not using ratio) */
      ecto::spore<unsigned> radius_, n_tables_, key_size_, multi_probe_level_;

    };
  }
}

ECTO_CELL(tod_detection, object_recognition::tod::LSHMatcher, "LSHMatcher", "Given descriptors, find matches.");
