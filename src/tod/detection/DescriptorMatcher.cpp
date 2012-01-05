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
//#include <opencv2/flann/flann.hpp>

#include "object_recognition/common/json_spirit/json_spirit.h"
#include "object_recognition/common/types.h"
#include "object_recognition/db/db.h"
#include "object_recognition/db/ModelReader.h"
#include "object_recognition/db/opencv.h"
#include "opencv_candidate/lsh.hpp"

using object_recognition::db::Documents;
using object_recognition::db::ObjectId;

namespace object_recognition
{
  namespace tod
  {
    struct DescriptorMatcher: public db::bases::ModelReaderImpl
    {
      void
      ParameterCallback(const Documents & db_documents)
      {
        descriptors_db_.resize(db_documents.size());
        features3d_db_.resize(db_documents.size());
        object_ids_.resize(db_documents.size());

        // Re-load the data from the DB
        std::cout << "Loading models. This may take some time..." << std::endl;
        unsigned int index = 0;
        BOOST_FOREACH(const object_recognition::db::Document & document, db_documents)
            {
              ObjectId object_id = document.get_value<std::string>("object_id");
              std::cout << "Loading model for object id: " << object_id << std::endl;
              cv::Mat descriptors;
              document.get_attachment<cv::Mat>("descriptors", descriptors);
              descriptors_db_[index] = descriptors;

              // Store the id conversion
              object_ids_[index] = object_id;

              // Store the 3d positions
              cv::Mat points3d;
              document.get_attachment<cv::Mat>("points", points3d);
              if (points3d.rows != 1)
                points3d = points3d.t();
              features3d_db_[index] = points3d;

              // Compute the span of the object
              float max_span_sq = 0;
              cv::MatConstIterator_<cv::Vec3f> i = points3d.begin<cv::Vec3f>(), end = points3d.end<cv::Vec3f>(), j;
              if (0)
              {
                // Too slow
                for (; i != end; ++i)
                {
                  for (j = i + 1; j != end; ++j)
                  {
                    cv::Vec3f vec = *i - *j;
                    max_span_sq = std::max(vec.val[0] * vec.val[0] + vec.val[1] * vec.val[1] + vec.val[2] * vec.val[2],
                                           max_span_sq);
                  }
                }
              }
              else
              {
                float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::min(), min_y =
                    std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::min(), min_z =
                    std::numeric_limits<float>::max(), max_z = std::numeric_limits<float>::min();
                for (; i != end; ++i)
                {
                  min_x = std::min(min_x, (*i).val[0]);
                  max_x = std::max(max_x, (*i).val[0]);
                  min_y = std::min(min_y, (*i).val[1]);
                  max_y = std::max(max_y, (*i).val[1]);
                  min_z = std::min(min_z, (*i).val[2]);
                  max_z = std::max(max_z, (*i).val[2]);
                }
                max_span_sq = (max_x - min_x) * (max_x - min_x) + (max_y - min_y) * (max_y - min_y)
                              + (max_z - min_z) * (max_z - min_z);
              }
              spans_[object_id] = std::sqrt(max_span_sq);
              std::cout << "span: " << spans_[object_id] << " meters" << std::endl;
              ++index;
            }

        // Clear the matcher and re-train it
        matcher_->clear();
        matcher_->add(descriptors_db_);
      }

      static void
      declare_params(ecto::tendrils& p)
      {
        // We can do radius and/or ratio test
        std::stringstream ss;
        ss << "JSON string that can contain the following fields: \"radius\" (for epsilon nearest neighbor search), "
           << "\"ratio\" when applying the ratio criterion like in SIFT";
        p.declare<std::string>("search_json_params", ss.str()).required(true);
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        inputs.declare<cv::Mat>("descriptors", "The descriptors to match to the database");

        outputs.declare<std::vector<std::vector<cv::DMatch> > >("matches", "The matches for the input descriptors");
        outputs.declare<std::vector<cv::Mat> >(
            "matches_3d",
            "For each point, the 3d position of the matches, 1 by n matrix with 3 channels for, x, y, and z.");
        outputs.declare<std::vector<ObjectId> >("object_ids", "The ids of the objects");
        outputs.declare<std::map<ObjectId, float> >("spans", "The ids of the objects");
      }

      void
      configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        // get some parameters
        {
          or_json::mObject search_param_tree;
          std::stringstream ssparams;
          ssparams << params.get<std::string>("search_json_params");

          {
            or_json::mValue value;
            or_json::read(ssparams, value);
            search_param_tree = value.get_obj();
          }

          radius_ = search_param_tree["radius"].get_real();
          ratio_ = search_param_tree["ratio"].get_real();

          // Create the matcher depending on the type of descriptors
          std::string search_type = search_param_tree["type"].get_str();
          if (search_type == "LSH")
          {
            /*cv::flann::LshIndexParams lsh_params(search_param_tree.get<unsigned int>("n_tables"),
             search_param_tree.get<unsigned int>("key_size"),
             search_param_tree.get<unsigned int>("multi_probe_level"));
             matcher_ = new cv::FlannBasedMatcher(&lsh_params);*/
            matcher_ = new lsh::LshMatcher(search_param_tree["n_tables"].get_uint64(),
                                           search_param_tree["key_size"].get_uint64(),
                                           search_param_tree["multi_probe_level"].get_uint64());
          }
          else
          {
            std::cerr << "Search not implemented for that type" << search_type;
            throw;
          }
        }
      }

      /** Get the 2d keypoints and figure out their 3D position from the depth map
       * @param inputs
       * @param outputs
       * @return
       */
      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        std::vector<std::vector<cv::DMatch> > matches;
        const cv::Mat & descriptors = inputs.get<cv::Mat>("descriptors");

        // Perform radius search
        if (radius_)
        {
          if (matcher_->getTrainDescriptors().empty())
          {
            std::cerr << "No descriptors loaded" << std::endl;
            return ecto::OK;
          }
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
                local_matches_3d.at<cv::Vec3f>(0, i) = features3d_db_[match.imgIdx].at<cv::Vec3f>(0, match.trainIdx);
                ++i;
              }
        }

        outputs["matches"] << matches;
        outputs["matches_3d"] << matches_3d;
        outputs["object_ids"] << object_ids_;
        outputs["spans"] << spans_;

        return ecto::OK;
      }
    private:
      /** The object used to match descriptors to our DB of descriptors */
      cv::Ptr<cv::DescriptorMatcher> matcher_;
      /** The radius for the nearest neighbors (if not using ratio) */
      unsigned int radius_;
      /** The ratio used for k-nearest neighbors, if not using radius search */
      unsigned int ratio_;
      /** The descriptors loaded from the DB */
      std::vector<cv::Mat> descriptors_db_;
      /** The 3d position of the descriptors loaded from the DB */
      std::vector<cv::Mat> features3d_db_;
      /** For each object id, the maximum distance between the known descriptors (span) */
      std::map<ObjectId, float> spans_;

      /** The matching object ids */
      std::vector<ObjectId> object_ids_;
    };

  }
}

ECTO_CELL(tod_detection, object_recognition::db::bases::ModelReaderBase<object_recognition::tod::DescriptorMatcher>,
          "DescriptorMatcher", "Given descriptors, find matches, relating to objects.");
