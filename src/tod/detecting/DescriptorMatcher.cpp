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

#include <string>
#include <map>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <ecto/ecto.hpp>

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
        p.declare<std::string>("collection_models", "The collection where the models are stored.", "models");
        p.declare<std::string>("db_json_params", "A JSON string describing the db to use").required();
        p.declare<boost::python::object>("object_ids", "The list of objects ids we should consider.\n").required();
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
        outputs.declare<std::vector<std::vector<cv::DMatch> > >("matches", "The matches for the input descriptors");
        outputs.declare<std::vector<cv::Mat> >(
            "matches_3d",
            "For each point, the 3d position of the matches, 1 by n matrix with 3 channels for, x, y, and z.");
        outputs.declare<std::map<ObjectId, float> >("spans",
                                                    "For each found object, its span based on known features.");
        outputs.declare<std::map<ObjectOpenCVId, ObjectId> >(
            "id_correspondences", "Correspondences from OpenCV integer id to the JSON object ids");
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

        // Load the list of Object to study
        const boost::python::object & python_object_ids = params.get<boost::python::object>("object_ids");
        boost::python::stl_input_iterator<std::string> begin(python_object_ids), end;
        std::vector<ObjectId> object_ids;
        std::copy(begin, end, std::back_inserter(object_ids));

        // load the descriptors from the DB
        db_future::ObjectDb db(params.get<std::string>("db_json_params"));
        collection_models_ = params.get<std::string>("collection_models");
        features_3d_.reserve(object_ids.size());
        std::vector<cv::Mat> all_descriptors;

        unsigned int object_opencv_id = 0;
        BOOST_FOREACH(const ObjectId & object_id, object_ids)
            {
              db_future::DocumentView query;
              query.set_db(db);
              query.set_collection(collection_models_);
              query.AddView("CouchDB", db_future::couch::WhereDocId(object_id));
              std::cout << "object_id: " << object_id << std::endl;
              // TODO be robust to missing entries
              for (db_future::DocumentView view = query.begin(), view_end = db_future::DocumentView::end();
                  view != view_end; ++view)
                  {
                db_future::Document doc = *view;
                cv::Mat descriptors;
                doc.get_attachment<cv::Mat>(db, "descriptors", descriptors);
                all_descriptors.push_back(descriptors);

                // Store the id conversion
                id_correspondences_.insert(std::pair<ObjectOpenCVId, ObjectId>(object_opencv_id, object_id));
                ++object_opencv_id;

                // Store the 3d positions
                cv::Mat points3d;
                doc.get_attachment<cv::Mat>(db, "points", points3d);
                if (points3d.rows != 1)
                  points3d = points3d.t();
                features_3d_.push_back(points3d);

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
                      max_span_sq = std::max(
                          vec.val[0] * vec.val[0] + vec.val[1] * vec.val[1] + vec.val[2] * vec.val[2], max_span_sq);
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
                std::cout << "span" << spans_[object_id] << std::endl;
              }
            }
        matcher_->add(all_descriptors);
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
                local_matches_3d.at<cv::Vec3f>(0, i) = features_3d_[match.imgIdx].at<cv::Vec3f>(0, match.trainIdx);
                ++i;
              }
        }

        outputs["matches_3d"] << matches_3d;
        outputs["spans"] << spans_;
        outputs["id_correspondences"] << id_correspondences_;

        return 0;
      }
    private:
      /** The collection where the models are stored */
      std::string collection_models_;
      /** The object used to match descriptors to our DB of descriptors */
      cv::Ptr<cv::DescriptorMatcher> matcher_;
      /** The radius for the nearest neighbors (if not using ratio) */
      unsigned int radius_;
      /** The ratio used for k-nearest neighbors, if not using radius search */
      unsigned int ratio_;
      /** The 3d position of the loaded descriptors (the first index is on the object ID) */
      std::vector<cv::Mat> features_3d_;
      /** For each object id, the maximum distance between the known descriptors (span) */
      std::map<ObjectId, float> spans_;
      /** Matching between an OpenCV integer ID and the ids found in the JSON */
      std::map<ObjectOpenCVId, ObjectId> id_correspondences_;
    };
  }
}

ECTO_CELL(tod_detection, object_recognition::tod::DescriptorMatcher, "DescriptorMatcher",
          "Given descriptors, find matches, relating to objects.");
