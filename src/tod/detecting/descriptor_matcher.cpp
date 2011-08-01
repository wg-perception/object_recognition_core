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

#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "object_recognition/db/db.h"

namespace object_recognition
{
  namespace tod
  {
    struct DescriptorMatcher
    {
      static void
      declare_params(ecto::tendrils& p)
      {
        p.declare<std::string>("collection_objects", "The collection where the objects are stored.");
        p.declare<std::string>("db_json_params", "A JSON string describing the db to use");
        p.declare<boost::python::object>("object_ids", "The list of objects ids we should consider.\n");
        // We can do radius and/or ratio test
        std::stringstream ss;
        ss << "JSON string that can contain the following fields: \"radius\" (for epsilon nearest neighbor search), "
           << "\"ratio\" when applying the ratio criterion like in SIFT";
        p.declare<std::string>("search_json_params", ss.str());
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        inputs.declare<cv::Mat>("descriptors", "The descriptors to match to the database");
        outputs.declare<std::vector<std::vector<cv::DMatch> > >("matches", "The matches for the input descriptors");
        outputs.declare<std::vector<std::vector<cv::Point3f> > >("matches_3d", "The 3d position of the matches");
      }

      void
      configure(ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        // get some parameters
        {
          boost::property_tree::ptree search_param_tree;
          std::stringstream ssparams;
          ssparams << params.get<std::string>("search_json_params");
          boost::property_tree::read_json(ssparams, search_param_tree);

          radius_ = search_param_tree.get<float>("radius");
          ratio_ = search_param_tree.get<float>("ratio");
        }

        // Load the list of Object to study
        const boost::python::object & python_object_ids = params.get<boost::python::object>("object_ids");
        boost::python::stl_input_iterator<std::string> begin(python_object_ids), end;
        std::vector<std::string> object_ids;
        std::copy(begin, end, std::back_inserter(object_ids));

        // load the descriptors from the DB
        db_future::ObjectDb db(params.get<std::string>("db_json_params"));
        /*BOOST_FOREACH(const std::string & object_id, object_ids)
            {
              db_future::View query;
              query.AddWhere("object_id", object_id);
              query.set_db(db);
              query.set_collection(params.get<std::string>("collection_objects"));
              for (db_future::DocumentIterator query_iterator = query.begin(), query_iterator_end =
                  db_future::DocumentIterator::end(); query_iterator != query_iterator_end; ++query_iterator)
              {

              }
            }*/

        // TODO Create the matcher depending on the type of descriptors
        //matcher_

      }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
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
    std::vector<std::vector<cv::Point3f> > &matches_3d = outputs.get<std::vector<std::vector<cv::Point3f> > >(
        "matches_3d");
    matches_3d.clear();
    matches_3d.resize(descriptors.cols);
    for (int match_index = 0; match_index < descriptors.cols; ++match_index)
    {
      std::vector<cv::Point3f> & local_matches_3d = matches_3d[match_index];
      BOOST_FOREACH(const cv::DMatch & match, matches[match_index])
            local_matches_3d.push_back(features_3d_[match.imgIdx][match.trainIdx]);
    }

    return 0;
  }
private:
  /** The object used to match descriptors to our DB of descriptors */
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  /** The radius for the nearest neighbors (if not using ratio) */
  unsigned int radius_;
  /** The ratio used for k-nearest neighbors, if not using radius search */
  unsigned int ratio_;
  /** The 3d position of the loaded descriptors (the first index is on the object ID) */
  std::vector<std::vector<cv::Point3f> > features_3d_;
};
}
}

ECTO_CELL(tod, object_recognition::tod::DescriptorMatcher, "DescriptorMatcher",
          "Given descriptors, find matches, relating to objects.");
