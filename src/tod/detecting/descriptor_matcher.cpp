/*
 * feature_matcher.cpp
 *
 *  Created on: Jul 7, 2011
 *      Author: vrabaud
 */

#include <string>

#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "objcog/db/db.h"

struct DescriptorMatcher
{
  static void declare_params(ecto::tendrils& p)
  {
    p.declare<std::string>("db", "A JSON string describing the db to use");
    p.declare<std::vector<std::string> >("object_ids", "The list of objects we should consider");
    // We can do radius and/or ratio test
    p.declare<float>("radius", "The radius for the NN search", 0);
    p.declare<float>("ratio", "The min ratio between a match and the next one to be valid", 0);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    inputs.declare<cv::Mat>("descriptors", "The descriptors to match to the database");
    inputs.declare<std::string>("params", "The parameters used to compute the descriptors, as JSON string");
    outputs.declare<std::vector<std::vector<cv::DMatch> > >("matches", "The matches for the input descriptors");
    outputs.declare<std::vector<std::vector<cv::Point3f> > >("matches_3d", "The 3d position of the matches");
  }

  void configure(ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    // get some parameters
    radius_ = params.get<float>("radius");
    ratio_ = params.get<float>("ratio");

    // TODO Create the matcher depending on the type of descriptors
    //matcher_

    // load the descriptors from the DB
    db_future::ObjectDb db(params.get<std::string>("db"));
    BOOST_FOREACH(const std::string & object_id, params.get<std::vector<std::string> >("object_ids")) {
      db_future::Query query;
      query.add_where("object_id", object_id);
      query.query(db);
    }

    features_3d_;
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

void wrap_DescriptorMatcher()
{
  ecto::wrap<DescriptorMatcher>("DescriptorMatcher", "Given descriptors, find matches, relating to objects.");
}
