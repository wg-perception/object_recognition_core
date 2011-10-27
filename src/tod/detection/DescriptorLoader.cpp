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
    struct DescriptorLoader
    {
      static void
      declare_params(ecto::tendrils& p)
      {
        p.declare<std::string>("collection", "The collection where the models are stored.").required(true);
        p.declare<db::ObjectDbParameters>("db_params", "The DB parameters").required(true);
        p.declare<boost::python::object>("model_ids", "The list of model ids we should consider.\n").required();
        p.declare<boost::python::object>("object_ids", "The list of model ids we should consider.\n").required(true);
        // We can do radius and/or ratio test
        std::stringstream ss;
        ss << "JSON string that can contain the following fields: \"radius\" (for epsilon nearest neighbor search), "
           << "\"ratio\" when applying the ratio criterion like in SIFT";
        p.declare<std::string>("feature_descriptor_params", ss.str()).required(true);
        p.declare(&DescriptorLoader::do_update_, "do_update", "Update the matcher from the database, expensive.",
                  false);
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        outputs.declare<std::vector<cv::Mat> >("descriptors", "The descriptors to match to the database");
        outputs.declare<std::vector<cv::Mat> >("features3d", "The 3d position of the descriptors");
        outputs.declare<std::map<ObjectId, float> >("spans",
                                                    "For each found object, its span based on known features.");
        outputs.declare<std::map<ObjectOpenCVId, ObjectId> >(
            "id_correspondences", "Correspondences from OpenCV integer id to the JSON object ids");
        outputs.declare<bool>("do_update", "If true, that means new descriptors have been loaded");
      }

      void
      configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        // Load the list of Object to study
        {
          const boost::python::object & python_model_ids = params.get<boost::python::object>("model_ids");
          boost::python::stl_input_iterator<std::string> begin(python_model_ids), end;
          std::copy(begin, end, std::back_inserter(model_ids_));
        }

        // Load the list of Object to study
        {
          const boost::python::object & python_object_ids = params.get<boost::python::object>("object_ids");
          boost::python::stl_input_iterator<std::string> begin(python_object_ids), end;
          std::copy(begin, end, std::back_inserter(object_ids_));
        }

        if ((model_ids_.size() != object_ids_.size()) || (model_ids_.empty()))
        {
          std::stringstream ss;
          ss << object_ids_.size() << " object ids given and " << model_ids_.size() << " model ids given." << std::endl;
          throw std::runtime_error(ss.str());
        }

        // load the descriptors from the DB
        db_params_ = params["db_params"];
        collection_ = params["collection"];

        descriptors_ = outputs["descriptors"];
        features_3d_ = outputs["features3d"];
        do_update_out_ = outputs["do_update"];
        spans_ = outputs["spans"];
        id_correspondences_ = outputs["id_correspondences"];
        do_update_.set_callback(boost::bind(&DescriptorLoader::on_do_update, this, _1));
        *do_update_ = true;
        do_update_.dirty(true);
        do_update_.notify();
      }
      void
      on_do_update(bool on_do_update)
      {
        if (!on_do_update)
          return;

        std::cout << "Loading models. This may take some time..." << std::endl;

        *do_update_out_ = true;
        db::ObjectDb db(*db_params_);
        unsigned int object_opencv_id = 0;
        std::vector<ModelId>::const_iterator model_id = model_ids_.begin(), model_id_end = model_ids_.end();
        std::vector<ObjectId>::const_iterator object_id = object_ids_.begin();
        descriptors_->reserve(model_ids_.size());
        for (; model_id != model_id_end; ++model_id, ++object_id)
        {
          std::cout << "Loading model for object id: " << *object_id << std::endl;
          db::Document doc(db, *collection_, *model_id);
          cv::Mat descriptors;
          doc.get_attachment<cv::Mat>("descriptors", descriptors);
          descriptors_->push_back(descriptors);

          // Store the id conversion
          id_correspondences_->insert(std::pair<ObjectOpenCVId, ObjectId>(object_opencv_id, *object_id));
          ++object_opencv_id;

          // Store the 3d positions
          cv::Mat points3d;
          doc.get_attachment<cv::Mat>("points", points3d);
          if (points3d.rows != 1)
            points3d = points3d.t();
          features_3d_->push_back(points3d);

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
          (*spans_)[*object_id] = std::sqrt(max_span_sq);
          std::cout << "span: " << (*spans_)[*object_id] << " meters" << std::endl;
        }
      }

      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        *do_update_out_ = *do_update_;
        *do_update_ = false;
        return ecto::OK;
      }

      /** The collection where the models are stored */
      ecto::spore<std::string> collection_;
      /** The objects ids to use */
      std::vector<ObjectId> object_ids_;
      /** The matching model ids to use */
      std::vector<ModelId> model_ids_;
      /** the DB JSON parameters */
      ecto::spore<db::ObjectDbParameters> db_params_;

      /** The loaded descriptors */
      ecto::spore<std::vector<cv::Mat> > descriptors_;
      /** The 3d position of the edscriptors */
      ecto::spore<std::vector<cv::Mat> > features_3d_;
      /** If True, that means we got new data */
      ecto::spore<bool> do_update_out_;
      /** If True, load from the db */
      ecto::spore<bool> do_update_;
      /** For each object id, the maximum distance between the known descriptors (span) */
      ecto::spore<std::map<ObjectId, float> > spans_;
      /** Matching between an OpenCV integer ID and the ids found in the JSON */
      ecto::spore<std::map<ObjectOpenCVId, ObjectId> > id_correspondences_;
    };
  }
}

ECTO_CELL(tod_detection, object_recognition::tod::DescriptorLoader, "DescriptorLoader",
          "Loads descriptors from the DB.");
