/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <boost/format.hpp>

#include <opencv2/core/core.hpp>

#include <ecto/ecto.hpp>

#include <object_recognition/common/types.h>

namespace object_recognition
{
  namespace voters
  {
    struct Aggregator
    {
      static std::string
      get_input_string(const std::string & type, unsigned int i)
      {
        return type + boost::str(boost::format("%i") % (i + 1));
      }

      static void
      declare_params(ecto::tendrils& p)
      {
        p.declare<unsigned int>("n_inputs", "Number of inputs to AND together").required(true);
      }

      static void
      declare_io(const ecto::tendrils& p, ecto::tendrils& in, ecto::tendrils& out)
      {
        unsigned int ninput = p.get<unsigned int>("n_inputs");
        //inputs
        for (unsigned int i = 0; i < ninput; i++)
        {
          in.declare<std::vector<cv::Mat> >(get_input_string("Rs", i), "The rotation matrices of the object poses");
          in.declare<std::vector<cv::Mat> >(get_input_string("Ts", i), "The translation matrices of the object poses");
          in.declare<std::vector<db::ObjectId> >(get_input_string("object_ids", i),
                                                 "The object ids of the found objects");
        }

        //output
        out.declare(&Aggregator::output_Rs_, "Rs", "The rotation matrices of the object poses");
        out.declare(&Aggregator::output_Ts_, "Ts", "The translation matrices of the object poses");
        out.declare(&Aggregator::output_object_ids_, "object_ids", "The object ids of the found objects");
      }

      void
      configure(const ecto::tendrils& p, const ecto::tendrils& in, const ecto::tendrils& out)
      {
        for (unsigned int i = 0; i < in.size(); i++)
        {
          input_Rs_.push_back(in[get_input_string("Rs", i)]);
          input_Ts_.push_back(in[get_input_string("Ts", i)]);
          input_object_ids_.push_back(in[get_input_string("object_ids", i)]);
        }
      }

      int
      process(const ecto::tendrils& in, const ecto::tendrils& out)
      {
        // Figure out the number of inputs
        unsigned int n_objects = 0;
        for (unsigned int i = 0; i < input_Rs_.size(); i++)
          n_objects += input_Rs_[i]->size();

        output_Rs_->resize(n_objects);
        output_Ts_->resize(n_objects);
        output_object_ids_->resize(n_objects);

        for (unsigned int i = 0; i < input_Rs_.size(); i++)
        {
          std::copy(input_Rs_[i]->begin(), input_Rs_[i]->end(), output_Rs_->begin());
          std::copy(input_Ts_[i]->begin(), input_Ts_[i]->end(), output_Ts_->begin());
          std::copy(input_object_ids_[i]->begin(), input_object_ids_[i]->end(), output_object_ids_->begin());
        }
        return ecto::OK;
      }

      std::vector<ecto::spore<std::vector<cv::Mat> > > input_Rs_;
      std::vector<ecto::spore<std::vector<cv::Mat> > > input_Ts_;
      std::vector<ecto::spore<std::vector<db::ObjectId> > > input_object_ids_;
      ecto::spore<std::vector<cv::Mat> > output_Rs_;
      ecto::spore<std::vector<cv::Mat> > output_Ts_;
      ecto::spore<std::vector<db::ObjectId> > output_object_ids_;
    };
  }
}

ECTO_CELL(voters, object_recognition::voters::Aggregator, "Aggregator",
          "Simply aggregates the results from several pipelines");
