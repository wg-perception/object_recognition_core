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

#include <ecto/ecto.hpp>

#include <boost/format.hpp>

#include <opencv2/core/core.hpp>

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/common/pose_result.h>

using object_recognition_core::common::PoseResult;

namespace object_recognition_core
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
      declare_io(const ecto::tendrils& p, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        unsigned int ninput = p.get<unsigned int>("n_inputs");
        //inputs
        for (unsigned int i = 0; i < ninput; i++)
        {
          inputs.declare<std::vector<PoseResult> >(get_input_string("pose_results", i),
                                                   "The results of object recognition");
        }

        //output
        outputs.declare(&Aggregator::output_pose_results_, "pose_results", "The results of object recognition");
      }

      void
      configure(const ecto::tendrils& p, const ecto::tendrils& in, const ecto::tendrils& out)
      {
        for (unsigned int i = 0; i < in.size(); i++)
          input_pose_results_.push_back(in[get_input_string("pose_results", i)]);
      }

      int
      process(const ecto::tendrils& in, const ecto::tendrils& out)
      {
        // Figure out the number of inputs
        unsigned int n_objects = 0;
        for (unsigned int i = 0; i < input_pose_results_.size(); i++)
          n_objects += input_pose_results_[i]->size();

        output_pose_results_->resize(n_objects);
        std::vector<PoseResult>::iterator end = output_pose_results_->begin();
        for (unsigned int i = 0; i < input_pose_results_.size(); i++)
        {
          std::copy(input_pose_results_[i]->begin(), input_pose_results_[i]->end(), end);
          end += input_pose_results_[i]->size();
        }

        return ecto::OK;
      }

      std::vector<ecto::spore<std::vector<PoseResult> > > input_pose_results_;
      ecto::spore<std::vector<PoseResult> > output_pose_results_;
    };
  }
}

ECTO_CELL(voter, object_recognition_core::voters::Aggregator, "Aggregator",
          "Simply aggregates the results from several pipelines")
