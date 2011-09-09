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

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>

using ecto::tendrils;

namespace object_recognition
{
  namespace tod
  {
    /** Class inserting the TOD models in the db
     */
    struct TodModelStacker
    {
    public:
      static void
      declare_params(tendrils& params)
      {
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<cv::Mat>("points", "The 3d position of the points.");
        inputs.declare<cv::Mat>("descriptors", "The descriptors.");
        outputs.declare<std::vector<cv::Mat> >("points", "The stacked 3d position of the points.");
        outputs.declare<std::vector<cv::Mat> >("descriptors", "The stacked descriptors.");
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        cv::Mat points;
        inputs["points"] >> points;
        if (!points.empty())
        {
          points_.push_back(points);
          descriptors_.push_back(inputs.get<cv::Mat>("descriptors"));
        }
        outputs.get<std::vector<cv::Mat> >("points") = points_;
        outputs.get<std::vector<cv::Mat> >("descriptors") = descriptors_;
        return ecto::OK;
      }
    private:
      std::vector<cv::Mat> points_;
      std::vector<cv::Mat> descriptors_;
    };
  }
}

ECTO_CELL(tod_training, object_recognition::tod::TodModelStacker, "TodModelStacker", "Stack 3d points and descriptors")
