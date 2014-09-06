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

#include <limits>

#include <opencv2/core/core.hpp>

namespace object_recognition_core
{
  namespace filters
  {
    struct DepthFilter
    {
      static void
      declare_params(ecto::tendrils& params)
      {
        params.declare<float>("d_min", "The minimal distance at which object become interesting (in meters)",
                              -std::numeric_limits<float>::max());
        params.declare<float>("d_max", "The maximal distance at which object become interesting (in meters)",
                              std::numeric_limits<float>::max());
      }

      void
      configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        d_min_ = params.get<float>("d_min");
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        inputs.declare<cv::Mat>("points3d", "The 3d points: width by height by 3 channels");
        outputs.declare<cv::Mat>("mask", "The mask of what is within the depth range in the image");
      }

      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        // Get the depth
        std::vector<cv::Mat> channels(3);
        cv::split(inputs.get<cv::Mat>("points3d"), channels);

        cv::Mat output = ((d_min_ < channels[2]) & (channels[2] < d_max_));

        outputs["mask"] << output;
        return ecto::OK;
      }
    private:
      float d_min_, d_max_;
    };
  }
}

ECTO_CELL(filters, object_recognition_core::filters::DepthFilter, "depth_filter",
          "Given a depth image, return the mask of what is between two depths.")
