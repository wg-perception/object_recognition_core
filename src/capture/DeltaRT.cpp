#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <string>
#include <numeric>

#include "object_recognition/capture/capture.hpp"
#include "object_recognition/db/couch.hpp"
#include "object_recognition/db/opencv.h"

using ecto::tendrils;
namespace object_recognition
{
  namespace capture
  {
    struct DeltaRT
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<double>("angle_thresh", "The angle thresh hold.", CV_PI / 36);
        params.declare<bool>("reset", "Reset observations.", false);
      }
      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<bool>("found", "Whether or not the R|T is good.").required(true);
        inputs.declare<cv::Mat>("R", "The orientation.").required(true);
        inputs.declare<cv::Mat>("T", "The translation.").required(true);
        outputs.declare<bool>("novel", "Whether or not the R|T is novel relative to previous novel R|T poses.", false);
      }
      void
      configure(const tendrils& params, const tendrils& inputs,const tendrils& outputs)
      {
        reset_ = params["reset"];
        angle_thresh_ = params["angle_thresh"];
        novel_ = outputs["novel"];
        R_ = inputs["R"];
        T_ = inputs["T"];
        found_ = inputs["found"];
      }
      int
      process(const tendrils& inputs,const tendrils& outputs)
      {
        *novel_ = false;
        if (*found_ == false)
          return ecto::OK;
        if (*reset_)
        {
          observations_.clear();
          *reset_ = false;
        }
        double min_delta = std::numeric_limits<double>::infinity();
        cv::Mat R = R_->clone();
        cv::Mat T = T_->clone();
        cv::Mat R_delta, r_delta;
        for (size_t i = 0; i < observations_.size(); i++)
        {
          cv::Mat& R_i = observations_[i].first;
          R_delta = R * R_i.t();
          cv::Rodrigues(R_delta, r_delta);
          double theta_delta = std::abs(cv::norm(r_delta));
          if (theta_delta < min_delta)
            min_delta = theta_delta;
        }
        if (min_delta > *angle_thresh_)
        {
          std::cout << "Novel" << std::endl;
          *novel_ = true;
          observations_.push_back(std::make_pair(R, T));
        }
        return ecto::OK;
      }
      std::vector<std::pair<cv::Mat, cv::Mat> > observations_;
      ecto::spore<cv::Mat> R_, T_;
      ecto::spore<bool> found_, novel_, reset_;
      ecto::spore<double> angle_thresh_;
    };
  }
}
ECTO_CELL(capture, object_recognition::capture::DeltaRT, "DeltaRT",
          "Uses the R|T of the camera to determine when a frame in novel.");
