#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <string>
#include <numeric>

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
        params.declare<unsigned>("n_desired", "The number of desired views", std::numeric_limits<unsigned>::max());
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
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        reset_ = params["reset"];
        angle_thresh_ = params["angle_thresh"];
        novel_ = outputs["novel"];
        R_ = inputs["R"];
        T_ = inputs["T"];
        found_ = inputs["found"];
        n_desired_ = params["n_desired"];
      }
      int
      process(const tendrils& inputs, const tendrils& outputs)
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
        if (min_delta > *angle_thresh_ / 2)
        {
          *novel_ = true;
          observations_.push_back(std::make_pair(R, T));
          if (observations_.size() > *n_desired_)
          {
            std::cout << "Satisfied, total observations: " << *n_desired_ << std::endl;
            return ecto::QUIT;
          }
        }
        return ecto::OK;
      }
      std::vector<std::pair<cv::Mat, cv::Mat> > observations_;
      ecto::spore<cv::Mat> R_, T_;
      ecto::spore<bool> found_, novel_, reset_;
      ecto::spore<double> angle_thresh_;
      ecto::spore<unsigned> n_desired_;
    };
  }
}
ECTO_CELL(capture, object_recognition::capture::DeltaRT, "DeltaRT",
          "Uses the R|T of the camera to determine when a frame in novel.");
