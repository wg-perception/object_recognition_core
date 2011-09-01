#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ecto/registry.hpp>
using ecto::tendrils;
namespace object_recognition
{
  struct RescaledRegisteredDepth
  {
    static void
    declare_params(tendrils& params)
    {
    }

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat>("depth", "The depth image to rescale.");
      in.declare<cv::Mat>("image", "The rgb image.");
      out.declare<cv::Mat>("depth", "The rescaled depth image.");
    }
    void
    configure(const tendrils& p, const tendrils& inputs, const tendrils& outputs)
    {
      depth_in = inputs["depth"];
      image_in = inputs["image"];
      depth_out = outputs["depth"];
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      cv::Size dsize = depth_in->size(), isize = image_in->size();
      if (dsize == isize)
      {
        *depth_out = *depth_in;
        return ecto::OK;
      }

      float factor = float(isize.width) / dsize.width;
      cv::Mat output(isize, depth_in->type(), NAN);
      cv::Mat subregion(output.rowRange(0, dsize.height * factor));
      cv::resize(*depth_in, subregion, subregion.size());
      *depth_out = output;
      return ecto::OK;
    }
    ecto::spore<cv::Mat> image_in, depth_in, depth_out;
  };
}
ECTO_CELL(capture, object_recognition::RescaledRegisteredDepth, "RescaledRegisteredDepth",
          "Rescale the openni depth image to be the same size as the image if necessary.");
