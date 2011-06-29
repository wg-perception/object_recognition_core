#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using ecto::tendrils;
//#include "opencv_candidate/PoseRT.h"
/* BOILER_PLATE_MODULE
 struct MyModule
 {
 static void declare_params(tendrils& params);
 static void declare_io(const tendrils& params, tendrils& in, tendrils& out);
 void configure(tendrils& params, tendrils& inputs, tendrils& outputs);
 int process(const tendrils& in, tendrils& out);
 void destroy();
 };
 */

namespace tod
{
  struct PlanarSegmentation
  {
    static void declare_params(tendrils& params)
    {
      params.declare<float> ("x_crop",
                             "The amount to keep in the x direction (meters) relative\n"
                               "to the coordinate frame defined by the pose.",
                             0.1);
      params.declare<float> ("y_crop",
                             "The amount to keep in the y direction (meters) relative to\n"
                               "the coordinate frame defined by the pose.", 0.1);
      params.declare<float> ("z_crop",
                             "The amount to keep in the z direction (meters) relative to\n"
                               "the coordinate frame defined by the pose.",
                             0.25);
      params.declare<float> ("z_min",
                             "The amount to crop above the plane, in meters.",
                             0.02);
    }

    static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat> ("depth", "The depth image to segment");
      //FIXME use a pose object here?
      in.declare<cv::Mat> ("R", "The pose rotation matrix.");
      in.declare<cv::Mat> ("T", "The pose translation vector.");
      in.declare<cv::Mat> ("K", "The camera matrix.");

      out.declare<cv::Mat> ("mask",
                            "The output mask, determined by the segmentation.\n"
                              "255 is the value for objects satisfying the constraints.\n"
                              "0 otherwise.");
    }
    void configure(tendrils& p, tendrils& inputs, tendrils& outputs)
    {
      z_crop = p.get<float> ("z_crop");
      x_crop = p.get<float> ("x_crop");
      y_crop = p.get<float> ("y_crop");
      z_min = p.get<float> ("z_min");

    }

    void computeNormal(const cv::Mat& R, const cv::Mat& T,
                       cv::Matx<double, 3, 1>& N, cv::Matx<double, 3, 1>& O)
    {

      cv::Vec3d z(0, 0, 1);
      //std::cout << cv::Mat(O) << std::endl;
      cv::Mat N_ = R * cv::Mat(z);
      cv::Mat O_ = T + N_ * z_min;
      O = O_;
      N = N_;
      //compute the offset from vector from the normal.
      //std::cout << "N = " << N << std::endl;
      //std::cout << "O = " << O << std::endl;

    }

    int process(const tendrils& in, tendrils& out)
    {
      cv::Mat R, T, K, depth;
      in.get<cv::Mat> ("R").convertTo(R, CV_64F);
      in.get<cv::Mat> ("T").convertTo(T, CV_64F);
      in.get<cv::Mat> ("K").convertTo(K, CV_64F);
      depth = in.get<cv::Mat> ("depth");
      cv::Mat& mask = out.get<cv::Mat> ("mask");

      if (!depth.empty())
        {
          mask.create(depth.size(), CV_8UC1);
          mask = cv::Scalar(0);
        }
      else
        return 0;
      if (R.empty() || T.empty() || K.empty())
        return 0;

      box_mask.create(depth.size());
      box_mask.setTo(cv::Scalar(0));

      std::vector<cv::Point3f> box(8);
      box[0] = cv::Point3f(x_crop, y_crop, 0);
      box[1] = cv::Point3f(-x_crop, y_crop, 0);
      box[2] = cv::Point3f(-x_crop, -y_crop, 0);
      box[3] = cv::Point3f(x_crop, -y_crop, 0);
      box[4] = cv::Point3f(x_crop, y_crop, z_crop);
      box[5] = cv::Point3f(-x_crop, y_crop, z_crop);
      box[6] = cv::Point3f(-x_crop, -y_crop, z_crop);
      box[7] = cv::Point3f(x_crop, -y_crop, z_crop);

      std::vector<cv::Point2f> projected, hull;
      cv::projectPoints(box, R, T, K, cv::Mat(4, 1, CV_64FC1, cv::Scalar(0)),
                        projected);

      cv::convexHull(projected, hull, true);
      std::vector<cv::Point> points(hull.size());
      std::copy(hull.begin(), hull.end(), points.begin());
      cv::fillConvexPoly(box_mask, points.data(), points.size(),
                         cv::Scalar::all(255));
      cv::Matx<double, 3, 3> A_x;

      A_x = K;
      A_x = A_x.inv();
      cv::Matx<double, 3, 1> N, O;
      computeNormal(R, T, N, O);
      cv::Matx<double, 1, 3> N_t = N.t();
      cv::Matx<double, 1, 3> N_t_A_x = N_t * A_x;
      cv::Matx<double, 1, 1> numerator_ = (O).t() * N;
      double numerator = numerator_(0);
      int width = mask.size().width;
      int height = mask.size().height;
      cv::Mat_<uint16_t>::iterator dit = depth.begin<uint16_t> ();
      cv::Mat_<uint8_t>::iterator it = mask.begin<uint8_t> ();
      cv::Mat_<uint8_t>::iterator mit = box_mask.begin();
      cv::Vec3d uv(0, 0, 1);
      for (int v = 0; v < height; v++)
        {
          uv[1] = v;
          for (int u = 0; u < width; u++, ++it, ++mit, ++dit)
            {
              if (*mit == 0)
                continue;
              uv[0] = u;
              cv::Matx<double, 1, 1> AtN = N_t_A_x * uv;
              double k = numerator / AtN(0);
              cv::Matx<double, 1, 1> X = k * (A_x.row(2) * uv);
              uint16_t depth = *dit;
              *it = 255 * uint8_t(
                                  (depth > (O(2) + z_crop)) && (depth
                                      < uint16_t(X(0) * 1000)));
            }
        }
      return 0;
    }
    float x_crop, y_crop, z_crop, z_min;
    cv::Mat_<uint8_t> box_mask;

  };
}

void wrap_CameraToWorld();
void wrap_TwoDToThreeD();
void wrap_BagReader();
void wrap_GuessGenerator();

BOOST_PYTHON_MODULE(tod)
{
  ecto::wrap<tod::PlanarSegmentation>("PlanarSegmentation", "Given a pose, "
    "assuming it describes the center of the object coordinate system and "
    "lies on a plane, segment the object from the plane");
  wrap_CameraToWorld();
  wrap_TwoDToThreeD();
  wrap_BagReader();
  wrap_GuessGenerator();
}
