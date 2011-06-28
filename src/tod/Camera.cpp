/*
 * Camera.cpp
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

#include <opencv_candidate/Camera.h>

#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/calib3d/calib3d.hpp>
namespace opencv_candidate
{
namespace
{

void readOpenCVCalibration(Camera& camera, const std::string& calibfile)
{
  cv::FileStorage fs(calibfile, cv::FileStorage::READ);
  CV_Assert(fs.isOpened())
    ;

  cv::read(fs["camera_matrix"], camera.K, cv::Mat());
  cv::read(fs["distortion_coefficients"], camera.D, cv::Mat());
  cv::read(fs["image_width"], camera.image_size.width, 0);
  cv::read(fs["image_height"], camera.image_size.height, 0);
  CV_Assert(camera.K.empty() == false);
}

}
const std::string Camera::YAML_NODE_NAME = "camera";

Camera::Camera()
{

}

Camera::~Camera()
{

}

Camera::Camera(const std::string& calibfile, CalibrationFormat format)
{
  switch (format)
  {
    case OPENCV_YAML:
      readOpenCVCalibration(*this, calibfile);
      Kinv = K.inv();
      break;
    case TOD_YAML:
      cv::FileStorage fs(calibfile, cv::FileStorage::READ);
      CV_Assert (fs.isOpened())
        ;
      read(fs[Camera::YAML_NODE_NAME]);
      break;
  }
}

Camera::Camera(const cv::FileNode& fn)
{
  read(fn);
}

void Camera::projectPoints(const std::vector<cv::Point3f>& rays, std::vector<cv::Point2f>& pts, bool use_pose) const
{
  cv::projectPoints(cv::Mat(rays), use_pose ? pose.rvec : cv::Mat_<float>::zeros(3,1), use_pose ? pose.tvec : cv::Mat_<float>::zeros(3,1), K, cv::Mat(4, 1, CV_64FC1, cv::Scalar(0)), pts);
}

//serialization
void Camera::write(cv::FileStorage& fs) const
{
  cvWriteComment(*fs, "Camera", 0);
  fs << "{";
  if (!K.empty())
    fs << "K" << K;
  if (!D.empty())
    fs << "D" << D;
  fs << "width" << image_size.width;
  fs << "height" << image_size.height;
  fs << PoseRT::YAML_NODE_NAME;
  pose.write(fs);
  fs << "}";
}
void Camera::read(const cv::FileNode& fn)
{
  fn["K"] >> K;
  fn["D"] >> D;
  if (!K.empty())
    Kinv = K.inv();
  image_size.width = (int)fn["width"];
  image_size.height = (int)fn["height"];
  pose.read(fn["pose"]);
}

}
