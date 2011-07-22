/*
 * pose.cpp
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

#include <opencv_candidate/PoseRT.h>
#include <opencv2/core/core_c.h>
#include <opencv2/calib3d/calib3d.hpp>
namespace opencv_candidate
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const std::string Pose::YAML_NODE_NAME = "pose";

/** access the rotation matrix
 * @return the rotation matrix
 */
const Eigen::Matrix3f & Pose::r() const
{
  return r_;
}

/** access the rotation matrix
 * @return the rotation matrix
 */
template<>
  cv::Mat_<float> Pose::r<cv::Mat_<float> >() const
  {
    cv::Mat_<float> r_float(3, 3);
    // TODO, speedup that process
    float * data = r_float.ptr<float> (0);
    for (unsigned int j = 0; j < 3; ++j)
      for (unsigned int i = 0; i < 3; ++i, ++data)
        *data = r_(j, i);
    return r_float;
  }

/** access the rotation matrix
 * @return the rotation matrix
 */
template<>
  cv::Mat Pose::r<cv::Mat>() const
  {
    return r<cv::Mat_<float> > ();
  }

/** access the translation vector
 * @return the translation vector
 */
const Eigen::Vector3f & Pose::t() const
{
  return t_;
}

/** access the translation vector
 * @return the translation vector
 */
template<>
  cv::Mat Pose::t<cv::Mat>() const
  {
    return (cv::Mat_<float>(3, 1) << t_(0), t_(1), t_(2));
  }

/** access the translation vector
 * @return the translation vector
 */
template<>
  cv::Mat_<float> Pose::t<cv::Mat_<float> >() const
  {
    return (cv::Mat_<float>(3, 1) << t_(0), t_(1), t_(2));
  }

/** set the rotation matrix
 * @param r the input rotation matrix
 */
template<>
  void Pose::setR<cv::Mat>(const cv::Mat & r)
  {
    cv::Mat_<float> R_float(3, 3), r_float;
    r.convertTo(r_float, CV_32F);

    //if (r_float.cols * r_float.rows == 1) - I think that it was a bug
    
    if (r_float.cols * r_float.rows == 3)
      cv::Rodrigues(r_float, R_float);
    else
      R_float = r_float;
    // TODO, speedup that process
    float * data = R_float.ptr<float> (0);
    for (unsigned int j = 0; j < 3; ++j)
      for (unsigned int i = 0; i < 3; ++i, ++data)
        r_(j, i) = *data;
  }

/** set the rotation matrix
 * @param r the input rotation matrix
 */
template<>
  void Pose::setR<cv::Mat_<float> >(const cv::Mat_<float> & r)
  {
    setR<cv::Mat>(r);
  }

/** set the rotation matrix
 * @param r the input rotation matrix
 */
template<>
  void Pose::setR<Eigen::Matrix3f>(const Eigen::Matrix3f & r)
  {
    r_ = r;
  }

/** set the translation vector
 * @param t the input translation vector matrix
 */
template<>
  void Pose::setT<cv::Mat>(const cv::Mat & t)
  {
    cv::Mat_<float> t_float(3, 1);
    t.convertTo(t_float, CV_32F);
    // TODO, speedup that process
    float * data = t_float.ptr<float> (0);
    for (unsigned int j = 0; j < 3; ++j, ++data)
    {
      t_(j) = *data;
    }
  }

/** set the translation vector
 * @param t the input translation vector matrix
 */
template<>
  void Pose::setT<cv::Mat_<float> >(const cv::Mat_<float> & t)
  {
    setT<cv::Mat> (t);
  }

/** set the translation vector
 * @param t the input translation vector matrix
 */
template<>
  void Pose::setT<Eigen::Vector3f>(const Eigen::Vector3f & t)
  {
    t_ = t;
  }

//serialization
void Pose::write(cv::FileStorage& fs) const
{
  cv::Mat rvec, rmat = r<cv::Mat> ();
  cv::Rodrigues(rmat, rvec);
  cvWriteComment(*fs, "PoseRT", 0);
  fs << "{" << "rvec" << rvec << "tvec" << t<cv::Mat> () << "estimated" << true << "}";
}

void Pose::read(const cv::FileNode& fn)
{
  cv::Mat rvec, tvec;
  fn["rvec"] >> rvec;
  fn["tvec"] >> tvec;
  setR(rvec);
  setT(tvec);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const std::string PoseRT::YAML_NODE_NAME = "pose";
PoseRT::PoseRT() :
  rvec(cv::Mat_<float>::zeros(3, 1)), tvec(cv::Mat_<float>::zeros(3, 1)), estimated(false)
{

}

PoseRT::PoseRT(const cv::Mat & rvec_in, const cv::Mat & tvec_in) :
  estimated(true) //prevent modification by reference!
{
  if (rvec.cols * rvec.rows == 9)
  {
    cv::Mat real_rvec;
    cv::Rodrigues(rvec_in, real_rvec);
    real_rvec.convertTo(rvec, CV_32F);
  }
  else
    rvec_in.convertTo(rvec, CV_32F);

  tvec_in.convertTo(tvec, CV_32F);
}

PoseRT::PoseRT(const Pose &pose) {
  pose.t<cv::Mat_<float> >().copyTo(tvec);
  cv::Rodrigues(pose.r<cv::Mat_<float> >(), rvec);
}

//serialization
void PoseRT::write(cv::FileStorage& fs) const
{
  cvWriteComment(*fs, "PoseRT", 0);
  fs << "{" << "rvec" << rvec << "tvec" << tvec << "estimated" << (int)estimated << "}";
}
void PoseRT::read(const cv::FileNode& fn)
{
  fn["rvec"] >> rvec;
  fn["tvec"] >> tvec;
  estimated = (int)fn["estimated"];
}

cv::Vec4f PoseRT::toPlanarCoefficients() const
{
  cv::Vec4f coefficients;
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  R.clone().convertTo(R, CV_32F);
  cv::Mat t;
  tvec.convertTo(t, CV_32F);
  cv::Mat n = (cv::Mat_<float>(3, 1) << 0, 0, 1);
  {
    cv::Mat Z = (cv::Mat_<float>(3, 1) << 0, 0, 1);
    n = R * Z; // n is unit vector normal to plane
  }
  float d = n.dot(-t); //plane given by n.(r - r_0) = 0, where r_0 is tvec;
  coefficients[0] = n.at<float> (0);
  coefficients[1] = n.at<float> (1);
  coefficients[2] = n.at<float> (2);
  coefficients[3] = d;
  return coefficients;
}

void PoseRT::enforceForm()
{
  CV_Assert(tvec.size().area() == 3 && (rvec.size().area() == 3 || rvec.size().area() == 9))
    ;
  if (rvec.size().area() == 9)
  {
    cv::Rodrigues(rvec.clone(), rvec);
  }
}

}
