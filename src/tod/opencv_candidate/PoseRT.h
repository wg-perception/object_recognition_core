/*
 * pose.h
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

#ifndef CVC_POSE_H_
#define CVC_POSE_H_

#include <iostream>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>

namespace opencv_candidate
{

// Forward declaration
class Pose;

/**\brief A pose data structure, contains the rotation and translation of an object
 */
class PoseRT
{
public:
  PoseRT();
  PoseRT(const cv::Mat &rvec, const cv::Mat& tvec);
  PoseRT(const Pose &pose);
  ~PoseRT()
  {
  }
  cv::Mat rvec; //!<rodriguez formula rotation 3 vector
  cv::Mat tvec; //!<3 vector, translation
  bool estimated;

  void write(cv::FileStorage& fs) const;
  void read(const cv::FileNode& fn);

  static const std::string YAML_NODE_NAME;
  cv::Vec4f toPlanarCoefficients() const;
  void enforceForm();
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**class Rotation : public Eigen::RotationBase<float, 3> {

};*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** This is an API unstable class for the pose object. It will become a better/clearer pose object
 */
class Pose
{
public:
  /** access the rotation matrix
   * @return the rotation matrix
   */
  const Eigen::Matrix3f & r() const;

  /** access the rotation matrix
   * @return the rotation matrix
   */
  template<typename MatrixType>
    MatrixType r() const;

  /** access the translation vector
   * @return the translation vector
   */
  const Eigen::Vector3f & t() const;

  /** access the translation vector
   * @return the translation vector
   */
  template<typename VectorType>
    VectorType t() const;

  /** set the rotation matrix
   * @param r the input rotation matrix
   */
  template<typename MatrixType>
    void setR(const MatrixType & r);

  /** set the translation vector
   * @param t the input translation vector matrix
   */
  template<typename VectorType>
    void setT(const VectorType & t);

  void write(cv::FileStorage& fs) const;
  void read(const cv::FileNode& fn);

  static const std::string YAML_NODE_NAME;
private:
  /** The rotation matrix
   */
  Eigen::Matrix3f r_;

  /** The translation vector
   */
  Eigen::Vector3f t_;
};

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline std::ostream& operator <<(std::ostream& out, const opencv_candidate::PoseRT& pose)
{
  out << "r=" << pose.rvec << ", t=" << pose.tvec;
  return out;
}

inline std::ostream& operator <<(std::ostream& out, const opencv_candidate::Pose& pose)
{
  out << "r=" << pose.r() << ", t=" << pose.t();
  return out;
}

#endif /* POSE_H_ */
