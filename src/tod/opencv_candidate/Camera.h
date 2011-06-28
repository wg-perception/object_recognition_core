#ifndef opencv_candidate_CAMERA_H_
#define opencv_candidate_CAMERA_H_
#include <opencv_candidate/PoseRT.h>
#include <opencv2/core/core.hpp>
#include <stdexcept>
namespace opencv_candidate
{


/** \brief Relevant camera calibration parameters, and their serialization and deserialization.
 *
 * Also useful for projection of points into the camera plane.
 */
class Camera
{
public:
  enum CalibrationFormat {OPENCV_YAML = 0, TOD_YAML=1};
  Camera();
  /**
   * \brief Construct a camera from from known intrinsic matrix and distortion
   * parameters.
   *
   * @param K a 3x3 floating point camera matrix.
   * @param D a 5x1 or 8x1 floating point vector of distortion parameters.
   */
  Camera(const cv::Mat& K, const cv::Mat& D);
  /** Deserialize from a camera.yml file - opencv style yaml calibration
   */
  Camera(const std::string& calibfile, CalibrationFormat format);
  /** Deserialize from an opencv fs object
   */
  explicit Camera(const cv::FileNode& fn);

  virtual ~Camera();

  cv::Mat K; //!< intrinsics
  cv::Mat D; //!< distortion coeffs
  cv::Mat Kinv; //!< inverse intrinsics - precomputed for projecting points
  cv::Size image_size; //!< image size - the size of the image that this calibration is based on
  PoseRT pose; //!< camera pose with respect to some arbitrary world frame.

  /** \brief Given world points, project them onto the image plane.
   *
   * \param worldPoints a set of points to project
   * \param pts the output image plane points
   * \param use_pose use the camera pose as part of the project,
   *        if false will assume that the points are already in the coordinate frame of the camera
   */
  void projectPoints(const std::vector<cv::Point3f>& worldPoints, std::vector<cv::Point2f>& pts, bool use_pose = false) const;

  /**
   * Projects img_pts to rays.
   */
  void reprojectPoints(const std::vector<cv::Point2f>& img_pts, std::vector<cv::Point3f>& rays) const{
    throw std::logic_error("not implemented!");
  }

  //serialization
  void write(cv::FileStorage& fs) const;
  void read(const cv::FileNode& fn);

  static const std::string YAML_NODE_NAME;
};

}

#endif /* CAMERA_H_ */
