#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <opencv2/core/eigen.hpp>

#include <fstream>
namespace object_recognition
{
  /**
   * \breif convert an opencv collection of points to a pcl::PoinCloud, your opencv mat should have NAN's for invalid points.
   * @param points3d opencv matrix of nx1 3 channel points
   * @param cloud output cloud
   * @param rgb the rgb, required, will color points
   * @param mask the mask, required, must be same size as rgb
   */
  inline void
  cvToCloudXYZRGB(const cv::Mat_<cv::Point3f>& points3d, pcl::PointCloud<pcl::PointXYZRGB>& cloud, const cv::Mat& rgb,
                  const cv::Mat& mask, bool brg = true)
  {
    cloud.clear();
    cv::Mat_<cv::Point3f>::const_iterator point_it = points3d.begin(), point_end = points3d.end();
    cv::Mat_<cv::Vec3b>::const_iterator rgb_it = rgb.begin<cv::Vec3b>();
    cv::Mat_<uchar>::const_iterator mask_it = mask.begin<uchar>();
    for (; point_it != point_end; ++point_it, ++mask_it, ++rgb_it)
    {
      if (!*mask_it)
        continue;
      cv::Point3f p = *point_it;
      if (p.x != p.x && p.y != p.y && p.z != p.z) //throw out NANs
        continue;
      pcl::PointXYZRGB cp;
      cp.x = p.x;
      cp.y = p.y;
      cp.z = p.z;
      cp.r = (*rgb_it)[2]; //expecting in BGR format.
      cp.g = (*rgb_it)[1];
      cp.b = (*rgb_it)[0];
      cloud.push_back(cp);
    }
  }

  template<typename PointT>
  inline void
  cvToCloud(const cv::Mat_<cv::Point3f>& points3d, pcl::PointCloud<PointT>& cloud, const cv::Mat& mask = cv::Mat())
  {
    cloud.clear();
    cloud.width = points3d.size().width;
    cloud.height = points3d.size().height;
    cv::Mat_<cv::Point3f>::const_iterator point_it = points3d.begin(), point_end = points3d.end();
    const bool has_mask = !mask.empty();
    cv::Mat_<uchar>::const_iterator mask_it;
    if (has_mask)
      mask_it = mask.begin<uchar>();
    for (; point_it != point_end; ++point_it, (has_mask ? ++mask_it : mask_it))
    {
      if (has_mask && !*mask_it)
        continue;
      cv::Point3f p = *point_it;
      if (p.x != p.x && p.y != p.y && p.z != p.z) //throw out NANs
        continue;
      PointT cp;
      cp.x = p.x;
      cp.y = p.y;
      cp.z = p.z;
      cloud.push_back(cp);
    }
  }

  template<typename PointT>
  inline void
  cvToCloudOrganized(const cv::Mat_<cv::Point3f>& points3d, pcl::PointCloud<PointT>& cloud, size_t width, size_t height,
                     const cv::Mat& mask = cv::Mat())
  {
    cloud.points.resize(width * height);
    cloud.width = width;
    cloud.height = height;

    for (size_t v = 0; v < height; ++v)
    {
      const float * begin = reinterpret_cast<const float*>(points3d.ptr(v));for (size_t u = 0; u < width; ++u)
      {
        PointT& p = cloud(u, v);
        p.x = *(begin++);
        p.y = *(begin++);
        p.z = *(begin++);
      }
    }
  }

      /**
       * p = R*x + T
       * if inverse:
       *  x = R^1*(p - T)
       */
  inline Eigen::Affine3f
  RT2Transform(cv::Mat& R, cv::Mat& T, bool inverse)
  {
    //convert the tranform from our fiducial markers to
    //the Eigen
    Eigen::Matrix<float, 3, 3> eR;
    Eigen::Vector3f eT;
    cv::cv2eigen(R, eR);
    cv::cv2eigen(T, eT);
    // p = R*x + T
    Eigen::Affine3f transform;
    if (inverse)
    {
      //x = R^1*(p - T)
      transform = Eigen::Translation3f(-eT);
      transform.prerotate(eR.transpose());
    }
    else
    {
      //p = R*x + T
      transform = Eigen::AngleAxisf(eR);
      transform.translate(eT);
    }
    return transform;
  }
}
