/*
 * Reconstruction.cpp
 *
 *  Created on: Jun 30, 2011
 *      Author: mkrainin
 */

#include <ecto/ecto.hpp>
#include "surfels.h"
#include "surfel_conversion.h"

#include "boost/foreach.hpp"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/impl/passthrough.hpp"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/impl/statistical_outlier_removal.hpp"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transforms.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

using ecto::tendrils;
namespace object_recognition
{
  namespace reconstruction
  {
    struct rgb
    {
      union
      {
        unsigned char color[sizeof(float)];
        float data;
      };
    };

    void
    cvToCloudXYZRGB(const cv::Mat_<cv::Point3f>& points3d, pcl::PointCloud<pcl::PointXYZRGB>& cloud, const cv::Mat& rgb,
                    const cv::Mat& mask)
    {
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
        cp.r = (*rgb_it)[2];
        cp.g = (*rgb_it)[1];
        cp.b = (*rgb_it)[0];
        cloud.push_back(cp);
      }
    }

    template<typename PointT>
    void
    writePLY(const pcl::PointCloud<PointT>& cloud_m, const std::string& mesh_file_name)
    {
      std::ofstream mesh_file(std::string(mesh_file_name).c_str());
      mesh_file << "ply\n"
                "format ascii 1.0\n"
                "element vertex "
                << cloud_m.points.size() << "\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "property uchar red\n"
                "property uchar green\n"
                "property uchar blue\n"
//                "property float nx\n"
//                "property float ny\n"
//                "property float nz\n"
          "end_header\n";
      //<x> <y> <z> <r> <g> <b>
      for (size_t i = 0; i < cloud_m.points.size(); i++)
      {
        const PointT& p = cloud_m.points[i];
        mesh_file << p.x << " " << p.y << " " << p.z << " " << int(p.r) << " " << int(p.g) << " " << int(p.b)
        //<< " " << p.normal_x << " " << p.normal_y << " " << p.normal_z
                  << "\n";
      }
    }

    struct SimpleReconstruction
    {
      static void
      declare_params(tendrils& p)
      {

      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<cv::Mat>("R", "The the rotation matrix. 3x3.").required(true);
        inputs.declare<cv::Mat>("T", "The translation vector. 3x1.").required(true);
        inputs.declare<cv::Mat>("K", "The calibration matrix, 3x3").required(true);
        inputs.declare<cv::Mat>("image", "The rgb image.").required(true);
        inputs.declare<cv::Mat>("mask", "The binary mask for valid points.").required(true);
        inputs.declare<cv::Mat>("points3d", "The 3d points.").required(true);
      }

      void
      configure(const tendrils&p, const tendrils&i, const tendrils&o)
      {
        R = i["R"];
        T = i["T"];
        K = i["K"];
        image = i["image"];
        mask = i["mask"];
        points3d = i["points3d"];
      }

      int
      process(const tendrils& i, const tendrils& o)
      {
        cv::Mat mask = *this->mask;

        //convert the tranform from our fiducial markers to
        //the Eigen
        Eigen::Matrix<float, 3, 3> eR;
        Eigen::Vector3f eT;
        cv::cv2eigen(*R, eR);
        cv::cv2eigen(*T, eT);

        Eigen::Affine3f transform;
        transform = Eigen::Translation3f(-eT);
        transform.prerotate(eR.transpose());
        std::cout << "R = " << eR << "\n T= " << eT << std::endl;
        //extract the cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cvToCloudXYZRGB(*points3d, cloud, *image, mask);
        pcl::transformPointCloud(cloud, *transformed, transform);


        if (!full_cloud)
        {
          full_cloud = transformed;
        }
        else
        {
          *full_cloud += *transformed;
        }
        static int count = 0;
        writePLY(*transformed, boost::str(boost::format("view%04d.ply") % count++));
        writePLY(*full_cloud, "model.ply");

        return ecto::OK;
      }
      ecto::spore<cv::Mat> R, T, K, mask, image, points3d;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud;
    };
  }
}

using namespace object_recognition::reconstruction;

ECTO_CELL(reconstruction, SimpleReconstruction, "SimpleReconstruction",
          "Reconstruct a series of observations into a surface.");
