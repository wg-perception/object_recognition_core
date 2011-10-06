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
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

#include <object_recognition/common/conversions.hpp>
#include <ecto_pcl/ecto_pcl.hpp>

using ecto::tendrils;
namespace object_recognition
{
  namespace reconstruction
  {
    struct PointCloudTransform
    {
      typedef ecto::pcl::PointCloud CloudOutT;
      typedef pcl::PointXYZRGB Point;
      typedef pcl::PointCloud<Point> CloudT;
      typedef ecto::pcl::PointCloud CloudInT;

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare(&PointCloudTransform::R, "R", "Rotation matrix.").required(true);
        inputs.declare(&PointCloudTransform::T, "T", "Translation vector.").required(true);
        inputs.declare(&PointCloudTransform::cloudin, "cloud", "The input point cloud.").required(true);
        outputs.declare(&PointCloudTransform::cloudout, "view",
                        "The current 3d view transformed into object coordinates");
      }

      int
      process(const tendrils& i, const tendrils& o)
      {

        typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudNormalT;

        //extract the cloud
        CloudT::ConstPtr cloud = cloudin->cast<CloudT>();

        pcl::KdTree<Point>::Ptr tree_;
        pcl::NormalEstimation<Point, pcl::Normal> impl;
        pcl::PointCloud<pcl::Normal> normals;
        tree_.reset(new pcl::KdTreeFLANN<Point>);
        impl.setSearchMethod(tree_);
        impl.setInputCloud(cloud);
        impl.setKSearch(50);
        impl.compute(normals);

        CloudNormalT::Ptr cloud_with_normals(new CloudNormalT);
        pcl::concatenateFields(*cloud, normals, *cloud_with_normals);
        {
          CloudNormalT::Ptr tempc(new CloudNormalT);
          bool inverse = true;
          Eigen::Affine3f transform = RT2Transform(*R, *T, inverse); //compute the inverse transform
          pcl::transformPointCloudWithNormals(*cloud_with_normals, *tempc, transform);
          cloud_with_normals.swap(tempc);
        }
        *cloudout = cloud_with_normals;
        return ecto::OK;
      }
      ecto::spore<cv::Mat> R, T;
      ecto::spore<CloudInT> cloudin;
      ecto::spore<CloudOutT> cloudout;

    };
  }
}

using namespace object_recognition::reconstruction;

ECTO_CELL(
    reconstruction,
    PointCloudTransform,
    "PointCloudTransform",
    "Transform an pcl point cloud into the object cordinate, with a mask, and image to determine which points to transform.");
