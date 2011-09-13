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
#include <pcl/filters/voxel_grid.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

#include <object_recognition/common/conversions.hpp>

using ecto::tendrils;
namespace object_recognition
{
  namespace reconstruction
  {
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
        //extract the cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>()), tempRGB(
            new pcl::PointCloud<pcl::PointXYZRGB>());
        cvToCloudXYZRGB(*points3d, *cloud, *image, *mask);
        std::cout << " cvToCloudXYZRGB " << cloud->points.size() << " points on object." << std::endl;
////
////        // Create the filtering object
////        pcl::VoxelGrid<pcl::PointXYZRGB> vox;
////        vox.setInputCloud(cloud);
////        vox.setLeafSize(0.01, 0.01, 0.01);
////        vox.filter(*tempRGB);
////        cloud.swap(tempRGB);
////        std::cout << "  voxeled " << cloud->points.size() << " points on object." << std::endl;
//        {
//          // Create the filtering object
//          pcl::VoxelGrid<pcl::PointXYZRGB> vox;
//          vox.setInputCloud(full_cloud);
//          vox.setLeafSize(0.0025, 0.0025, 0.0025);
//          vox.filter(*tempRGB);
//          full_cloud.swap(tempRGB);
//          std::cout << "  voxeled " << full_cloud->points.size() << " points on object." << std::endl;
//        }
//        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>), temp(
//            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//        {
//          // Create the normal estimation class, and pass the input dataset to it
//          pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
//          ne.setInputCloud(cloud);
//
////          // Create an empty kdtree representation, and pass it to the normal estimation object.
////          // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
////          pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());
////          ne.setSearchMethod(tree);
//
//          // Output datasets
//
//          // Use all neighbors in a sphere of radius 3cm
//          ne.setRadiusSearch(0.01);
//
//          // Compute the features
//          ne.compute(*cloud_normals);
//        }
//        std::cout << " normals " << std::endl;
////
////        {
////          bool inverse = true;
////          Eigen::Affine3f transform = RT2Transform(*R, *T, inverse); //compute the inverse transform
////          pcl::transformPointCloud(*cloud_normals, *temp, transform);
////          cloud_normals.swap(temp);
////        }

        {
          bool inverse = true;
          Eigen::Affine3f transform = RT2Transform(*R, *T, inverse); //compute the inverse transform
          pcl::transformPointCloud(*cloud, *tempRGB, transform);
          cloud.swap(tempRGB);
        }
        std::cout << " transformed " << std::endl;
        if (!full_cloud)
        {
          full_cloud = cloud;
        }
        else
        {
          *full_cloud += *cloud;
        }

//        {
//          // Create the filtering object
//          pcl::VoxelGrid<pcl::PointXYZRGB> vox;
//          vox.setInputCloud(full_cloud);
//          vox.setLeafSize(0.0025, 0.0025, 0.0025);
//          vox.filter(*tempRGB);
//          full_cloud.swap(tempRGB);
//          std::cout << "  voxeled " << full_cloud->points.size() << " points on object." << std::endl;
//        }
//        {
//          pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
//          sor.setInputCloud(full_cloud);
//          sor.setMeanK(5);
//          sor.setStddevMulThresh(2.0);
//          sor.filter(*tempRGB);
//          full_cloud.swap(tempRGB);
//          std::cout << "  filtered " << full_cloud->points.size() << std::endl;
//        }

//        {
//          // Create the filtering object
//          pcl::VoxelGrid<pcl::PointXYZRGB> vox;
//          vox.setInputCloud(full_cloud);
//          vox.setLeafSize(0.01, 0.01, 0.01);
//          vox.filter(*tempRGB);
//          std::cout << "  voxeled final " << tempRGB->points.size() << " points on object." << std::endl;
//          writePLY(*tempRGB, "model.ply");
//
//        }
        writePLY(*full_cloud, "model.ply");

        //static int count = 0;
        //writePLY(*transformed, boost::str(boost::format("view%04d.ply") % count++));
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
