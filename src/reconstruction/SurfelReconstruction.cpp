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

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>

#include <image_pipeline/pcl/conversions.hpp>

using ecto::tendrils;
namespace object_recognition
{
  namespace reconstruction
  {

    struct SurfelReconstruction
    {
      static void
      declare_params(tendrils& p)
      {
        p.declare<float>("maxInterpolationDist", "max dist to interpolate over in SurfelUpdateData", 0.01f);
        p.declare<float>("corrDistForUpdate",
                         "max dist of new measurement from surface to be considered part of that surface", 0.005);
        // p.declare<unsigned int>("highConfidence", "high confidence surfels are exempt from any removal");
        p.declare<int>("starvationConfidence", "this confidence and higher are exempt from starvation removal", 4);
        p.declare<int>("timeDiffForRemoval", "how long ago lastSeen must be to perform starvation removal", 5);
        p.declare<float>("maxNormalAngle", "max surfel angle away from camera to be updated", 80 * M_PI / 180);
        //p.declare<float>("maxNormalAngleDiff", "max difference between an new reading and the existing surfel normal",);

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

        outputs.declare<surfels::SurfelModel>("model", "The surfel model");
        outputs.declare<surfels::SurfelUpdateParams>("params", "The SurfelUpdateParams.");
        outputs.declare<surfels::CameraParams>("camera_params", "The CameraParams.");
      }

      void
      configure(const tendrils&p, const tendrils&i, const tendrils&o)
      {
        model = o["model"];
        params = o["params"];
        cam_params = o["camera_params"];
        R = i["R"];
        T = i["T"];
        K = i["K"];
        image = i["image"];
        mask = i["mask"];
        points3d = i["points3d"];
        params->maxInterpolationDist = p.get<float>("maxInterpolationDist");
        params->starvationConfidence = p.get<int>("starvationConfidence");
        params->timeDiffForRemoval = p.get<int>("timeDiffForRemoval");
        params->maxNormalAngle = p.get<float>("maxNormalAngle");
        params->corrDistForUpdate = p.get<float>("corrDistForUpdate");
      }

      int
      process(const tendrils& i, const tendrils& o)
      {
        cv::Mat mask = *this->mask;
        cv::Mat rvec, tvec, K;
        this->K->convertTo(K, CV_32F);
        R->convertTo(rvec, CV_32F);
        cv::Rodrigues(rvec.clone(), rvec);
        T->convertTo(tvec, CV_32F);
        cam_params->centerX = K.at<float>(0, 2);
        cam_params->centerY = K.at<float>(1, 2);
        cam_params->xRes = mask.size().width;
        cam_params->yRes = mask.size().height;
        cam_params->focalLength = K.at<float>(0, 0);

        //get the camera pose
        Eigen::Vector3f axis(rvec.at<float>(0), rvec.at<float>(1), rvec.at<float>(2));
        surfels::Transform3f objPose(Eigen::AngleAxisf(axis.norm(), axis.normalized()));
        objPose.pretranslate(Eigen::Vector3f(tvec.at<float>(0), tvec.at<float>(1), tvec.at<float>(2)));
        surfels::Transform3f camPose = objPose.inverse(Eigen::Isometry);
        //extract the cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cvToCloudOrganized(*points3d, *cloud, mask.size().width, mask.size().height);

        //compute normals

        // Estimate normals
        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal> result;
        ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(cloud);
        ne.compute(result);

        static int count_n = 0;
        std::cout << " n estimated " << count_n++ << std::endl;

        //copy into required structures
        boost::shared_ptr<boost::multi_array<Eigen::Vector3f, 2> > points(new boost::multi_array<Eigen::Vector3f, 2>());
        boost::shared_ptr<boost::multi_array<Eigen::Vector3f, 2> > normals(
            new boost::multi_array<Eigen::Vector3f, 2>());
        boost::shared_ptr<boost::multi_array<bool, 2> > canAdd(new boost::multi_array<bool, 2>());
        boost::shared_ptr<boost::multi_array<bool, 2> > validity(new boost::multi_array<bool, 2>());

        points->resize(boost::extents[cloud->width][cloud->height]);
        normals->resize(boost::extents[cloud->width][cloud->height]);
        canAdd->resize(boost::extents[cloud->width][cloud->height]);
        validity->resize(boost::extents[cloud->width][cloud->height]);

        for (unsigned int x = 0; x < cloud->width; x++)
        {
          for (unsigned int y = 0; y < cloud->height; y++)
          {
            bool mask_value = mask.at<unsigned char>(y, x) > 0;
            pcl::Normal n = result(x, y);
            pcl::PointXYZ p = (*cloud)(x, y);
            (*points)[x][y] = Eigen::Vector3f(p.x, p.y, p.z);
            (*normals)[x][y] = Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z).normalized();
            (*validity)[x][y] = (*canAdd)[x][y] = mask_value;
          }
        }

        //fill in updatedata
        surfels::SurfelUpdateData update_data;
        update_data.currentTime = time++;
        update_data.camParams = *cam_params;
        update_data.camTransform = camPose;
        update_data.image = *image;
        update_data.pointGrid = points;
        update_data.normalGrid = normals;
        update_data.validityGrid = validity;
        update_data.willingToAdd = canAdd;

        //perform the update
        std::vector<bool> can_update(model->surfels.size(), true);
        std::vector<unsigned int> removed;
        surfels::updateSurfelModel(*model, update_data, *params, can_update, removed);
        return ecto::OK;
      }
      ecto::spore<cv::Mat> R, T, K, mask, image, points3d;
      ecto::spore<surfels::SurfelModel> model;
      ecto::spore<surfels::SurfelUpdateParams> params;
      ecto::spore<surfels::CameraParams> cam_params;
      size_t time;
    };
    struct SurfelToPly
    {
      static void
      declare_params(tendrils& p)
      {
        p.declare<std::string>("filename", "The name of the ply file.", "surfel.ply");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<surfels::SurfelModel>("model", "The surfel model");
        inputs.declare<surfels::SurfelUpdateParams>("params", "The SurfelUpdateParams.");
        inputs.declare<surfels::CameraParams>("camera_params", "The CameraParams.");
      }

      void
      configure(const tendrils&p, const tendrils&i, const tendrils&o)
      {
        model = i["model"];
        params = i["params"];
        cam_params = i["camera_params"];
        filename = p["filename"];
      }

      int
      process(const tendrils& i, const tendrils& o)
      {
//        //clean up the model
        std::vector<unsigned int> removed;
        surfels::finalCleanup(*model, *params, removed);

        //convert to cloud
        pcl::PointCloud<surfels::surfelPt>::Ptr surfelCloud(new pcl::PointCloud<surfels::surfelPt>());
        surfels::convertModelToCloud(*model, *surfelCloud);

        //passthrough filter (remove anything at or below table height
        pcl::PointCloud<surfels::surfelPt>::Ptr tmpCloud(new pcl::PointCloud<surfels::surfelPt>());
//        pcl::PassThrough<surfels::surfelPt> pass;
//        pass.setInputCloud(surfelCloud);
//        pass.setFilterFieldName("z");
//        pass.setFilterLimits(0.001, 1.0);
//        pass.filter(*tmpCloud);
//        surfelCloud.swap(tmpCloud);

//statistical outlier filter
        pcl::StatisticalOutlierRemoval<surfels::surfelPt> sor;
        sor.setInputCloud(surfelCloud);
        sor.setMeanK(10);
        sor.setStddevMulThresh(2.0);
        sor.filter(*tmpCloud);
        surfelCloud.swap(tmpCloud);

        //try to determine the dimensions of the base
        float minX = 0, minY = 0, maxX = 0, maxY = 0;
        pcl::PassThrough<surfels::surfelPt> basePass;
        basePass.setInputCloud(surfelCloud);
        basePass.setFilterFieldName("z");
        basePass.setFilterLimits(0.001, 0.02);
        basePass.filter(*tmpCloud);
        for (unsigned int i = 0; i < tmpCloud->points.size(); i++)
        {
          surfels::surfelPt const& pt = tmpCloud->points[i];
          if (i == 0 || pt.x < minX)
            minX = pt.x;
          if (i == 0 || pt.x > maxX)
            maxX = pt.x;
          if (i == 0 || pt.y < minY)
            minY = pt.y;
          if (i == 0 || pt.y > maxY)
            maxY = pt.y;
        }
        float maxRadius = fabs(minX);
        if (maxX < maxRadius)
          maxRadius = maxX;
        if (fabs(minY) < maxRadius)
          maxRadius = fabs(minY);
        if (maxY < maxRadius)
          maxRadius = maxY;

        //fake seeing the bottom so the reconstruction won't have a lumpy bottom
        surfels::surfelPt origin, tmp;
        origin.x = origin.y = origin.z = 0;
        origin.normal_x = origin.normal_y = 0;
        origin.normal_z = -1.0;
        surfelCloud->points.push_back(origin);
        surfelCloud->width++;
        float radius_inc = 0.002;
        for (float radius = radius_inc; radius < maxRadius; radius += radius_inc)
        {
          float angle_inc = M_PI / (400 * radius);
          for (float theta = 0; theta < 2 * M_PI; theta += angle_inc)
          {
            tmp = origin;
            tmp.x += radius * cos(theta);
            tmp.y += radius * sin(theta);
            surfelCloud->points.push_back(tmp);
            surfelCloud->width++;
          }
        }

        //save as ply
        //rgbd::write_ply_file(*surfelCloud,out_file);
//        writePLY(*surfelCloud, *filename);
        return ecto::OK;
      }
      ecto::spore<std::string> filename;
      ecto::spore<surfels::SurfelModel> model;
      ecto::spore<surfels::SurfelUpdateParams> params;
      ecto::spore<surfels::CameraParams> cam_params;
      size_t time;
    };
  }
}

using namespace object_recognition::reconstruction;

ECTO_CELL(reconstruction, SurfelReconstruction, "SurfelReconstruction",
          "Reconstruct a series of observations into a surface using surfels.");

ECTO_CELL(reconstruction, SurfelToPly, "SurfelToPly", "Save a surfel as a ply file.");
