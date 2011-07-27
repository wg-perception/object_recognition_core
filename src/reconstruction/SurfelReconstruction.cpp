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

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace object_recognition
{
  namespace reconstruction
  {
    struct SurfelReconstruction
    {
    };
  }
}

using namespace object_recognition::reconstruction;

ECTO_CELL(reconstruction, SurfelReconstruction,"SurfelReconstruction","Reconstruct a series of observations into a surface using surfels.")

#if 0
Reconstruction::Reconstruction()
{
  params.maxInterpolationDist = 0.01;
  params.starvationConfidence = 2;
  params.maxNormalAngle = 85;
  params.corrDistForUpdate = 0.015;

  cam_params.centerX = 319.5;
  cam_params.centerY = 239.5;
  cam_params.xRes = 640;
  cam_params.yRes = 480;
  cam_params.focalLength = 525;
}

Reconstruction::~Reconstruction()
{
}

void Reconstruction::addFrame(FrameData const& data, unsigned int time)
{
  //get the camera pose
  Eigen::Vector3f axis(data.pose->rvec[0],data.pose->rvec[1],data.pose->rvec[2]);
  surfels::Transform3f objPose(Eigen::AngleAxisf(axis.norm(),axis.normalized()));
  objPose.pretranslate(Eigen::Vector3f(data.pose->tvec[0],data.pose->tvec[1],data.pose->tvec[2]));
  surfels::Transform3f camPose = objPose.inverse(Eigen::Isometry);

  //resize the images
  cv::Mat image_resized, mask_resized;
  cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(data.image,"bgr8");
  cv_bridge::CvImagePtr cv_mask_img = cv_bridge::toCvCopy(data.mask);
  cv::Mat image_roi(cv_img->image,cv::Rect(0,0,1280,960));
  cv::Mat maks_roi(cv_mask_img->image,cv::Rect(0,0,1280,960));
  cv::resize(image_roi,image_resized,cv::Size(640,480));
  cv::resize(maks_roi,mask_resized,cv::Size(640,480),0,0,cv::INTER_NEAREST);

  //extract the cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
  pcl::MsgFieldMap mapping;
  pcl::createMapping<pcl::PointXYZ>(data.cloud->fields,mapping);
  pcl::fromROSMsg(*data.cloud, *cloud, mapping);

  //compute normals
  boost::shared_ptr<boost::multi_array<bool, 2> > validity(new boost::multi_array<bool, 2>());
  rgbd::getValidityGrid(*cloud,*validity);
  rgbd::setOrganizedNormals(*cloud,*validity,2,2,0.02,false,true);

  //copy into required structures
  boost::shared_ptr<boost::multi_array<Eigen::Vector3f, 2> > points(
      new boost::multi_array<Eigen::Vector3f, 2>());
  boost::shared_ptr<boost::multi_array<Eigen::Vector3f, 2> > normals(
      new boost::multi_array<Eigen::Vector3f, 2>());
  boost::shared_ptr<boost::multi_array<bool, 2> > canAdd(new boost::multi_array<bool, 2>());
  points->resize(boost::extents[cloud->width][cloud->height]);
  normals->resize(boost::extents[cloud->width][cloud->height]);
  canAdd->resize(boost::extents[cloud->width][cloud->height]);
  for(unsigned int x=0; x<cloud->width; x++){
    for(unsigned int y=0; y<cloud->height; y++){
      if((*validity)[x][y]){
        int i = x + y*cloud->width;
        pcl::PointNormal const& pt = cloud->points[i];

        (*points)[x][y] = Eigen::Vector3f(pt.x,pt.y,pt.z);
        (*normals)[x][y] = Eigen::Vector3f(pt.normal_x,pt.normal_y,pt.normal_z).normalized();
        (*canAdd)[x][y] = mask_resized.at<unsigned char>(y,x) > 0;
      }
    }
  }

  //fill in updatedata
  surfels::SurfelUpdateData update_data;
  update_data.currentTime = time;
  update_data.camParams = cam_params;
  update_data.camTransform = camPose;
  update_data.image = image_resized;
  update_data.pointGrid = points;
  update_data.normalGrid = normals;
  update_data.validityGrid = validity;
  update_data.willingToAdd = canAdd;

  //perform the update
  std::vector<bool> can_update(model.surfels.size(),true);
  std::vector<unsigned int> removed;
  surfels::updateSurfelModel(model,update_data,params,can_update,removed);
}

void Reconstruction::reconstruct(
    rosbag::Bag bag, std::vector<std::string> topics,std::string out_file)
{
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  unsigned int frame_num = 0;
  FrameData data;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    sensor_msgs::ImageConstPtr image = m.instantiate<sensor_msgs::Image> ();
    if (image != NULL)
    {
      if(m.getTopic() == "image_mask")
        data.mask = image;
      else if(m.getTopic() == "image_color")
        data.image = image;
      else
        assert(false && "unknown topic");
    }
    sensor_msgs::PointCloud2ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2> ();
    if (cloud != NULL)
    {

      if(m.getTopic() == "points")
        data.cloud = cloud;
      else if(m.getTopic() == "points_registered")
        data.registered = cloud;
      else
        assert(false && "unknown topic");
    }
    tod_training::PoseRTConstPtr pose = m.instantiate<tod_training::PoseRT> ();
    if (pose != NULL)
    {
      data.pose = pose;
    }

    if (data.full())
    {
      ROS_INFO("Adding frame");
      this->addFrame(data,frame_num);
      data.clear();
    }
  }

  //clean up the model
  std::vector<unsigned int> removed;
  surfels::finalCleanup(model,params,removed);

  //convert to cloud
  pcl::PointCloud<surfels::surfelPt>::Ptr surfelCloud(new pcl::PointCloud<surfels::surfelPt>());
  surfels::convertModelToCloud(model,*surfelCloud);

  //passthrough filter (remove anything at or below table height
  pcl::PointCloud<surfels::surfelPt>::Ptr tmpCloud(new pcl::PointCloud<surfels::surfelPt>());
  pcl::PassThrough<surfels::surfelPt> pass;
  pass.setInputCloud(surfelCloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.001,1.0);
  pass.filter(*tmpCloud);
  surfelCloud.swap(tmpCloud);

  //statistical outlier filter
  pcl::StatisticalOutlierRemoval<surfels::surfelPt> sor;
  sor.setInputCloud (surfelCloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (2.0);
  sor.filter (*tmpCloud);
  surfelCloud.swap(tmpCloud);

  //try to determine the dimensions of the base
  float minX,minY,maxX,maxY;
  pcl::PassThrough<surfels::surfelPt> basePass;
  pass.setInputCloud(surfelCloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.001,0.02);
  pass.filter(*tmpCloud);
  for(unsigned int i=0; i<tmpCloud->points.size(); i++){
    surfels::surfelPt const& pt = tmpCloud->points[i];
    if(i==0 || pt.x < minX) minX = pt.x;
    if(i==0 || pt.x > maxX) maxX = pt.x;
    if(i==0 || pt.y < minY) minY = pt.y;
    if(i==0 || pt.y > maxY) maxY = pt.y;
  }
  float maxRadius = fabs(minX);
  if(maxX < maxRadius) maxRadius = maxX;
  if(fabs(minY) < maxRadius) maxRadius = fabs(minY);
  if(maxY < maxRadius) maxRadius = maxY;
  std::cout<<"base points: "<<tmpCloud->points.size()<<std::endl;
  std::cout<<"max radius: "<<maxRadius<<std::endl;

  //fake seeing the bottom so the reconstruction won't have a lumpy bottom
  surfels::surfelPt origin, tmp;
  origin.x = origin.y = origin.z = 0;
  origin.normal_x = origin.normal_y = 0;
  origin.normal_z = -1.0;
  surfelCloud->points.push_back(origin);
  surfelCloud->width++;
  float radius_inc = 0.002;
  for(float radius=radius_inc; radius<maxRadius; radius+=radius_inc){
    float angle_inc = M_PI/(400*radius);
    for(float theta=0; theta<2*M_PI; theta+=angle_inc){
      tmp = origin;
      tmp.x+=radius*cos(theta); tmp.y+=radius*sin(theta);
      surfelCloud->points.push_back(tmp);
      surfelCloud->width++;
    }
  }

  //save as ply
  rgbd::write_ply_file(*surfelCloud,out_file);

}

}
#endif
