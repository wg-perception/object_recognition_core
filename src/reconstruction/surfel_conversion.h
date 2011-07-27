/*
 * surfel_conversion.h
 *
 *  Created on: 2009-12-02
 *      Author: mkrainin
 */

#ifndef SURFEL_CONVERSION_H_
#define SURFEL_CONVERSTION_H_

#include <surfels.h>

//#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/register_point_struct.h>

#include <Eigen/Core>

namespace surfels{

struct surfelPt
{
        PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
        float rgb; //packed
        PCL_ADD_NORMAL4D;   // This adds the member normal[3] which can also be accessed using the point (which is float[4])
        float curvature;
        float radius;
        float confidence;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#ifdef RGBD_UTIL_USE_EIGEN3
} EIGEN_ALIGN16;
#else
} EIGEN_ALIGN_128;
#endif
inline std::ostream& operator << (std::ostream& os, const surfelPt& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.rgb << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.curvature << " - " << p.radius << " - " << p.confidence << ")";
  return (os);
}

/**
 * Returns a triangle list representing the surfel model. Each surfel is
 * approximated as a hexagon
 *
 * @param model
 * @param marker
 */
//void convertModelToMarker(
//                SurfelModel const& model,
//                visualization_msgs::Marker &marker);

/**
 * Returns a PointCloud representing a SurfelModel. Each surfel is represented
 * by a point in the cloud. Additionally, the cloud has channels for normal,
 * radius, confidence, and color
 *
 * @param model
 * @param cloud
 */
template <typename PointT>
void convertModelToCloud(SurfelModel const& model, pcl::PointCloud<PointT> &cloud);

/**
 * Returns a point cloud for an ArticulatedSurfelModel. This contains the
 * armModel and objectModel clouds concatenated together
 *
 * @param model
 * @param cloud
 */
template <typename PointT>
void convertArticulatedModelToCloud(ArticulatedSurfelModel const& model, pcl::PointCloud<PointT> & cloud);

/**
 * Returns a point cloud for a set of surfels
 *
 * @param surfels
 * @param surfelCloud
 */
template <typename PointT>
void convertSurfelsToPointCloud(std::vector<Surfel> const& surfels, pcl::PointCloud<PointT> &surfelCloud);

/**
 * Adds color channels to an existing point cloud
 *
 * @param colors
 * @param surfelCloud
 */
template <typename PointT>
void setSurfelCloudColors(std::vector<Eigen::Vector3f> const& colors, pcl::PointCloud<PointT> &surfelCloud);


} //namespace

POINT_CLOUD_REGISTER_POINT_STRUCT(
        surfels::surfelPt,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (float, normal[0], nx)
        (float, normal[1], ny)
        (float, normal[2], nz)
        (float, curvature, curvature)
        (float, radius, radius)
        (float, confidence, confidence)
);

#include "surfel_conversion.hpp"

#endif /* SURFEL_CONVERSION_H_ */
