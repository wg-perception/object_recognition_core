/*
 * surfel_conversion.cpp
 *
 *  Created on: Jul 12, 2011
 *      Author: mkrainin
 */

#include "surfel_conversion.h"

namespace surfels
{
surfelPt EIGEN_ALIGN_128;
/*
void convertModelToMarker(SurfelModel const& model, visualization_msgs::Marker &marker)
{
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "surfel_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 1; //flatten in x direction to give pre-rotation normal of (1,0,0)
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1.0;

  marker.points.resize(12 * model.surfels.size());
  marker.colors.resize(12 * model.surfels.size());
  for (unsigned int i = 0; i < model.surfels.size(); i++)
  {
    Surfel const& surfel = model.surfels[i];
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = model.colors[i].x();
    color.g = model.colors[i].y();
    color.b = model.colors[i].z();

    float len = 2 / sqrt(3) * surfel.radius;

    //the 6 vertices of the hexagon
    std::vector<geometry_msgs::Point> vertices(6);
    for (unsigned int j = 0; j < 6; j++)
    {
      Eigen::Vector3f vertex = surfel.location + len * (Eigen::AngleAxisf(j / 3.0 * M_PI, surfel.normal)
          * surfel.reference_dir);
      vertices[j].x = vertex.x();
      vertices[j].y = vertex.y();
      vertices[j].z = vertex.z();
    }

    bool illegal_vertex = false;
    for (unsigned int j = 0; j < 6; j++)
    {
      if (std::isnan(vertices[j].x) || std::isnan(vertices[j].y) || std::isnan(vertices[j].z))
      {
        std::cout << "NaN in vertex: " << j << " of surfel " << i << std::endl;
        illegal_vertex = true;
        break;
      }
      if (std::isinf(vertices[j].x) || std::isinf(vertices[j].y) || std::isinf(vertices[j].z))
      {
        std::cout << "Inf in vertex: " << j << " of surfel " << i << std::endl;
        illegal_vertex = true;
        break;
      }
    }
    if (illegal_vertex)
    {
      std::cout << "Location:" << surfel.location.transpose() << std::endl;
      std::cout << "Normal:" << surfel.normal.transpose() << std::endl;
      std::cout << "Reference Direction:" << surfel.reference_dir.transpose() << std::endl;
      std::cout << "Not adding marker for surfel with illegal values..." << std::endl;
      continue;
    }

    //add the 4 triangles. at present, this requires redundantly adding vertices

    //triangle 1
    marker.points[12 * i + 0] = vertices[0];
    marker.points[12 * i + 1] = vertices[2];
    marker.points[12 * i + 2] = vertices[4];
    //triangle 2
    marker.points[12 * i + 3] = vertices[0];
    marker.points[12 * i + 4] = vertices[1];
    marker.points[12 * i + 5] = vertices[2];
    //triangle 3
    marker.points[12 * i + 6] = vertices[2];
    marker.points[12 * i + 7] = vertices[3];
    marker.points[12 * i + 8] = vertices[4];
    //triangle 4
    marker.points[12 * i + 9] = vertices[0];
    marker.points[12 * i + 10] = vertices[4];
    marker.points[12 * i + 11] = vertices[5];

    for (unsigned int j = 0; j < 12; j++)
      marker.colors[12 * i + j] = color;
  }
}
*/
}//namespace
