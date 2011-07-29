/*
 * surfel_utility
 *
 * Evan Herbst
 * 7 / 5 / 10
 */

namespace surfels
{

  template<typename PointT>
  void convertSurfelsToPointCloud(std::vector<Surfel> const& surfels, pcl::PointCloud<PointT> &surfelCloud)
  {
    surfelCloud.points.resize(surfels.size());
    surfelCloud.width = surfelCloud.points.size();
    surfelCloud.height = 1;
    surfelCloud.is_dense = false;
    for(unsigned int i=0; i<surfels.size(); i++)
    {
      surfelCloud.points[i].x = surfels[i].location.x();
      surfelCloud.points[i].y = surfels[i].location.y();
      surfelCloud.points[i].z = surfels[i].location.z();
      surfelCloud.points[i].normal[0] = surfels[i].normal.x();
      surfelCloud.points[i].normal[1] = surfels[i].normal.y();
      surfelCloud.points[i].normal[2] = surfels[i].normal.z();
      surfelCloud.points[i].radius = surfels[i].radius;
      surfelCloud.points[i].confidence = surfels[i].visibility_confidence;
    }
  }

  inline float packRGB(const boost::array<float, 3> rgb) //args should be in [0, 1)
  {
    const uint32_t rgbi = ((uint32_t)(255 * rgb[0]) << 16) | ((uint32_t)(255 * rgb[1]) << 8) | (uint32_t)(255 * rgb[2]);
    return *reinterpret_cast<const float*>(&rgbi);
  }

  template <typename PointT>
  void setSurfelCloudColors(std::vector<Eigen::Vector3f> const& colors, pcl::PointCloud<PointT> &surfelCloud)
  {
    for(unsigned int i=0; i<colors.size(); i++)
    {
      const boost::array<float, 3> rgbArray = {{colors[i].x(), colors[i].y(), colors[i].z()}};
      surfelCloud.points[i].rgb = packRGB(rgbArray);
    }
  }

  template <typename PointT>
  void convertModelToCloud(SurfelModel const& model, pcl::PointCloud<PointT> &cloud)
  {
    convertSurfelsToPointCloud(model.surfels,cloud);
    setSurfelCloudColors(model.colors,cloud);
  }

  template <typename PointT>
  void convertArticulatedModelToCloud(ArticulatedSurfelModel const& model, pcl::PointCloud<PointT> & cloud)
  {
    //arm
    convertModelToCloud(model.armModel,cloud);

    //object
    pcl::PointCloud<PointT> objectCloud;
    convertModelToCloud(model.objectModel,objectCloud);

    cloud+=objectCloud;
  }

} //namespace
