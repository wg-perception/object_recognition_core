/*
 * surfels.h
 *
 *  Created on: 2009-12-02
 *      Author: mkrainin
 */

#ifndef SURFELS_H_
#define SURFELS_H_

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <vector>
#include <set>

#include <boost/array.hpp>
#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>

namespace surfels
{

//quantization of viewing angles
const unsigned int num_azimuth_angles = 8;
const unsigned int num_polar_angles = 8;

typedef Eigen::Isometry3f Transform3f;

struct CameraParams{
        unsigned int xRes;
        unsigned int yRes;
        float centerX;
        float centerY;
        float focalLength;
};

/**
 * A surfel is a disc-shaped surface region with a location, normal, radius,
 * and confidence
 */
struct Surfel
{
  Eigen::Vector3f location;
  Eigen::Vector3f normal;

  //for discretizing azimuth angle
  Eigen::Vector3f reference_dir;

  float radius;unsigned int visibility_confidence;

  unsigned int total_views;
  boost::array<boost::array<bool, num_polar_angles>, num_azimuth_angles> observed_directions;

  Surfel()
  {
    total_views = 0;
    visibility_confidence = 0;

    for (unsigned int i = 0; i < num_azimuth_angles; i++)
    {
      for (unsigned int j = 0; j < num_polar_angles; j++)
      {
        observed_directions[i][j] = false;
      }
    }
  }
};

struct colorcomp
{
  bool operator()(const Eigen::Vector3f& lhs, const Eigen::Vector3f& rhs) const
  {
    //compare intensities
    return (0.299 * lhs.x() + 0.587 * lhs.y() + 0.114 * lhs.z())
        < (0.299 * rhs.x() + 0.587 * rhs.y() + 0.114 * rhs.z());
  }
};

/**
 * A collection of surfels and associated attributes for the surfels.
 * Each surfel has a color and a last seen time. minAngleFromNormal stores
 * the minimum polar viewing angle as a heuristic for color selection
 */
struct SurfelModel
{
  std::vector<Surfel> surfels;
  std::vector<Eigen::Vector3f> colors;
  std::vector<unsigned int> lastSeen;
  // These are now only allocated if you're using that color model:
  std::vector<std::multiset<Eigen::Vector3f, colorcomp> > allColors; // only for median color
  std::vector<float> minAngleFromNormal; // only for best view color
  std::vector<float> minDistanceSeen; // only for nearest color
};

/**
 * A structure for representing a surfel model of a robotic arm. Each surfel
 * in the arm model has a link number. In addition, there is a (possibly empty)
 * SurfelModel for an object being manipulated by the arm.
 */
struct ArticulatedSurfelModel
{
  SurfelModel armModel;
  std::vector<unsigned int> links;

  //object being manipulated
  SurfelModel objectModel;
};

/**
 * Data from a single frame for updating a SurfelModel.
 *
 * Contains a camera image and grid structures containing points and normals
 * (indexed as [x][y], stored in camera coordinates).
 * Each (x,y) also has a validity (whether there was a reading)
 * and whether that reading can be added to the model (whether it is believed to
 * belong to the object as opposed to background, for example)
 *
 * Additionally, the intrinsic camera parameters and camera transform
 * (in object model coordinates) are required.
 */
struct SurfelUpdateData
{
  unsigned int currentTime;

  cv::Mat image; //expects bgr8

  //all of the 3D data indexed by x,y coordinates
  boost::shared_ptr<const boost::multi_array<Eigen::Vector3f, 2> > pointGrid;
  boost::shared_ptr<const boost::multi_array<Eigen::Vector3f, 2> > normalGrid;
  boost::shared_ptr<const boost::multi_array<bool, 2> > validityGrid;
  boost::shared_ptr<const boost::multi_array<bool, 2> > willingToAdd;

  CameraParams camParams;
  Transform3f camTransform;

  SurfelUpdateData()
  {
    camTransform.setIdentity();
  }
};

/**
 * Parameters for updating a SurfelModel
 */
enum SurfelUpdateParamsColorUpdateRule
{
  COLOR_UPDATE_BEST_VIEWPOINT, COLOR_UPDATE_MEDIAN_COLOR, COLOR_UPDATE_FIRST_COLOR, COLOR_UPDATE_LAST_COLOR,
  COLOR_UPDATE_NEAREST
};

struct SurfelUpdateParams
{
  //max dist to interpolate over in SurfelUpdateData
  float maxInterpolationDist;
  //max dist of new measurement from surface to be considered part of that surface
  float corrDistForUpdate;

  //high confidence surfels are exempt from any removal
  unsigned int highConfidence;
  //this confidence and higher are exempt from starvation removal
  unsigned int starvationConfidence;
  //how long ago lastSeen must be to perform starvation removal
  unsigned int timeDiffForRemoval;

  //max surfel angle away from camera to be updated
  float maxNormalAngle;
  //max difference between an new reading and the existing surfel normal
  float maxNormalAngleDiff;

#if 0
  // color selection:  Earliest true value gets priority
  bool useOriginalColor;

  //color selection. median (computed in 1-d intensity) requires storing every color. otherwise, uses
  //most straight on color
  bool useMedianColor;
#endif
  SurfelUpdateParamsColorUpdateRule colorUpdateRule;

  SurfelUpdateParams()
  {
    maxInterpolationDist = .02;
    corrDistForUpdate = .005;
    highConfidence = 5;
    starvationConfidence = 3;
    timeDiffForRemoval = 30;
    maxNormalAngle = 70 * M_PI / 180;
    maxNormalAngleDiff = 45 * M_PI / 180;
    colorUpdateRule = COLOR_UPDATE_BEST_VIEWPOINT;
  }

  bool setColorRule(std::string colorRuleString)
  {
    if (colorRuleString == "COLOR_UPDATE_BEST_VIEWPOINT")
    {
      colorUpdateRule = COLOR_UPDATE_BEST_VIEWPOINT;
    }
    else if (colorRuleString == "COLOR_UPDATE_MEDIAN_COLOR")
    {
      colorUpdateRule = COLOR_UPDATE_MEDIAN_COLOR;
    }
    else if (colorRuleString == "COLOR_UPDATE_FIRST_COLOR")
    {
      colorUpdateRule = COLOR_UPDATE_FIRST_COLOR;
    }
    else if (colorRuleString == "COLOR_UPDATE_LAST_COLOR")
    {
      colorUpdateRule = COLOR_UPDATE_LAST_COLOR;
    }
    else if (colorRuleString == "COLOR_UPDATE_NEAREST")
    {
      colorUpdateRule = COLOR_UPDATE_NEAREST;
    }
    else
    {
      return false;
    }
    return true;
  }
};

/*
 *
 * Updating surfel models
 *
 */

/**
 * Updates a SurfelModel (update, addition, and removal) based on new SurfelUpdateData.
 * It is recommended that all updating of a SurfelModel be done through this function
 * (as opposed to directly calling addObservation, createSurfel, etc)
 *
 * @param surfelModel contains the surfels and associated data
 * @param data contains the sensor data to be incorporated
 * @param params update parameters
 * @param canUpdateSurfel whether the surfel can be updated
 * @param removed which surfels (in old surfel indices) were removed
 */
void updateSurfelModel(SurfelModel &surfelModel, SurfelUpdateData &data, SurfelUpdateParams &params,
                       std::vector<bool> & canUpdateSurfel, std::vector<unsigned int> &removed);

/*
 *
 * Transforming surfels
 *
 */

/**
 * Transforms a surfel in place with a specified transform
 *
 * @param surfel
 * @param transform
 */
void transformSurfelInPlace(Surfel &surfel, Transform3f const& transform);

/**
 * Transforms a surfel with a specified transform and returns the resulting surfel
 *
 * @param surfel
 * @param transform
 * @return transformed surfel
 */
Surfel transformSurfel(Surfel const& surfel, Transform3f const& transform);

/**
 * Transforms a set of surfels. Any surfel with doTransform false will have
 * the untransformed surfel in the result data structure.
 *
 * @param surfels
 * @param doTransform
 * @param transform
 * @param transformedSurfels
 */
void transformSurfels(std::vector<Surfel> const& surfels, std::vector<bool> const& doTransform,
                      Transform3f const& transform, std::vector<Surfel> &transformedSurfels);

/*
 *
 * Projecting surfels into the image plane
 *
 */

/**
 * Projects a single surfel (already in camera coordinates) into the camera
 * plane. The result is a fractional pixel location (x,y), which may or may
 * not be within bounds of the image resolution. A surfel behind the camera
 * gets a projection of (-1,-1)
 *
 * @param surfel
 * @param camParams
 * @return
 */
std::pair<float, float> projectSurfel(Surfel const& surfel, CameraParams const& camParams);

/**
 * Projects multiple surfels into the image plane
 *
 * @param surfels
 * @param camParams
 * @param projections
 */
void projectSurfels(std::vector<Surfel> const& surfels, CameraParams const& camParams,
                    std::vector<std::pair<float, float> > &projections);

/**
 * Projects multiple surfels in to the image plane. Same as above, but takes in
 * an argument specifying whether each surfel should be projected. Unprojected
 * surfels get a projection of (-1,-1)
 *
 * @param surfels
 * @param useSurfel
 * @param camParams
 * @param projections
 */
void projectSurfels(std::vector<Surfel> const& surfels, std::vector<bool> const& useSurfel,
                    CameraParams const& camParams, std::vector<std::pair<float, float> > &projections);

/**
 * Returns which (integer) pixel locations are covered by an estimated
 * projection of the surfel into the image plane.
 *
 * Note that in most cases this will be a conservative estimate in that this
 * may return more pixel locations than are actually covered.
 *
 * @param location
 * @param radius
 * @param camParams
 * @return
 */
std::vector<std::pair<int, int> > projectSurfelArea(Eigen::Vector3f location, float radius,
                                                    CameraParams const& camParams);

/*
 *
 * Indexing surfels by their projections
 *
 */

/**
 * Returns an xRes*yRes map (addressed as x+y*xRes), where each pixel location
 * has indices into the surfel list. These indicate the surfels that are near
 * the surfel. Typically, these are conservative estimates, and not every
 * surfel in the list may actually overlap the pixel
 *
 * If checknormal is true, then surfels which are facing away from the camera
 * will not be included in any of the lists
 *
 * @param surfels
 * @param camParams
 * @param resultMap
 * @param checkNormal
 */
void getNearbySurfelMap(std::vector<Surfel> const& surfels, CameraParams const& camParams,
                        std::vector<std::vector<unsigned int> > & resultMap, bool checkNormal = false);

/**
 * Same as other getNearbySurfelMap, but additionally takes a parameter indicating
 * whether each of the surfels should be included in the map
 *
 * @param surfels
 * @param useSurfel
 * @param camParams
 * @param resultMap
 * @param checkNormal
 */
void getNearbySurfelMap(std::vector<Surfel> const& surfels, std::vector<bool> const& useSurfel,
                        CameraParams const& camParams, std::vector<std::vector<unsigned int> > & resultMap,
                        bool checkNormal = false);

/*
 *
 * Surfel intersection tests
 *
 */

/**
 * Returns whether the specified line segment intersects the surfel. The
 * line segment is defined by its endpoints.
 *
 * @param pt1
 * @param pt2
 * @param surfel
 * @param intersection Intersection point if any
 * @return
 */
bool segmentIntersectsSurfel(Eigen::Vector3f const& pt1, Eigen::Vector3f const& pt2, Surfel const& surfel,
                             Eigen::Vector3f &intersection);

/*
 *
 * Less useful stuff, mainly used internally
 *
 */

/**
 * Update the surfel with the new observation. This assumes that inlier
 * checks and such have already been done, and the observation should actually
 * be integrated.
 *
 * @param surfel
 * @param observedLocation
 * @param viewedFrom
 * @param camLookDir
 * @param focalLength
 */
void addObservation(Surfel & surfel, Eigen::Vector3f const& observedLocation,
                    Eigen::Vector3f const& observedNormal, Eigen::Vector3f const& viewedFrom,
                    Eigen::Vector3f const& camLookDir, float focalLength);

/**
 * Creates a new surfel based on the observation and the camera params
 *
 * @param observedLocation
 * @param observedNormal
 * @param viewedFrom
 * @param camLookDir
 * @param focalLength
 * @return
 */
Surfel
       createSurfel(Eigen::Vector3f const& observedLocation, Eigen::Vector3f const& observedNormal,
                    Eigen::Vector3f const& viewedFrom, Eigen::Vector3f const& camLookDir, float focalLength);

void finalCleanup(SurfelModel &surfelModel, SurfelUpdateParams &params, std::vector<unsigned int> &removed);

/**
 * Computes the angle between the camera to pt line and the normal direction
 *
 * @param normal
 * @param pt
 * @param viewedFrom
 * @return angle
 */
float getAngleNormalToCamera(Eigen::Vector3f const& normal, Eigen::Vector3f const& pt,
                             Eigen::Vector3f const& viewedFrom);

/**
 * Get the angle between a surfel and the camera direction
 *
 * @param surfel
 * @param viewedFrom
 * @return
 */
float getAngleNormalToCamera(Surfel const& surfel, Eigen::Vector3f const& viewedFrom);

/**
 * Get the azimuth angle between a surfel and the camera direction
 *
 * @param surfel
 * @param viewedFrom
 * @return
 */
float getAzimuthAngle(Surfel const& surfel, Eigen::Vector3f const& viewedFrom);

/**
 * Returns the indices into observed_directions for the given viewing location
 *
 * @param surfel
 * @param viewedFrom
 * @return
 */
std::pair<unsigned int, unsigned int>
                                      getDirectionIndices(Surfel const& surfel, Eigen::Vector3f const& viewedFrom);

/**
 * Estimates the size that a patch would need to be so that it would
 * take up a single pixel in the image
 *
 * @param observedLocation
 * @param observedNormal
 * @param viewedFrom
 * @param camLookDir
 * @param focalLength
 * @return radius to use
 */
float getSurfelRadius(Eigen::Vector3f const& observedLocation, Eigen::Vector3f const& observedNormal,
                      Eigen::Vector3f const& viewedFrom, Eigen::Vector3f const& camLookDir,
                      float focalLength);

std::vector<unsigned int> removeSurfels(SurfelModel & surfelModel, std::vector<bool> const& shouldRemove);

void updateRefDirFromNormal(Surfel &surfel, Eigen::Vector3f const& oldNormal);

}

#endif /* SURFELS_H_ */
