/*
 * surfels.cpp
 *
 *  Created on: 2009-12-02
 *      Author: mkrainin
 */

#include "surfels.h"

#include <boost/range/concepts.hpp>
#include <boost/range.hpp>
#include <iostream>

namespace surfels
{

void addObservation(Surfel & surfel, Eigen::Vector3f const& observedLocation,
                    Eigen::Vector3f const& observedNormal, Eigen::Vector3f const& viewedFrom,
                    Eigen::Vector3f const& camLookDir, float focalLength)
{
  Eigen::Vector3f oldNormal = surfel.normal;

  //update location, normal, and total_views
  surfel.location = surfel.total_views * surfel.location + observedLocation;
  surfel.normal = surfel.total_views * surfel.normal + observedNormal;
  surfel.total_views++;
  surfel.location /= surfel.total_views;
  surfel.normal.normalize();

  //update reference_dir according to the update of the normal
  updateRefDirFromNormal(surfel, oldNormal);

  //update the radius
  float newRadius = getSurfelRadius(surfel.location, surfel.normal, viewedFrom, camLookDir, focalLength);
  if (newRadius < surfel.radius)
    surfel.radius = newRadius;

  //update observed_directions and visibility_confidence
  std::pair<unsigned int, unsigned int> dir = getDirectionIndices(surfel, viewedFrom);
  if (!surfel.observed_directions[dir.first][dir.second])
  {
    surfel.observed_directions[dir.first][dir.second] = true;
    surfel.visibility_confidence++;
  }
}

Surfel createSurfel(Eigen::Vector3f const& observedLocation, Eigen::Vector3f const& observedNormal,
                    Eigen::Vector3f const& viewedFrom, Eigen::Vector3f const& camLookDir, float focalLength)
{
  Surfel newSurfel;
  newSurfel.location = observedLocation;
  newSurfel.normal = observedNormal.normalized();
  newSurfel.radius = getSurfelRadius(observedLocation, observedNormal, viewedFrom, camLookDir, focalLength);

  //reference dir is arbitrary but must be perpendicular to normal
  Eigen::Vector3f nonParallel = Eigen::Vector3f::UnitX();
  if (fabs(observedNormal.dot(nonParallel)) > .9)
    nonParallel = Eigen::Vector3f::UnitY();
  newSurfel.reference_dir = observedNormal.cross(nonParallel).normalized();

  //initialize viewing directions
  std::pair<unsigned int, unsigned int> dir = getDirectionIndices(newSurfel, viewedFrom);
  newSurfel.observed_directions[dir.first][dir.second] = true;
  newSurfel.visibility_confidence = 1;
  newSurfel.total_views = 1;

  return newSurfel;

}

float getAngleNormalToCamera(Eigen::Vector3f const& normal, Eigen::Vector3f const& pt,
                             Eigen::Vector3f const& viewedFrom)
{
  float dotProduct = (viewedFrom - pt).normalized().dot(normal);
  if (fabs(dotProduct) > 1)
    dotProduct /= fabs(dotProduct);
  return acos(dotProduct);
}

float getAngleNormalToCamera(Surfel const& surfel, Eigen::Vector3f const& viewedFrom)
{
  return getAngleNormalToCamera(surfel.normal, surfel.location, viewedFrom);
}

float getAzimuthAngle(Surfel const& surfel, Eigen::Vector3f const& viewedFrom)
{
  Eigen::Vector3f toViewpoint = viewedFrom - surfel.location;

  float dotProduct = toViewpoint.normalized().dot(surfel.reference_dir);
  if (fabs(dotProduct) > 1)
    dotProduct /= fabs(dotProduct);
  float invCos = acos(dotProduct);

  //invCos is between 0 and pi. expand to be from 0 to 2*pi
  Eigen::Vector3f refDir2 = surfel.normal.cross(surfel.reference_dir);
  if (toViewpoint.dot(refDir2) < 0)
    invCos = 2 * M_PI - invCos;

  return invCos;
}

std::pair<unsigned int, unsigned int> getDirectionIndices(Surfel const& surfel, Eigen::Vector3f const& viewedFrom)
{
  float polarAngle = getAngleNormalToCamera(surfel, viewedFrom);
  float azimuthAngle = getAzimuthAngle(surfel, viewedFrom);

  //shouldn't be the case, but make it well defined anyways
  if (polarAngle > M_PI_2)
  {
    //flip polar angle about equator
    polarAngle = M_PI - polarAngle;
    //move azimuth angle to opposite side
    if (azimuthAngle < M_PI)
      azimuthAngle += M_PI;
    else
      azimuthAngle -= M_PI;
  }

  unsigned int azIndex = ((int)((azimuthAngle / (2 * M_PI)) * num_azimuth_angles)) % num_azimuth_angles;
  unsigned int polarIndex = ((int)((polarAngle / M_PI_2) * num_polar_angles)) % num_polar_angles;
  return std::pair<unsigned int, unsigned int>(azIndex, polarIndex);
}

void updateRefDirFromNormal(Surfel &surfel, Eigen::Vector3f const& oldNormal)
{
  float dot = oldNormal.dot(surfel.normal);
  if (fabs(dot) > 1)
    dot /= fabs(dot);
  float angle = acos(dot);
  Eigen::Vector3f rotationAxis = oldNormal.cross(surfel.normal).normalized();
  surfel.reference_dir = Eigen::AngleAxisf(angle, rotationAxis) * surfel.reference_dir;
  surfel.reference_dir = (surfel.reference_dir - surfel.normal * (surfel.reference_dir.dot(surfel.normal)));
  surfel.reference_dir.normalize();
}

void getNearbySurfelMap(std::vector<Surfel> const& surfels, CameraParams const& camParams,
                        std::vector<std::vector<unsigned int> > & resultMap, bool checkNormal)
{
  getNearbySurfelMap(surfels, std::vector<bool>(surfels.size(), true), camParams, resultMap, checkNormal);
}

void getNearbySurfelMap(std::vector<Surfel> const& surfels, std::vector<bool> const& useSurfel,
                        CameraParams const& camParams, std::vector<std::vector<unsigned int> > & resultMap,
                        bool checkNormal)
{
  resultMap.clear();
  resultMap.resize(camParams.xRes * camParams.yRes);

  //add each surfel index to the lists of the nearby pixels
#pragma omp parallel for schedule (dynamic, 10)
  for (unsigned int i = 0; i < surfels.size(); i++)
  {
    if (!useSurfel[i])
      continue;

    if (checkNormal && surfels[i].normal.z() > 0)
      continue;

    std::vector<std::pair<int, int> > area = projectSurfelArea(surfels[i].location, surfels[i].radius, camParams);
    for (unsigned int j = 0; j < area.size(); j++)
    {
      int pixel = area[j].first + area[j].second * camParams.xRes;
#pragma omp critical
      {
      resultMap[pixel].push_back(i);
      }
    }
  }
}

float getSurfelRadius(Eigen::Vector3f const& observedLocation, Eigen::Vector3f const& observedNormal,
                      Eigen::Vector3f const& viewedFrom, Eigen::Vector3f const& camLookDir,
                      float focalLength)
{
  Eigen::Vector3f surfelToCam = viewedFrom - observedLocation;
  float z = fabs(surfelToCam.dot(camLookDir));
  float normalDot = fabs(camLookDir.dot(observedNormal));
  if (normalDot < .3)
    normalDot = .3;
  return (M_SQRT1_2 * z / focalLength) / normalDot;
}

void transformSurfels(std::vector<Surfel> const& surfels, std::vector<bool> const& doTransform,
                      Transform3f const& transform, std::vector<Surfel> &transformedSurfels)
{
  transformedSurfels.clear();
  transformedSurfels.reserve(surfels.size());
  for (unsigned int i = 0; i < surfels.size(); i++)
  {
    if (doTransform[i])
      transformedSurfels.push_back(transformSurfel(surfels[i], transform));
    else
      transformedSurfels.push_back(surfels[i]);
  }
}

bool interpolateGrid(std::pair<float, float> const& pixel,
                     boost::multi_array<Eigen::Vector3f, 2> const& dataGrid,
                     boost::multi_array<Eigen::Vector3f, 2> const& depthGrid,
                     boost::multi_array<bool, 2> const& validityGrid, Eigen::Vector3f & result,
                     float maxDepthDiff)
{
  result = Eigen::Vector3f::Zero();

  unsigned int xRes = dataGrid.size();
  unsigned int yRes = dataGrid[0].size();

  if (pixel.first < 0 || pixel.first >= (float)xRes - 1 || pixel.second < 0 || pixel.second >= (float)yRes - 1)
    return false;

  float x = pixel.first;
  float y = pixel.second;
  int x_floor = (int)x;
  int y_floor = (int)y;

  if (x_floor < 0 || x_floor + 1 >= (int)xRes || y_floor < 0 || y_floor + 1 >= (int)yRes)
    return false;

  //bilinear interpolation
  if (validityGrid[x_floor][y_floor] && validityGrid[x_floor][y_floor + 1] && validityGrid[x_floor + 1][y_floor]
      && validityGrid[x_floor + 1][y_floor + 1])
  {
    //make sure we're not trying to interpolate over too large a depth difference
    float depthCC = depthGrid[x_floor + 1][y_floor + 1].z();
    float depthCF = depthGrid[x_floor + 1][y_floor].z();
    float depthFC = depthGrid[x_floor][y_floor + 1].z();
    float depthFF = depthGrid[x_floor][y_floor].z();
    if (fabs(depthCC - depthCF) > maxDepthDiff || fabs(depthCC - depthFC) > maxDepthDiff || fabs(depthCC - depthFF)
        > maxDepthDiff)
      return false;

    //between 0 and 1
    float x_resid = x - x_floor;
    float y_resid = y - y_floor;
    //C for ceiling, F for floor
    float coefCC = x_resid * y_resid;
    float coefCF = x_resid * (1 - y_resid);
    float coefFC = (1 - x_resid) * y_resid;
    float coefFF = (1 - x_resid) * (1 - y_resid);

    result = coefFF * dataGrid[x_floor][y_floor] + coefFC * dataGrid[x_floor][y_floor + 1] + coefCF * dataGrid[x_floor
        + 1][y_floor] + coefCC * dataGrid[x_floor + 1][y_floor + 1];
    return true;
  }
  else
  {
    return false;
  }
}

Eigen::Vector3f getColorVector(cv::Mat const& img, unsigned int x, unsigned int y)
{
  cv::Vec3b const& pix = img.at<cv::Vec3b>(y,x);
  return Eigen::Vector3f(pix[2], pix[1], pix[0]) / 255.0;
}

void interpolateImagePixel(std::pair<float, float> const& pixel, cv::Mat const& img, Eigen::Vector3f & result)
{
  float x = pixel.first;
  float y = pixel.second;
  int x_floor = (int)x;
  int y_floor = (int)y;

  //between 0 and 1
  float x_resid = x - x_floor;
  float y_resid = y - y_floor;
  //C for ceiling, F for floor
  float coefCC = x_resid * y_resid;
  float coefCF = x_resid * (1 - y_resid);
  float coefFC = (1 - x_resid) * y_resid;
  float coefFF = (1 - x_resid) * (1 - y_resid);

  result = coefFF * getColorVector(img, x_floor, y_floor) + coefFC * getColorVector(img, x_floor, y_floor + 1) + coefCF
      * getColorVector(img, x_floor + 1, y_floor) + coefCC * getColorVector(img, x_floor + 1, y_floor + 1);

}

bool interpolatePointAndNormal(std::pair<float, float> const& pixel,
                               boost::multi_array<Eigen::Vector3f, 2> const& pointGrid,
                               boost::multi_array<Eigen::Vector3f, 2> const& normalGrid,
                               boost::multi_array<bool, 2> const& validityGrid, Eigen::Vector3f &interpPoint,
                               Eigen::Vector3f &interpNormal, float maxDepthDiff)
{
  interpPoint = Eigen::Vector3f::Zero();
  interpNormal = Eigen::Vector3f::Zero();

  bool success = interpolateGrid(pixel, pointGrid, pointGrid, validityGrid, interpPoint, maxDepthDiff)
      && interpolateGrid(pixel, normalGrid, pointGrid, validityGrid, interpNormal, maxDepthDiff);
  if (success)
    interpNormal.normalize();

  return success;
}

std::pair<float, float> projectSurfelLocation(Eigen::Vector3f const& location, CameraParams const& camParams)
{
  if (location.z() < 0)
    return std::pair<float, float>(-1, -1);

  float x = location.x() / location.z() * camParams.focalLength + camParams.centerX;
  float y = location.y() / location.z() * camParams.focalLength + camParams.centerY;
  return std::pair<float, float>(x, y);
}

std::pair<float, float> projectSurfel(Surfel const& surfel, CameraParams const& camParams)
{
  return projectSurfelLocation(surfel.location, camParams);
}

void projectSurfels(std::vector<Surfel> const& surfels, CameraParams const& camParams,
                    std::vector<std::pair<float, float> > &projections)
{
  projectSurfels(surfels, std::vector<bool>(surfels.size(), true), camParams, projections);
}

void projectSurfels(std::vector<Surfel> const& surfels, std::vector<bool> const& useSurfel,
                    CameraParams const& camParams, std::vector<std::pair<float, float> > &projections)
{
  projections.clear();
  projections.resize(surfels.size(), std::pair<float, float>(-1, -1));
#pragma omp parallel for schedule (dynamic, 10)
  for (unsigned int i = 0; i < surfels.size(); i++)
  {
    if (!useSurfel[i])
      continue;
    projections[i] = projectSurfel(surfels[i], camParams);
  }
}

std::vector<std::pair<int, int> > projectSurfelArea(Eigen::Vector3f location, float radius,
                                                    CameraParams const& camParams)
{
  std::vector<std::pair<int, int> > toReturn;

  if (location.z() < 0)
    return toReturn;

  std::pair<float, float> projection = projectSurfelLocation(location, camParams);
  float circleCenterX = projection.first;
  float circleCenterY = projection.second;

  //estimate surfel as being faced directly at the camera for simplicity
  float circleRadius = radius / location.z() * camParams.focalLength;

  //use the actual radius if it's very large in the image, otherwise,
  //just take the 4 surrounding pixels
  if (circleRadius > 1)
  {
    float sqRadius = pow(circleRadius, 2);

    //establish bounds on what pixel locations could be in the projected surfel
    int minX = floor(circleCenterX) - ceil(circleRadius);
    int maxX = ceil(circleCenterX) + ceil(circleRadius);
    int minY = floor(circleCenterY) - ceil(circleRadius);
    int maxY = ceil(circleCenterY) + ceil(circleRadius);

    //check that some part of the circle could be in the window
    if (minX >= (int)camParams.xRes || maxX < 0 || minY >= (int)camParams.yRes || maxY < 0)
      return toReturn;

    //put the bounds within the window bounds
    if (minX < 0)
      minX = 0;
    if (maxX >= (int)camParams.xRes)
      maxX = camParams.xRes - 1;
    if (minY < 0)
      minY = 0;
    if (maxY >= (int)camParams.yRes)
      maxY = camParams.yRes - 1;

    //check which ones are actually within the radius
    for (int x = minX; x <= maxX; x++)
    {
      for (int y = minY; y <= maxY; y++)
      {
        float dx = x - circleCenterX;
        float dy = y - circleCenterY;
        float sqDist = pow(dx, 2) + pow(dy, 2);
        if (sqDist <= sqRadius)
          toReturn.push_back(std::pair<int, int>(x, y));
      }
    }
  }

  else
  {
    if (circleCenterX >= 0 && circleCenterX < camParams.xRes - 1 && circleCenterY >= 0 && circleCenterY
        < camParams.yRes - 1)
    {
      unsigned int x = floor(circleCenterX);
      unsigned int y = floor(circleCenterY);
      toReturn.push_back(std::pair<int, int>(x, y));
      toReturn.push_back(std::pair<int, int>(x, y + 1));
      toReturn.push_back(std::pair<int, int>(x + 1, y));
      toReturn.push_back(std::pair<int, int>(x + 1, y + 1));
    }
  }

  return toReturn;
}

bool segmentIntersectsSurfel(Eigen::Vector3f const& pt1, Eigen::Vector3f const& pt2, Surfel const& surfel,
                             Eigen::Vector3f &intersection)
{

  //check that the endpoints are on different sides of the plane
  Eigen::Vector3f diff1 = pt1 - surfel.location;
  Eigen::Vector3f diff2 = pt2 - surfel.location;
  float dot1 = diff1.dot(surfel.normal);
  float dot2 = diff2.dot(surfel.normal);
  if (dot1 * dot2 > 0 || (dot1 == 0 && dot2 == 0))
    return false;

  //figure out where it intersects the plane
  float interpolationRate = fabs(dot1) / (fabs(dot1) + fabs(dot2));
  intersection = (1 - interpolationRate) * pt1 + interpolationRate * pt2;
  return (intersection - surfel.location).norm() < surfel.radius;
}

void transformSurfelInPlace(Surfel &surfel, Transform3f const& transform)
{
  surfel.location = transform * surfel.location;
  surfel.normal = transform.linear() * surfel.normal;
  surfel.reference_dir = transform.linear() * surfel.reference_dir;
}

Surfel transformSurfel(Surfel const& surfel, Transform3f const& transform)
{
  Surfel toReturn = surfel;
  toReturn.location = transform * surfel.location;
  toReturn.normal = transform.linear() * surfel.normal;
  toReturn.reference_dir = transform.linear() * surfel.reference_dir;

  return toReturn;
}

/**
 * Helper for updateSurfelModel. Just the update part, not the addition or removal.
 */
void updateSurfels(SurfelModel &model, SurfelUpdateData &data, SurfelUpdateParams &params,
                   std::vector<bool> const& canUpdateSurfel, std::vector<std::pair<float, float> > const& projections,
                   std::vector<bool> &shouldRemove)
{
  //camera at 0 looking in z direction
  Eigen::Vector3f viewedFrom = Eigen::Vector3f::Zero();
  Eigen::Vector3f lookDir = Eigen::Vector3f::UnitZ();

  float minDotWithExistingNormal = cos(params.maxNormalAngleDiff);

  //loop through surfel range, performing updates
#pragma omp parallel for schedule (dynamic, 10)
  for (unsigned int i = 0; i < model.surfels.size(); i++)
  {
    //skip
    if (!canUpdateSurfel[i] || shouldRemove[i])
      continue;

    float normalAngle = getAngleNormalToCamera(model.surfels[i], viewedFrom);
    std::pair<float, float> pixel = projections[i];

    //get interpolated readings (point and normal) for the projection location
    Eigen::Vector3f corrPoint, corrNormal;
    bool interpolationSuccess = interpolatePointAndNormal(pixel, *data.pointGrid, *data.normalGrid, *data.validityGrid,
                                                          corrPoint, corrNormal, params.maxInterpolationDist);

    //make sure there is something to project to
    if (interpolationSuccess)
    {
      int x = round(pixel.first);
      int y = round(pixel.second);

      //check its distance to correspondence
      float depthDiff = model.surfels[i].location.dot(lookDir) - corrPoint.dot(lookDir);

      //within range
      if (fabs(depthDiff) <= params.corrDistForUpdate)
      {
        float corrNormalAngle = getAngleNormalToCamera(corrNormal, corrPoint, viewedFrom);
        bool willingToAddPixel = (*data.willingToAdd)[x][y] && corrNormalAngle < params.maxNormalAngle;

        //if the normals disagree by too much, consider the existing surfel erroneous
        float normalsDot = model.surfels[i].normal.dot(corrNormal);
        if (corrNormalAngle < params.maxNormalAngle && model.surfels[i].visibility_confidence < params.highConfidence
            && normalsDot > -1 * minDotWithExistingNormal && normalsDot < minDotWithExistingNormal)
        {
          shouldRemove[i] = true;
          continue;
        }

        //make sure we're willing to incorporate the new measurement
        if (normalAngle > params.maxNormalAngle || !willingToAddPixel)
          continue;

        //perform the update
        addObservation(model.surfels[i], corrPoint, corrNormal, viewedFrom, lookDir, data.camParams.focalLength);

        //color selection
        if (params.colorUpdateRule == COLOR_UPDATE_BEST_VIEWPOINT)
        {
          float newAngle = getAngleNormalToCamera(model.surfels[i], viewedFrom);
          if (newAngle < model.minAngleFromNormal[i])
          {
            Eigen::Vector3f pixelColor;
            interpolateImagePixel(pixel, data.image, pixelColor);
            model.colors[i] = pixelColor;
            model.minAngleFromNormal[i] = newAngle;
          }
        }
        else if (params.colorUpdateRule == COLOR_UPDATE_MEDIAN_COLOR)
        {
          Eigen::Vector3f pixelColor;
          interpolateImagePixel(pixel, data.image, pixelColor);
          model.allColors[i].insert(pixelColor);
          std::multiset<Eigen::Vector3f, colorcomp>::iterator it = model.allColors[i].begin();
          unsigned int middleElement = model.allColors[i].size() / 2;
          for (unsigned int j = 0; j < middleElement; j++)
            it++;
          model.colors[i] = *it;
        }
        else if (params.colorUpdateRule == COLOR_UPDATE_FIRST_COLOR)
        {
          // don't do anything
        }
        else if (params.colorUpdateRule == COLOR_UPDATE_LAST_COLOR)
        {
          Eigen::Vector3f pixelColor;
          interpolateImagePixel(pixel, data.image, pixelColor);
          model.colors[i] = pixelColor;
        }
        else if (params.colorUpdateRule == COLOR_UPDATE_NEAREST)
        {
          if (depthDiff > 0.0)
          {
            Eigen::Vector3f pixelColor;
            interpolateImagePixel(pixel, data.image, pixelColor);
            model.colors[i] = pixelColor;
            model.minDistanceSeen[i] = model.surfels[i].location.dot(lookDir);
          }
        }
        else
        {
          assert(false && "unknown color update rule");
        }

        model.lastSeen[i] = data.currentTime;
      }
      else if (depthDiff < -1 * params.corrDistForUpdate)
      {
        //saw through the surfel. if surfel confidence is low, remove it
        if (model.surfels[i].visibility_confidence < params.highConfidence)
        {
          shouldRemove[i] = true;
        }
        //else do nothing
      }
      else
      {
        //There was a rule here for removing surfels which are occluded but
        //should have been visible. If surfels show up on the interior,
        //this rule can be put back in, but it didn't seem necessary
      } //end scan in front of surfel case
    } //end if could interpolate
  }

}

/*
 * toRemove: indices into vec
 *
 * optimized for a relatively small number of removals
 *
 * pre: toRemove is sorted ascending
 */
template<typename RandomAccessRangeT>
std::vector<typename boost::range_value<RandomAccessRangeT>::type> selectAllBut(const RandomAccessRangeT& vec, std::vector<unsigned int> const& toRemove)
{
        BOOST_CONCEPT_ASSERT((boost::RandomAccessRangeConcept<RandomAccessRangeT>));

        std::vector<typename boost::range_value<RandomAccessRangeT>::type> result(boost::size(vec) - toRemove.size());
        if(toRemove.empty())
        {
                std::copy(boost::begin(vec), boost::end(vec), result.begin());
        }
        else
        {
                for(unsigned int i = 0; i < toRemove.size() - 1; i++) assert(toRemove[i] < toRemove[i + 1]);
                assert(toRemove.back() <= (unsigned int)boost::size(vec));

                typename std::vector<typename boost::range_value<RandomAccessRangeT>::type>::iterator j = result.begin();
                std::copy(boost::begin(vec), boost::begin(vec) + toRemove[0], j);
                j += toRemove[0];
                for(unsigned int i = 0; i < toRemove.size() - 1; i++)
                {
                        std::copy(boost::begin(vec) + toRemove[i] + 1, boost::begin(vec) + toRemove[i + 1], j);
                        j += toRemove[i + 1] - toRemove[i] - 1;
                }
                std::copy(boost::begin(vec) + toRemove.back() + 1, boost::end(vec), j);
        }
        return result;
}

std::vector<unsigned int> removeSurfels(SurfelModel & surfelModel, std::vector<bool> const& shouldRemove)
{
  std::vector<unsigned int> removed;
  for(unsigned int i=0; i<shouldRemove.size(); i++)
    if(shouldRemove[i])
      removed.push_back(i);

  surfelModel.surfels = selectAllBut(surfelModel.surfels, removed);
  surfelModel.colors = selectAllBut(surfelModel.colors, removed);
  surfelModel.lastSeen = selectAllBut(surfelModel.lastSeen, removed);
  // These (at least) can be empty depending on color update rule
  if (!surfelModel.allColors.empty())
  {
    surfelModel.allColors = selectAllBut(surfelModel.allColors, removed);
  }
  if (!surfelModel.minAngleFromNormal.empty())
  {
    surfelModel.minAngleFromNormal = selectAllBut(surfelModel.minAngleFromNormal, removed);
  }
  if (!surfelModel.minDistanceSeen.empty())
  {
    surfelModel.minDistanceSeen = selectAllBut(surfelModel.minDistanceSeen, removed);
  }

  return removed;
}

void updateSurfelModel(SurfelModel &surfelModel, SurfelUpdateData &data, SurfelUpdateParams &params,
                       std::vector<bool> &canUpdateSurfel, std::vector<unsigned int> &removed)
{
  std::vector<Surfel> &surfels = surfelModel.surfels;

  Transform3f identity;
  identity.setIdentity();
  bool transformIsIdentity = data.camTransform.isApprox(identity);

  //transform all of the surfels to camera coordinates
  if (!transformIsIdentity)
  {
    Transform3f inverseCamTrans = data.camTransform.inverse(Eigen::Isometry);
#pragma omp parallel for schedule (dynamic, 10)
    for (size_t i = 0; i < surfels.size(); i++)
    {
      if (canUpdateSurfel[i])
        transformSurfelInPlace(surfels[i], inverseCamTrans);
    }
  }

  //get the projections of the existing surfels onto the image plane
  std::vector<std::pair<float, float> > projections;
  projectSurfels(surfelModel.surfels, canUpdateSurfel, data.camParams, projections);

  //update existing surfels
  std::vector<bool> shouldRemove(surfels.size(), false);
  updateSurfels(surfelModel,data,params,canUpdateSurfel,projections,shouldRemove);

  //get a pixel-indexed structure for the existing surfels
  std::vector<std::vector<unsigned int> > nearbySurfels;
  getNearbySurfelMap(surfels, canUpdateSurfel, data.camParams, nearbySurfels, true);

  //camera is at 0 and looking in the z direction
  Eigen::Vector3f viewedFrom = Eigen::Vector3f::Zero();
  Eigen::Vector3f lookDir = Eigen::Vector3f::UnitZ();

  //add new surfels
  unsigned int numAdded = 0;
  //should probably be its own param
  float minDistInFront = params.maxInterpolationDist;
#pragma omp parallel for schedule (dynamic, 10)
  for (unsigned int x = 0; x < data.camParams.xRes; x++)
  {
    for (unsigned int y = 0; y < data.camParams.yRes; y++)
    {
      if (!(*data.willingToAdd)[x][y])
        continue;

      Surfel potentialSurfel = createSurfel((*data.pointGrid)[x][y], (*data.normalGrid)[x][y], viewedFrom, lookDir,
                                            data.camParams.focalLength);

      //check orientation wrt camera
      float angleFromCamera = getAngleNormalToCamera(potentialSurfel, viewedFrom);
      if (angleFromCamera > params.maxNormalAngle)
        continue;

      //check for an existing surfel explaining the measurement
      bool dontAdd = false;
      Eigen::Vector3f intersection;
      for (unsigned int j = 0; j < nearbySurfels[x + data.camParams.xRes * y].size(); j++)
      {
        unsigned int index = nearbySurfels[x + data.camParams.xRes * y][j];

        if (shouldRemove[index])
          continue;

        float zDiff = surfels[index].location.z() - potentialSurfel.location.z();
        //can't be behind
        bool behind = segmentIntersectsSurfel(Eigen::Vector3f::Zero(), potentialSurfel.location, surfels[index],
                                              intersection);
        if (behind)
        {
          dontAdd = true;
          break;
        }
        //can be in front, but must be some minimum distance in front
        bool inFront = segmentIntersectsSurfel(Eigen::Vector3f::Zero(), surfels[index].location, potentialSurfel,
                                               intersection);
        if (inFront && zDiff < minDistInFront)
        {
          dontAdd = true;
          break;
        }
      }

      //add the new surfel
      if (!dontAdd)
      {
        Eigen::Vector3f pixelColor = getColorVector(data.image, x, y);

#pragma omp critical
        {
          surfels.push_back(potentialSurfel);
          surfelModel.colors.push_back(pixelColor);
          surfelModel.lastSeen.push_back(data.currentTime);
          if (params.colorUpdateRule == COLOR_UPDATE_MEDIAN_COLOR)
          {
            surfelModel.allColors.push_back(std::multiset<Eigen::Vector3f, colorcomp>());
            surfelModel.allColors.back().insert(pixelColor);
          }
          else if (params.colorUpdateRule == COLOR_UPDATE_BEST_VIEWPOINT)
          {
            surfelModel.minAngleFromNormal.push_back(angleFromCamera);
          }
          else if (params.colorUpdateRule == COLOR_UPDATE_NEAREST)
          {
            surfelModel.minDistanceSeen.push_back(potentialSurfel.location.z());
          }
          canUpdateSurfel.push_back(true);
          shouldRemove.push_back(false);
          projections.push_back(std::pair<float, float>(x, y));

          numAdded++;
        }//critical
      }// add
    }// y
  }// x
  //std::cout<<"Added "<<numAdded<<" surfels"<<std::endl;

  //check for starvation removal
  unsigned int totalSurfels = surfels.size();
#pragma omp parallel for schedule (dynamic, 10)
  for (unsigned int origIndex = 0; origIndex < totalSurfels; origIndex++)
  {
    bool timeRemove = (data.currentTime - surfelModel.lastSeen[origIndex] > params.timeDiffForRemoval)
        && (surfels[origIndex].visibility_confidence < params.starvationConfidence);
    if (timeRemove)
      shouldRemove[origIndex] = true;
  }

  //removal
  removed = removeSurfels(surfelModel, shouldRemove);
  std::vector<bool> canUpdateCopy = canUpdateSurfel;
  canUpdateSurfel.clear();
  for(unsigned int i=0; i<canUpdateCopy.size(); i++){
    if(!shouldRemove[i]){
      canUpdateSurfel.push_back(canUpdateCopy[i]);
    }
  }
  //std::cout<<"Removed "<<removed.size()<<" surfels"<<std::endl;

  //transform surfels back to original coordinate system
  if (!transformIsIdentity)
  {
#pragma omp parallel for schedule (dynamic, 10)
    for (unsigned int i = 0; i < surfels.size(); i++)
    {
      if (canUpdateSurfel[i])
        transformSurfelInPlace(surfels[i], data.camTransform);
    }
  }
}

void finalCleanup(SurfelModel &surfelModel, SurfelUpdateParams &params, std::vector<unsigned int> &removed)
{
  //get rid of any low confidence surfels
  std::vector<bool> shouldRemove(surfelModel.surfels.size(), false);
  for (unsigned int i = 0; i < shouldRemove.size(); i++)
    if (surfelModel.surfels[i].visibility_confidence < params.starvationConfidence)
      shouldRemove[i] = true;
  removed = removeSurfels(surfelModel, shouldRemove);
}

} //namespace surfels

