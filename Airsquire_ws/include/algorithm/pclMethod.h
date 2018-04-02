

#ifndef AIRSQUIRE_WS_PCLMETHOD_H
#define AIRSQUIRE_WS_PCLMETHOD_H

#include "general/PointTypes.h"


namespace interview {

/**
 * \class pclMethod
 * \brief This class extracts cylinder shapes from point clouds.
 * \details It detects planar surfaces according to their normals and removes them as outliers. Then tries to fit a
 * cylinder shape surface to remaining of the points.
 */
  class pclMethod {

  private:

    CloudColorPtrType cloud;

  public:
    explicit pclMethod(CloudColorPtrType &inputCloud);

    /** \brief Removes NaN and out of limits values from imported point in the specified axis cloud.
          *
          * \param[in] input is the container consisting of the input Points.
          * \param[in] minLimit is the lowest limit value of the points.
          * \param[in] maxLimit is the highest limit value of the points.
          * \param[in] axis defines the axis which applies minLimits and maxLimits.
          * \param[out] FilteredCloud is the container consisting of the removed NaN and out of limits Points.
          */
    CloudColorPtrType
    passThroughFilter(const CloudColorPtrType input, double minLimit, double maxLimit, std::string axis);


    /** \brief Estimates normals of each point according to its neighbors
          *
          * \param[in] input is the container consisting of the input Points.
          * \param[in] neighbors is the number of K nearest neighbors.
          * \param[out] FilteredCloud is the container consisting of the normals of the Points.
          */
    NCloudPtrType estimateNormals(const CloudColorPtrType &input, int neighbors);

    /** \brief Segments all possible points into plannar and cylinder categories
          *
          * \param[in] input is the container consisting of the input Points.
          * \param[in] normals is the container consisting of the normals of the input Points.
          * \param[in] PlaneDistanceThreshold is the threshold to fit the plane.
          * \param[in] CylinderDistanceThreshold is the threshold to fit the cylinder.
          * \param[in] NormalDistanceWeight is the weight of normal vectors.
          * \param[in] minRadius is the minimum radius of the interest cylinder.
          * \param[in] maxRadius is the maximum radius of the interest cylinder.
          * \param[out] CylinderCloud is the container consisting of the cylinder Points.
          */
    CloudColorPtrType segmentation(const CloudColorPtrType &input, const NCloudPtrType &normals,
                                   double PlaneDistanceThreshold, double CylinderDistanceThreshold,
                                   double NormalDistanceWeight, double minRadius, double maxRadius);

    /** \brief extracts cylinder shapes from point clouds.
          *
          * \param[in] filterMinLimit defines the limit to filter points which have lower than this value.
          * \param[in] filterMaxLimit defines the limit to filter points which have higher than this value.
          * \param[in] filterAxis defines the axis which applies filterMinLimit and filterMaxLimit.
          * \param[in] NormalNeighbors is the number of K nearest neighbors.
          * \param[in] fitPlaneDistanceThreshold is the threshold to fit the plane.
          * \param[in] fitCylinderDistanceThreshold is the threshold to fit the cylinder.
          * \param[in] fitNormalDistanceWeight is the weight of normal vectors.
          * \param[in] CylinderMinRadius is the minimum radius of the interest cylinder.
          * \param[in] CylinderMaxRadius is the maximum radius of the interest cylinder.
          * \param[out] CylinderCloud is the container consisting of the cylinder Points.
          *
          */
    CloudColorPtrType findCylinder(double filterMinLimit = 0, double filterMaxLimit = 1.5, std::string filterAxis = "z",
                                   int NormalNeighbors = 50, double fitPlaneDistanceThreshold = 0.03,
                                   double fitCylinderDistanceThreshold = 0.05, double fitNormalDistanceWeight = 0.1,
                                   double CylinderMinRadius = 0, double CylinderMaxRadius = 0.1);

  };
}


#endif //AIRSQUIRE_WS_PCLMETHOD_H
