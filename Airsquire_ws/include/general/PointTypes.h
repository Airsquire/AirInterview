
#ifndef AIRSQUIRE_WS_POINTTYPES_H
#define AIRSQUIRE_WS_POINTTYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace interview {
  typedef pcl::PointXYZI PointType;
  typedef pcl::PointCloud<PointType> CloudType;
  typedef CloudType::Ptr CloudPtrType;
  typedef CloudType::ConstPtr CloudConstPtrType;


  typedef pcl::PointXYZRGB PointColorType;
  typedef pcl::PointCloud<PointColorType> CloudColorType;
  typedef CloudColorType::Ptr CloudColorPtrType;


  typedef pcl::search::KdTree<PointType> KdTreeType;
  typedef KdTreeType::Ptr KdTreePtrType;

  typedef pcl::search::KdTree<PointColorType> KdTreeColorType;
  typedef KdTreeColorType::Ptr KdTreePtrColorType;

  typedef pcl::Normal NormalType;
  typedef pcl::PointCloud<NormalType> NCloudType;
  typedef NCloudType::Ptr NCloudPtrType;
  typedef NCloudType::ConstPtr NCloudConstPtrType;



  typedef pcl::NormalEstimation<PointColorType, NormalType> NormalEstimator;
  typedef pcl::SACSegmentationFromNormals<PointColorType, NormalType> SACSegmentor;
  typedef pcl::ExtractIndices<PointColorType> PointIndiceExtractor;
  typedef pcl::ExtractIndices<NormalType> NormalIndiceExtractor;


}

#endif //AIRSQUIRE_WS_POINTTYPES_H
