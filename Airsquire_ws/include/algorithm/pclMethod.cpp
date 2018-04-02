
#include "pclMethod.h"

namespace interview {
  pclMethod::pclMethod(CloudColorPtrType &inputCloud) : cloud(inputCloud) {}


  CloudColorPtrType
  pclMethod::passThroughFilter(const CloudColorPtrType input, double minLimit, double maxLimit, std::string axis) {
    CloudColorPtrType output(new CloudColorType);
    pcl::PassThrough<PointColorType>::Ptr pass(new pcl::PassThrough<PointColorType>);
    pass->setInputCloud(input);
    pass->setFilterFieldName(axis);
    pass->setFilterLimits(minLimit, maxLimit);
    pass->filter(*output);
    return output;
  }


  NCloudPtrType pclMethod::estimateNormals(const CloudColorPtrType &input, int neighbors) {


    NCloudPtrType cloud_normals(new NCloudType);
    NormalEstimator::Ptr ne(new NormalEstimator);
    KdTreePtrColorType tree(new KdTreeColorType);
    ne->setSearchMethod(tree);
    ne->setInputCloud(input);
    ne->setKSearch(neighbors);
    ne->compute(*cloud_normals);
    return cloud_normals;
  }

  CloudColorPtrType pclMethod::segmentation(const CloudColorPtrType &input, const NCloudPtrType &normals,
                                            double PlaneDistanceThreshold, double CylinderDistanceThreshold,
                                            double NormalDistanceWeight, double minRadius, double maxRadius) {

    SACSegmentor seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(NormalDistanceWeight);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(PlaneDistanceThreshold);
    seg.setInputCloud(input);
    seg.setInputNormals(normals);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr temp(new pcl::ModelCoefficients);

    seg.segment(*inliers, *temp);


    PointIndiceExtractor::Ptr extract(new PointIndiceExtractor);
    extract->setInputCloud(input);
    extract->setIndices(inliers);

    CloudColorPtrType cloud_filtered(new CloudColorType);

    NCloudPtrType nonSurface_normals(new NCloudType);
    extract->setNegative(true);
    extract->filter(*cloud_filtered);

    NormalIndiceExtractor::Ptr extract_normals(new NormalIndiceExtractor);
    extract_normals->setNegative(true);
    extract_normals->setInputCloud(normals);
    extract_normals->setIndices(inliers);
    extract_normals->filter(*nonSurface_normals);


    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(NormalDistanceWeight);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(CylinderDistanceThreshold);
    seg.setRadiusLimits(minRadius, maxRadius);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(nonSurface_normals);

    seg.segment(*inliers, *temp);

    extract->setInputCloud(cloud_filtered);
    extract->setIndices(inliers);
    extract->setNegative(false);
    CloudColorPtrType cloud_cylinder(new CloudColorType);
    extract->filter(*cloud_cylinder);

    return cloud_cylinder;
  }

  CloudColorPtrType
  pclMethod::findCylinder(double filterMinLimit, double filterMaxLimit, std::string filterAxis, int NormalNeighbors,
                          double fitPlaneDistanceThreshold, double fitCylinderDistanceThreshold,
                          double fitNormalDistanceWeight, double CylinderMinRadius, double CylinderMaxRadius) {

    auto temp = passThroughFilter(cloud, filterMinLimit, filterMaxLimit, filterAxis);
    auto normals = estimateNormals(temp, NormalNeighbors);
    return segmentation(temp, normals, fitPlaneDistanceThreshold, fitCylinderDistanceThreshold, fitNormalDistanceWeight,
                        CylinderMinRadius, CylinderMaxRadius);
  }
}
