
#include "general/pcdHandler.h"
#include <pcl/visualization/cloud_viewer.h>
#include "algorithm/pclMethod.h"
#include "gtest/gtest.h"


using namespace interview;

int main(int argc, char** argv) {


  std::string fileName = argv[argc - 1];

  pcdHandler reader(fileName);
  CloudColorPtrType cloud = reader.getCloud();
  pclMethod cylinderEstimator(cloud);
  CloudColorPtrType output;
  output = cylinderEstimator.findCylinder();
  pcl::visualization::CloudViewer viewer("Airsquire Viewer");
  viewer.showCloud(output);


  while (!viewer.wasStopped ())
  {
    usleep( 10 );
  }

  reader.writer(output);

  return 0;
}