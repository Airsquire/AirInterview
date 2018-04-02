
#ifndef AIRSQUIRE_WS_PCDHANDLER_H
#define AIRSQUIRE_WS_PCDHANDLER_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include "PointTypes.h"


namespace interview {

/**
* \class pcdHandler
* \brief This class reads .PCD files
* \details It gets the path of the interest .PCD file and stores it as XYZRGB point cloud.
*/


  class pcdHandler {
  private:
    const std::string file;
    CloudColorPtrType inputColorCloud;
  public:

    explicit pcdHandler(const std::string &input) : file(input) {
      reader();
    }

    /**
     * \brief reads .PCD file in XYZRGB format and stores data in private point cloud
          */
    void reader() {
      inputColorCloud.reset(new CloudColorType);
      if (pcl::io::loadPCDFile<PointColorType>(file, *inputColorCloud) == -1) {
        PCL_ERROR ("Couldn't read colored data\n");
      }
    }

    /**
     * \brief writes .PCD file in XYZRGB format at the same path of the imported file
          */

    void writer(CloudColorPtrType inputCloud) {
      auto pose = file.find_last_of("/") + 1;
      auto path = file.substr(0, pose);
      auto name = "Export_" + file.substr(pose, file.size());
      auto filename = path + name;
      pcl::io::savePCDFile(filename, *inputCloud);
    }

    /** \brief return private point cloud
*
          * \param[out] inputColorCloud is the container consisting of the XYZRGB Points.
          */

    CloudColorPtrType getCloud() {
      return inputColorCloud;
    }

  };
};


#endif //AIRSQUIRE_WS_PCDHANDLER_H
