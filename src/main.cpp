#include <iostream>
#include "cylinder_segmentation_lib.hpp"
#include "common.h"

using namespace std;
using namespace google_test_sample;
int main(int argc, char *argv[])
{
    filePath = argv[1];
    auto cylSegLib = CylindricalSegmentationLib();
    std::cout << cylSegLib.getName() << std::endl;
    cylSegLib.findCylinder(filePath);
}
