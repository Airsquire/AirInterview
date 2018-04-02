#include <iostream>
#include <string>
#include "cylinder_segmentation_lib.hpp"

using namespace std;
using namespace google_test_sample;
int main(int argc, char *argv[])
{
    string filePath = "/Users/anushamodwal/Documents/Workspace/AirInterview/Dataset/";
    auto cylSegLib = CylindricalSegmentationLib();
    std::cout << cylSegLib.getName() << std::endl;
    cylSegLib.findCylinder(filePath);
}
