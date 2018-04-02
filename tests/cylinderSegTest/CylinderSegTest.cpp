#include <string>
#include "CylinderSegTest.hpp"
#include "cylinder_segmentation_lib.hpp"

using namespace google_test_sample;
using namespace std;

CylinderSegTest::CylinderSegTest() {}

CylinderSegTest::~CylinderSegTest() {};

void CylinderSegTest::SetUp() {};

void CylinderSegTest::TearDown() {};

// Check library name
TEST(getNameTest, ShouldReturnSampleLib1Name) {
    auto cylSegLib = CylindricalSegmentationLib();
    const char *result = "CylindricalSegmentationLib";
    ASSERT_STREQ(cylSegLib.getName().c_str(),result);
}
// Checks if file is not error has occurred
TEST(findCylinderTest, ShouldReturnFileNotFoundError) {
    const char *path = filePath;
    auto cylSegLib = CylindricalSegmentationLib();
    const char *result = "File Not Found";
    ASSERT_STREQ(cylSegLib.findCylinder(path).c_str(),result);
}
// Checks if the given .pcd file has cylinder or not
TEST(findCylinderTest, ShouldReturnTrueIfCylinderFound) {
    const char *path = "/Users/anushamodwal/Documents/Workspace/AirInterview/Dataset/";
    auto cylSegLib = CylindricalSegmentationLib();
    const char *result = "Found";
    ASSERT_STREQ(cylSegLib.findCylinder(path).c_str(),result);
}

