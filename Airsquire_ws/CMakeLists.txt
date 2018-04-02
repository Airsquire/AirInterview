cmake_minimum_required(VERSION 3.9)
project(Airsquire_ws)


set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O3")
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
#set(CMAKE_BUILD_TYPE RELEASE)
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "-Wall -Wextra")



include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${gtest_SOURCE_DIR}/include
        ${gtest_SOURCE_DIR}
)


add_executable(interview src/main.cpp include/general/pcdHandler.h include/general/PointTypes.h include/algorithm/pclMethod.cpp include/algorithm/pclMethod.h)
target_link_libraries(interview ${PCL_LIBRARIES} gtest gtest_main)
