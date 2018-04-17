

add_subdirectory(cylinder_segmentation_lib)

add_executable (main main.cpp)

target_link_libraries (main ${PCL_LIBRARIES})
target_link_libraries (main CYLINDER_SEGMENTATION_LIB)

