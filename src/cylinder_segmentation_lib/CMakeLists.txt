set(CYLINDER_SEGMENTATION_LIB_SRCS
    cylinder_segmentation_lib.hpp
    cylinder_segmentation_lib.cpp
)

# Declare the library
add_library(CYLINDER_SEGMENTATION_LIB STATIC
    ${CYLINDER_SEGMENTATION_LIB_SRCS}
)

# Specify here the include directories exported
# by this library
target_include_directories(CYLINDER_SEGMENTATION_LIB PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)