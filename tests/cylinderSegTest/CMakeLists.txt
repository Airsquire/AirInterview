file(GLOB SRCS *.cpp)

ADD_EXECUTABLE(cylinderSegTest ${SRCS})

TARGET_LINK_LIBRARIES(
    cylinderSegTest
    CYLINDER_SEGMENTATION_LIB
    libgtest
    libgmock
    ${PCL_LIBRARIES}
)

add_test(NAME cylinderSegTest
         COMMAND cylinderSegTest)