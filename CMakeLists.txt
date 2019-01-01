cmake_minimum_required(VERSION 3.10)

project( VisualOdometry )

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

# Enable debug flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3  -Wall")

message(STATUS "Compiler set to: ${CMAKE_CXX_COMPILER}")
message(STATUS "Comper flags: ${CMAKE_CXX_FLAGS}")

set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
#find_package(GTSAM_UNSTABLE REQUIRED)
find_package(Boost REQUIRED filesystem)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

# Include our header files
include_directories(
        src
        src/evaluate
        src/thirdparty
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        #${GTSAM_UNSTABLE_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS} # For Boost::Filesystem
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIR}
        /usr/include/GeographicLib
)

#####################################
# Main class files
#####################################
set(sources_visual_odom
        src/easylogging++.cc
        src/visualOdometryStereo.cpp
        src/mapDrawer.cpp
        src/system.cpp
        src/gtsamOptimizer.cpp
        src/loopDetector.cpp
        src/orbExtractor.cpp
        )


# Create the main executable, and link libraries
add_executable(run
        src/main.cpp
        ${sources_visual_odom}
        )

target_link_libraries(run
        ${EIGEN3_LIBRARIES}
        ${Boost_LIBRARIES}
        ${GTSAM_LIBS}
        gtsam
        #${GTSAM_UNSTABLE_LIBS}
        ${OpenCV_LIBRARIES}
        ${Pangolin_LIBRARY}
        /usr/lib/x86_64-linux-gnu/libGeographic.so
        ${PROJECT_SOURCE_DIR}/src/thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/src/thirdparty/DLib/lib/libDLib.so
        )
