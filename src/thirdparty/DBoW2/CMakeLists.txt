cmake_minimum_required(VERSION 2.8)
project(DBoW2)
include(ExternalProject)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

if(MSVC)
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")
endif()

set(HDRS
  include/BowVector.h           
  include/FBrief.h
  include/QueryResults.h
  include/TemplatedDatabase.h
  include/FORB.h
  include/DBoW2.h    
  include/FClass.h
  include/FeatureVector.h
  include/ScoringObject.h       
  include/TemplatedVocabulary.h
)

set(SRCS 
  src/BowVector.cpp     
  src/FBrief.cpp       
  src/FORB.cpp
  src/FeatureVector.cpp 
  src/QueryResults.cpp
  src/ScoringObject.cpp
)

set(DEPENDENCY_DIR ${CMAKE_CURRENT_BINARY_DIR}/dependencies)
set(DEPENDENCY_INSTALL_DIR ${DEPENDENCY_DIR}/install)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}   ../DLib/include)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED ${SRCS})
include_directories(include/)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PROJECT_SOURCE_DIR}/../DLib/lib/libDLib.so)


configure_file(src/DBoW2.cmake.in
  "${PROJECT_BINARY_DIR}/DBoW2Config.cmake" @ONLY)

