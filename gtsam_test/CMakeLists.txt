cmake_minimum_required(VERSION 2.6)

project(gesam_test CXX C)

find_package(GTSAM REQUIRED)

include_directories($(GTSAM_INCLUDE_DIR))
set(GTSAM_LIBRARIES gtsam)

find_package(GESAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_subdirectory(cpp)

