cmake_minimum_required(VERSION 2.8.3)
project(svo_gtsam_backend)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

################################################################################
# generic setup (compiler flags, etc.)
include(SvoSetup)

# gtsam
find_package(GTSAMCMakeTools REQUIRED)
find_package(GTSAM REQUIRED)
message(STATUS ${GTSAM_INCLUDE_DIR})
message(STATUS ${GTSAM_LIBRARIES})
include_directories(${GTSAM_INCLUDE_DIR})

# OpenMP
find_package(OpenMP QUIET)
if (OPENMP_FOUND)
  message(STATUS "OpenMP found")
  add_definitions(-DUSE_OPENMP)
endif ()

#############
# LIBRARIES #
#############
set(HEADERS
  include/svo/gtsam_backend_interface.hpp
  include/svo/gtsam_backend/estimator.hpp
  )

set(SOURCES
  src/gtsam_backend_interface.cpp
  src/estimator.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} gtsam)
target_link_libraries(${PROJECT_NAME} gtsam_unstable)


##########
# EXPORT #
##########
cs_install()
cs_export()
