cmake_minimum_required(VERSION 2.8.3)
project(svo_backend_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

################################################################################
# generic setup (compiler flags, etc.)
include(SvoSetup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/svo/motion_detector.hpp
  include/svo/outlier_rejection.hpp
  )

set(SOURCES
  src/motion_detector.cpp
  src/outlier_rejection.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# EXPORT #
##########
cs_install()
cs_export()
