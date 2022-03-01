SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
#SET(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# user build settings
SET(USE_LOOP_CLOSING FALSE)
SET(USE_GLOBAL_MAP FALSE)

SET(USE_16BIT_IMAGE TRUE)
SET(USE_16BIT_DETECTION TRUE)  # if true use fast16, else use histogram equalization + fast

# Set definitions
IF(USE_LOOP_CLOSING)
  ADD_DEFINITIONS(-DSVO_LOOP_CLOSING)
ENDIF()

IF(USE_GLOBAL_MAP)
  ADD_DEFINITIONS(-DSVO_GLOBAL_MAP)
ENDIF()

IF(USE_16BIT_IMAGE)
  ADD_DEFINITIONS(-DSTIO_USE_16BIT_IMAGE)

  IF(USE_16BIT_DETECTION)
    ADD_DEFINITIONS(-DSTIO_USE_16BIT_DETECTION)
  ENDIF()
ENDIF()

ADD_DEFINITIONS(-DSVO_USE_ROS)
ADD_DEFINITIONS(-DSVO_USE_OPENGV)
ADD_DEFINITIONS(-DSVO_DEPTHFILTER_IN_REPROJECTOR)

#############################################################################
# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -Werror -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas -Wno-unused-but-set-parameter -Wno-int-in-bool-context -Wno-maybe-uninitialized -Wno-unused-function")

IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
  ADD_DEFINITIONS(-DHAVE_FAST_NEON)
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse2 -msse3 -mssse3 -mno-avx")
ENDIF()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -ffast-math -fno-finite-math-only")

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wno-error=uninitialized")
