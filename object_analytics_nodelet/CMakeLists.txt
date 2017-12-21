# Copyright (c) 2017 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.3)
project(object_analytics_nodelet)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to Release" FORCE)
  message(${CMAKE_BUILD_TYPE})
endif()

set(catkin_dependent_components
  nodelet
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  message_filters
  image_geometry
  object_msgs
  object_analytics_msgs
  cv_bridge
  dynamic_reconfigure
  roslint
)

find_package(catkin REQUIRED COMPONENTS
  ${catkin_dependent_components}
)

find_package(PCL 1.7 REQUIRED COMPONENTS
  common
  segmentation
  kdtree
  search
  filters
)

find_package(OpenCV 3.2 REQUIRED)

# generate dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/SegmentationAlgorithms.cfg
  cfg/OrganizedMultiPlaneSegmentation.cfg
)

# about exporting include and libraries from this project
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${catkin_dpendent_component}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_SEGMENTATION_INCLUDE_DIRS}
)

# target library
add_library(${PROJECT_NAME}
  src/splitter/splitter.cpp
  src/splitter/splitter_nodelet.cpp
  src/segmenter/segmenter.cpp
  src/segmenter/segmenter_nodelet.cpp
  src/segmenter/algorithm_provider.cpp
  src/segmenter/organized_multi_plane_segmenter.cpp
  src/merger/merger.cpp
  src/merger/merger_nodelet.cpp
  src/model/object2d.cpp
  src/model/object3d.cpp
  src/model/projector_impl.cpp
  src/model/object_utils.cpp
  src/const.cpp
  src/tracker/tracking.cpp
  src/tracker/tracking_manager.cpp
  src/tracker/tracking_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_SEGMENTATION}
  opencv_tracking
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# dynamic configure header
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

# installation
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Flags
if(UNIX OR APPLE)
  # Linker flags.
  if( ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()

  # Compiler flags.
  if( ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    # GCC specific flags.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector-strong")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
    endif()
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Clang is compatbile with some of the flags.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel" )
    # Same as above, with exception that ICC compilation crashes with -fPIE option, even
    # though it uses -pie linker option that require -fPIE during compilation. Checksec
    # shows that it generates correct PIE anyway if only -pie is provided.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
  endif()

  # Generic flags.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-operator-names -Wformat -Wformat-security -Wall -fopenmp")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  # Dot not forward c++11 flag to GPU beucause it is not supported
  set( CUDA_PROPAGATE_HOST_FLAGS OFF )
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D_FORTIFY_SOURCE=2")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")
endif()

# test
if(CATKIN_ENABLE_TESTING)
  # static code check
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_python(cfg/SegmentationAlgorithms.cfg)
  roslint_python(cfg/OrganizedMultiPlaneSegmentation.cfg)
  roslint_add_test()

  # lib unit test
  find_package(PCL 1.7 REQUIRED COMPONENTS
    io
  )
set(UNITEST_LIBRARIES ${GTEST_LIBRARIES} ${PROJECT_NAME} pthread ${PCL_IO_LIBRARIES})

  get_filename_component(RESOURCE_DIR "tests/resource" ABSOLUTE)
  configure_file(tests/unittest_util.h.in tests/unittest_util.h)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  catkin_add_gtest(unittest_object3d tests/unittest_object3d.cpp tests/unittest_util.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
  target_link_libraries(unittest_object3d ${UNITEST_LIBRARIES})

  catkin_add_gtest(unittest_objectutils tests/unittest_objectutils.cpp tests/unittest_util.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
  target_link_libraries(unittest_objectutils ${UNITEST_LIBRARIES})

  # lib module test
  option(MTEST_TRACKING_ENABLE_VIEW "enable visual outputs" OFF)
  if(MTEST_TRACKING_ENABLE_VIEW STREQUAL "ON")
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMTEST_TRACKING_ENABLE_VIEW")
  endif()
  add_executable(mtest_tracking tests/mtest_tracking.cpp)
  target_link_libraries(mtest_tracking ${GTEST_LIBRARIES} ${PROJECT_NAME})
endif()
