cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3_ros)
set(ORB_SLAM_CORE "orb_slam3_core") # bare ORB_slam3 (without ROS)

# make sure the default build type is Release, otherwise the performance will be very slow
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_geometry_msgs
  tf2_ros
)

# VOXL-specific settings, OpenCV and PCL are installed in different locations (other than where CMake searches for by default)
if(EXISTS "/etc/modalai/voxl-cpu-monitor.conf")
  set(OpenCV_DIR /usr/lib64/cmake/opencv4)
  set(PCL_DIR /usr/lib/aarch64-linux-gnu/cmake/pcl)
  message(STATUS "Building on ModalAI platform..")
else()
  message(STATUS "Building on non-ModalAI platform..")
endif()

find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.0 QUIET)
  if(NOT OpenCV_FOUND)
      find_package(OpenCV 2.4.3 QUIET)
      if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
  endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)     
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Boost serialization not found.")
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveMap.srv
  SetLocalizationMode.srv
  RescaleMap.srv
  SetMopp.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   config/dynamic_reconfigure.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs message_runtime
  LIBRARIES lib${ORB_SLAM_CORE} libDBoW2 libg2o
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/orb_slam3_core
  ${PROJECT_SOURCE_DIR}/orb_slam3_core/include
  ${PROJECT_SOURCE_DIR}/orb_slam3_core/include/CameraModels
  ${PROJECT_SOURCE_DIR}/orb_slam3_core/Thirdparty/Sophus # just in case relative paths are used in the code
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include(${PROJECT_SOURCE_DIR}/orb_slam3_core/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam3_core/Thirdparty/g2o/CMakeLists.txt)

## Declare a C++ library
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3_core/lib)
add_library(${ORB_SLAM_CORE} SHARED
  orb_slam3_core/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3_core/src/CameraModels/Pinhole.cpp
  orb_slam3_core/src/Atlas.cc
  orb_slam3_core/src/Config.cc
  orb_slam3_core/src/Converter.cc
  orb_slam3_core/src/Frame.cc
  orb_slam3_core/src/FrameDrawer.cc
  orb_slam3_core/src/G2oTypes.cc
  orb_slam3_core/src/GeometricTools.cc
  orb_slam3_core/src/ImuTypes.cc
  orb_slam3_core/src/KeyFrame.cc
  orb_slam3_core/src/KeyFrameDatabase.cc
  orb_slam3_core/src/LocalMapping.cc
  orb_slam3_core/src/LoopClosing.cc
  orb_slam3_core/src/Map.cc
  orb_slam3_core/src/MapPoint.cc
  orb_slam3_core/src/MLPnPsolver.cpp
  orb_slam3_core/src/OptimizableTypes.cpp
  orb_slam3_core/src/Optimizer.cc
  orb_slam3_core/src/ORBextractor.cc
  orb_slam3_core/src/ORBmatcher.cc
  orb_slam3_core/src/Settings.cc
  orb_slam3_core/src/Sim3Solver.cc
  orb_slam3_core/src/System.cc
  orb_slam3_core/src/Tracking.cc
  orb_slam3_core/src/TwoViewReconstruction.cc
)
add_dependencies(${ORB_SLAM_CORE} g2o DBoW2)

set(LIBS_ORBSLAM
  ${OpenCV_LIBS}
  ${Eigen3_LIBS}
  ${Boost_LIBRARIES} # added for the serialization library
  ${PROJECT_SOURCE_DIR}/orb_slam3_core/Thirdparty/DBoW2/lib/libDBoW2.so # this was built by including DBow2 CMakeLists.txt
  ${PROJECT_SOURCE_DIR}/orb_slam3_core/Thirdparty/g2o/lib/libg2o.so # this was built by including g2o CMakeLists.txt
)

set(LIBS_ROS
  ${PROJECT_SOURCE_DIR}/orb_slam3_core/lib/lib${ORB_SLAM_CORE}.so # this was built by add_library(${ORB_SLAM_CORE} SHARED ...)
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES} # added for the serialization library
  ${catkin_LIBRARIES}
)
message(STATUS "LIBS_ROS: ${LIBS_ROS}, LIBS_ORBSLAM: ${LIBS_ORBSLAM}") # simply to check the values

target_link_libraries(${ORB_SLAM_CORE}
  ${LIBS_ORBSLAM}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_mono 
#   src/mono.cpp
#   src/node.cpp
# )
# add_dependencies(${PROJECT_NAME}_mono 
#   ${ORB_SLAM_CORE}
# )
# target_link_libraries(${PROJECT_NAME}_mono
#   ${LIBS_ROS}
# )

add_executable(${PROJECT_NAME}_stereo 
  src/stereo.cpp
  src/node.cpp
)
add_dependencies(${PROJECT_NAME}_stereo 
  ${ORB_SLAM_CORE}
)
target_link_libraries(${PROJECT_NAME}_stereo
  ${LIBS_ROS}
)

# add_executable(${PROJECT_NAME}_rgbd 
#   src/rgbd.cpp
#   src/node.cpp
# )
# add_dependencies(${PROJECT_NAME}_rgbd 
#   ${ORB_SLAM_CORE}
# )
# target_link_libraries(${PROJECT_NAME}_rgbd
#   ${LIBS_ROS}
# )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}
#   ${LIBS_ORBSLAM}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_orb_slam3_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
