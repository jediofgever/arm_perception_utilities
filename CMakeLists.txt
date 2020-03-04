cmake_minimum_required(VERSION 2.8.3)
project(arm_perception_utilities)

add_compile_options(-std=c++11)

find_package(ApproxMVBB REQUIRED COMPONENTS XML_SUPPORT KDTREE_SUPPORT)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(OpenCV REQUIRED )
find_package(jsoncpp CONFIG REQUIRED)
find_package(gazebo 9 REQUIRED)
set(Python_ADDITIONAL_VERSIONS 3.6)
find_package(PythonLibs 3 REQUIRED)

 
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    rospy
    roscpp
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    trajectory_msgs
    cv_bridge
    camera_info_manager
    gazebo_ros
    cmake_modules
)

 

find_package(PCL 1.7 REQUIRED)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
 

catkin_package(
  LIBRARIES
   INCLUDE_DIRS include
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} )
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(  include
${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

 

add_executable(fake_pickplace_ros_node src/pickplace/fake_pickplace_ros_node.cpp
src/utils.cpp
src/control/RobotController.cpp
src/pickplace/PickandPlacer.cpp
src/pickplace/FakePickPlace.cpp)
target_link_libraries(fake_pickplace_ros_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${Eigen3_LIBRARIES} )
install(TARGETS fake_pickplace_ros_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
 

add_executable(real_pickplace_ros_node src/pickplace/real_pickplace_ros_node.cpp
src/utils.cpp
src/control/RobotController.cpp
src/pickplace/PickandPlacer.cpp
src/pickplace/RealPickPlace.cpp)
target_link_libraries(real_pickplace_ros_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${Eigen3_LIBRARIES} )
install(TARGETS real_pickplace_ros_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
 


add_executable(simulation_pickplace_ros_node src/pickplace/simulation_pickplace_ros_node.cpp
src/utils.cpp
src/control/RobotController.cpp
src/pickplace/PickandPlacer.cpp
src/pickplace/SimulationPickPlace.cpp)
target_link_libraries(simulation_pickplace_ros_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${Eigen3_LIBRARIES} )
install(TARGETS simulation_pickplace_ros_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
 
 

add_executable(state_visualization_ros_node src/visualization/visualization_ros_node.cpp
src/utils.cpp
src/visualization/RobotStateVisualization.cpp)
target_link_libraries(state_visualization_ros_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${Eigen3_LIBRARIES} )
install(TARGETS state_visualization_ros_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
 


add_executable(vision_ros_node src/vision/vision_ros_node.cpp
src/vision/PointCloudManager.cpp
src/vision/Object3DDetector.cpp
src/utils.cpp)
target_link_libraries(vision_ros_node ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} jsoncpp_lib ${Eigen3_LIBRARIES} ApproxMVBB::Core )
install(TARGETS vision_ros_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
 


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
