cmake_minimum_required(VERSION 3.1.3)
project(moveit_tutorials)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(CUDA REQUIRED)
find_package(icl_core REQUIRED )
find_package(gpu_voxels REQUIRED)
# FIND_PACKAGE(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

find_package(fcl QUIET)
if (fcl_FOUND)
  set(LIBFCL_LIBRARIES ${FCL_LIBRARIES})
  set(LIBFCL_INCLUDE_DIRS ${FCL_INCLUDE_DIRS})
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBFCL_PC REQUIRED fcl)
  set(LIBFCL_INCLUDE_DIRS ${LIBFCL_PC_INCLUDE_DIRS})
  # find *absolute* paths to LIBFCL_LIBRARIES
  set(LIBFCL_LIBRARIES)
  foreach(_lib ${LIBFCL_PC_LIBRARIES})
    find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
    list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
  endforeach()
endif()

find_package(octomap REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

# set(VERSION_FILE_PATH "${CATKIN_DEVEL_PREFIX}/include")
# # Pass the folder of the generated version.h to catkin_package() for export in devel-space
# # This is how gencpp adds the folder of generated message code to the include dirs, see:
# #   https://github.com/ros/gencpp/blob/e5acaf6/cmake/gencpp-extras.cmake.em#L51-L54
# list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${VERSION_FILE_PATH})
# file(MAKE_DIRECTORY "${VERSION_FILE_PATH}/moveit_tutorials")

set(THIS_PACKAGE_INCLUDE_DIRS
    doc/interactivity/include
    doc/collision_detection_fcll/include
)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
    moveit_collision_detection_fcll
  CATKIN_DEPENDS
    eigen_stl_containers
    geometric_shapes
    geometry_msgs
    kdl_parser
    moveit_msgs
    octomap_msgs
    random_numbers
    sensor_msgs
    shape_msgs
    srdfdom
    std_msgs
    tf2_eigen
    tf2_geometry_msgs
    trajectory_msgs
    visualization_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    Boost
    EIGEN3
    LIBFCL
    OCTOMAP
    urdfdom
    urdfdom_headers
  )

add_definitions(
  ${icl_core_DEFINITIONS}
  ${gpu_voxels_DEFINITIONS}
)
message("icl_core_DEFINITIONS ${icl_core_DEFINITIONS}")


###########
## Build ##
###########

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS} 
  PUBLIC ${catkin_INCLUDE_DIRS} 
  PUBLIC ${Boost_INCLUDE_DIR} 
  PUBLIC ${EIGEN3_INCLUDE_DIRS}
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
  PUBLIC ${gpu_voxels_INCLUDE_DIRS}
  # ${orocos_kdl_INCLUDE_DIRS} # this should be removed in upcoming versions.
  PUBLIC ${CUDA_INCLUDE_DIRS}
)


add_subdirectory(doc/kinematics)
add_subdirectory(doc/robot_model_and_robot_state)
add_subdirectory(doc/planning)
add_subdirectory(doc/planning_scene)
add_subdirectory(doc/planning_scene_ros_api)
add_subdirectory(doc/motion_planning_api)
add_subdirectory(doc/motion_planning_pipeline)
add_subdirectory(doc/visualizing_collisions)
add_subdirectory(doc/move_group_interface)
add_subdirectory(doc/move_group_python_interface)
add_subdirectory(doc/state_display)
add_subdirectory(doc/interactivity)
add_subdirectory(doc/pick_place)
add_subdirectory(doc/perception_pipeline)
add_subdirectory(doc/collision_detection_fcll)
add_subdirectory(example_how_to_link)
