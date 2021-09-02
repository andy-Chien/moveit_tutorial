set(MOVEIT_LIB_NAME moveit_collision_detection_voxel)

message(STATUS "GVL include dirs are: ${gpu_voxels_INCLUDE_DIRS}")

add_library(${MOVEIT_LIB_NAME}
  src/collision_common.cpp
  src/collision_robot_voxel.cpp
  src/collision_world_voxel.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_compile_definitions(${MOVEIT_LIB_NAME} PRIVATE _SYSTEM_POSIX_=1)

target_link_libraries(
  ${MOVEIT_LIB_NAME} 
  LINK_PUBLIC ${catkin_LIBRARIES} 
  LINK_PUBLIC ${urdfdom_LIBRARIES} 
  LINK_PUBLIC ${urdfdom_headers_LIBRARIES} 
  LINK_PUBLIC ${LIBFCL_LIBRARIES} 
  # LINK_PUBLIC ${Boost_LIBRARIES}
  LINK_PUBLIC ${Boost_SYSTEM_LIBRARY}
  LINK_PUBLIC ${icl_core_LIBRARIES}
  LINK_PUBLIC ${gpu_voxels_LIBRARIES}
  LINK_PUBLIC ${CUDA_LIBRARIES}

)
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

add_library(collision_detector_voxel_plugin src/collision_detector_voxel_plugin_loader.cpp)
set_target_properties(collision_detector_voxel_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(collision_detector_voxel_plugin ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${Boost_SYSTEM_LIBRARY} ${icl_core_LIBRARIES} ${gpu_voxels_LIBRARIES} ${CUDA_LIBRARIES})


install(TARGETS ${MOVEIT_LIB_NAME} collision_detector_voxel_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES ./collision_detector_voxel_description_test.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_voxel_collision_detection test/test_voxel_collision_detection.cpp)
  target_link_libraries(test_voxel_collision_detection moveit_test_utils ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES} ${icl_core_LIBRARIES} ${gpu_voxels_LIBRARIES} ${CUDA_LIBRARIES})
  # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
  target_compile_options(test_voxel_collision_detection PRIVATE -Wno-deprecated-declarations)

endif()
