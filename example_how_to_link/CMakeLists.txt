# Create a library that uses GPU Voxels:
add_library (gvl_linkage_test_lib src/gvl_linkage_test_lib.cpp)

# Add an executable that calls the lib:
add_executable (gvl_linkage_test src/gvl_linkage_test.cpp)
add_executable (ur5_urdf_ros src/UR5_URDF_Loader_ROS_listener.cpp)

# Link the executable to the library.
# We currently also have to link against Boost and icl_core...
target_link_libraries (gvl_linkage_test
    LINK_PUBLIC gvl_linkage_test_lib
    LINK_PUBLIC ${Boost_SYSTEM_LIBRARY}
    LINK_PUBLIC ${icl_core_LIBRARIES}
    LINK_PUBLIC ${gpu_voxels_LIBRARIES}
    LINK_PUBLIC ${CUDA_LIBRARIES}
)

target_link_libraries (ur5_urdf_ros
    LINK_PUBLIC ${catkin_LIBRARIES}
    LINK_PUBLIC ${Boost_SYSTEM_LIBRARY}
    LINK_PUBLIC ${icl_core_LIBRARIES}
    LINK_PUBLIC ${gpu_voxels_LIBRARIES}
    LINK_PUBLIC ${CUDA_LIBRARIES}
)
