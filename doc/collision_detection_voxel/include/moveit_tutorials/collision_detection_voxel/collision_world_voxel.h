/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef MOVEIT_COLLISION_DETECTION_Voxel_COLLISION_WORLD_VOXEL_
#define MOVEIT_COLLISION_DETECTION_Voxel_COLLISION_WORLD_VOXEL_

#include <cstdlib>
#include <signal.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <icl_core_config/Config.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <moveit_tutorials/collision_detection_voxel/collision_robot_voxel.h>
#include <moveit_tutorials/collision_detection_voxel/voxel_compat.h>

#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
#include <fcl/broadphase/broadphase_collision_manager.h>
#else
#include <fcl/broadphase/broadphase.h>
#endif

using namespace gpu_voxels;
using boost::dynamic_pointer_cast;
using boost::shared_ptr;
using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;
namespace bfs = boost::filesystem;

namespace collision_detection
{
class CollisionWorldVoxel : public CollisionWorld
{
public:
  CollisionWorldVoxel();
  explicit CollisionWorldVoxel(const WorldPtr& world);
  CollisionWorldVoxel(const CollisionWorldVoxel& other, const WorldPtr& world);
  ~CollisionWorldVoxel() override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                           const robot_state::RobotState& state) const override;
  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                           const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const override;
  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                           const robot_state::RobotState& state1, const robot_state::RobotState& state2) const override;
  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                           const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                           const AllowedCollisionMatrix& acm) const override;
  void checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                           const CollisionWorld& other_world) const override;
  void checkWorldCollision(const CollisionRequest& req, CollisionResult& res, const CollisionWorld& other_world,
                           const AllowedCollisionMatrix& acm) const override;

  void distanceRobot(const DistanceRequest& req, DistanceResult& res, const CollisionRobot& robot,
                     const robot_state::RobotState& state) const override;

  void distanceWorld(const DistanceRequest& req, DistanceResult& res, const CollisionWorld& world) const override;

  void setWorld(const WorldPtr& world) override;

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
  void voxelInit();

protected:
  void checkWorldCollisionHelper(const CollisionRequest& req, CollisionResult& res, const CollisionWorld& other_world,
                                 const AllowedCollisionMatrix* acm) const;
  void checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                 const robot_state::RobotState& state, const AllowedCollisionMatrix* acm) const;

  void constructVoxelObject(const World::Object* obj, VoxelObject& voxel_obj) const;
  void updateVoxelObject(const std::string& id);

  std::unique_ptr<fcl::BroadPhaseCollisionManagerd> manager_;
  std::map<std::string, VoxelObject> voxel_objs_;

private:
  ros::NodeHandle n;
  ros::Subscriber point_sub;
  ros::Subscriber robot_sub;
  shared_ptr<GpuVoxels> gvl;

  Vector3ui map_dimensions;
  float voxel_side_length; // 1 cm voxel size

  bool new_data_received;
  PointCloud my_point_cloud;
  Matrix4f tf;
  BitVectorVoxel bits_in_collision;

  size_t iteration;

  robot::JointValueMap myRobotJointValues;

  void initialize();
  void notifyObjectChange(const ObjectConstPtr& obj, World::Action action);
  World::ObserverHandle observer_handle_;
};
}  // namespace collision_detection

#endif
