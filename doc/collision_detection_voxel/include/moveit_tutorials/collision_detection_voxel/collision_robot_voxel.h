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

#ifndef MOVEIT_COLLISION_DETECTION_VOXEL_COLLISION_ROBOT_
#define MOVEIT_COLLISION_DETECTION_VOXEL_COLLISION_ROBOT_

#include <moveit_tutorials/collision_detection_voxel/collision_common.h>
#include <moveit_tutorials/collision_detection_voxel/voxel_compat.h>

namespace collision_detection
{
class CollisionRobotVoxel : public CollisionRobot
{
  friend class CollisionWorldVoxel;

public:
  CollisionRobotVoxel(const robot_model::RobotModelConstPtr& robot_model, double padding = 0.0, double scale = 1.0);

  CollisionRobotVoxel(const CollisionRobotVoxel& other);

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                          const robot_state::RobotState& state) const override;
  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state,
                          const AllowedCollisionMatrix& acm) const override;
  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state1,
                          const robot_state::RobotState& state2) const override;
  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state1,
                          const robot_state::RobotState& state2, const AllowedCollisionMatrix& acm) const override;

  void checkOtherCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state,
                           const CollisionRobot& other_robot,
                           const robot_state::RobotState& other_state) const override;
  void checkOtherCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state,
                           const CollisionRobot& other_robot, const robot_state::RobotState& other_state,
                           const AllowedCollisionMatrix& acm) const override;
  void checkOtherCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state1,
                           const robot_state::RobotState& state2, const CollisionRobot& other_robot,
                           const robot_state::RobotState& other_state1,
                           const robot_state::RobotState& other_state2) const override;
  void checkOtherCollision(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state1,
                           const robot_state::RobotState& state2, const CollisionRobot& other_robot,
                           const robot_state::RobotState& other_state1, const robot_state::RobotState& other_state2,
                           const AllowedCollisionMatrix& acm) const override;

  void distanceSelf(const DistanceRequest& req, DistanceResult& res,
                    const robot_state::RobotState& state) const override;

  void distanceOther(const DistanceRequest& req, DistanceResult& res, const robot_state::RobotState& state,
                     const CollisionRobot& other_robot, const robot_state::RobotState& other_state) const override;

protected:
  void updatedPaddingOrScaling(const std::vector<std::string>& links) override;
  void constructVoxelObject(const robot_state::RobotState& state, VoxelObject& voxel_obj) const;
  void allocSelfCollisionBroadPhase(const robot_state::RobotState& state, VoxelManager& manager) const;
  void getAttachedBodyObjects(const robot_state::AttachedBody* ab, std::vector<VoxelGeometryConstPtr>& geoms) const;

  void checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res, const robot_state::RobotState& state,
                                const AllowedCollisionMatrix* acm) const;
  void checkOtherCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                 const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                 const robot_state::RobotState& other_state, const AllowedCollisionMatrix* acm) const;

  std::vector<VoxelGeometryConstPtr> geoms_;
  std::vector<VoxelCollisionObjectConstPtr> voxel_objs_;
};
}  // namespace collision_detection

#endif
