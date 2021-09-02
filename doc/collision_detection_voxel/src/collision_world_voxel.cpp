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

#include <moveit_tutorials/collision_detection_voxel/collision_world_voxel.h>
#include <moveit_tutorials/collision_detection_voxel/collision_detector_allocator_voxel.h>
#include <moveit_tutorials/collision_detection_voxel/voxel_compat.h>

#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <fcl/narrowphase/detail/traversal/collision/bvh_collision_traversal_node.h>
#include <fcl/narrowphase/detail/traversal/collision_node.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#else
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>
#endif

#include <boost/bind.hpp>

namespace collision_detection
{
const std::string CollisionDetectorAllocatorVoxel::NAME("FUCK");
// static const std::string NAME = "Voxel";
// const std::string& CollisionDetectorAllocatorVoxel::getName() const
// {
//   return NAME;
// }

CollisionWorldVoxel::CollisionWorldVoxel() : CollisionWorld()
{
  auto m = new fcl::DynamicAABBTreeCollisionManagerd();
  // m->tree_init_level = 2;
  manager_.reset(m);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldVoxel::notifyObjectChange, this, _1, _2));
}

CollisionWorldVoxel::CollisionWorldVoxel(const WorldPtr& world) : CollisionWorld(world)
{
  auto m = new fcl::DynamicAABBTreeCollisionManagerd();
  // m->tree_init_level = 2;
  manager_.reset(m);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldVoxel::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionWorldVoxel::CollisionWorldVoxel(const CollisionWorldVoxel& other, const WorldPtr& world)
  : CollisionWorld(other, world)
{
  auto m = new fcl::DynamicAABBTreeCollisionManagerd();
  // m->tree_init_level = 2;
  manager_.reset(m);

  voxel_objs_ = other.voxel_objs_;
  for (auto& voxel_obj : voxel_objs_)
    voxel_obj.second.registerTo(manager_.get());
  // manager_->update();

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldVoxel::notifyObjectChange, this, _1, _2));
}

CollisionWorldVoxel::~CollisionWorldVoxel()
{
  getWorld()->removeObserver(observer_handle_);
}

void CollisionWorldVoxel::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, robot, state, nullptr);
}

void CollisionWorldVoxel::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state,
                                            const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void CollisionWorldVoxel::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state1,
                                            const robot_state::RobotState& state2) const
{
  ROS_ERROR_NAMED("collision_detection.voxel", "Voxel continuous collision checking not yet implemented");
}

void CollisionWorldVoxel::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state1,
                                            const robot_state::RobotState& state2,
                                            const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.voxel", "Voxel continuous collision checking not yet implemented");
}

void CollisionWorldVoxel::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const CollisionRobot& robot, const robot_state::RobotState& state,
                                                  const AllowedCollisionMatrix* acm) const
{
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  const CollisionRobotVoxel& robot_voxel = dynamic_cast<const CollisionRobotVoxel&>(robot);
  VoxelObject voxel_obj;
  robot_voxel.constructVoxelObject(state, voxel_obj);

  CollisionData cd(&req, &res, acm);
  cd.enableGroup(robot.getRobotModel());
  for (std::size_t i = 0; !cd.done_ && i < voxel_obj.collision_objects_.size(); ++i)
    manager_->collide(voxel_obj.collision_objects_[i].get(), &cd, &collisionCallback);

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(robot.getRobotModel());
    distanceRobot(dreq, dres, robot, state);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionWorldVoxel::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionWorld& other_world) const
{
  checkWorldCollisionHelper(req, res, other_world, nullptr);
}

void CollisionWorldVoxel::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionWorld& other_world, const AllowedCollisionMatrix& acm) const
{
  checkWorldCollisionHelper(req, res, other_world, &acm);
}

void CollisionWorldVoxel::checkWorldCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const CollisionWorld& other_world,
                                                  const AllowedCollisionMatrix* acm) const
{
  std::cout << "===================================================================================" << std::endl;
  const CollisionWorldVoxel& other_voxel_world = dynamic_cast<const CollisionWorldVoxel&>(other_world);
  CollisionData cd(&req, &res, acm);
  manager_->collide(other_voxel_world.manager_.get(), &cd, &collisionCallback);

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    distanceWorld(dreq, dres, other_world);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionWorldVoxel::constructVoxelObject(const World::Object* obj, VoxelObject& voxel_obj) const
{
  for (std::size_t i = 0; i < obj->shapes_.size(); ++i)
  {
    VoxelGeometryConstPtr g = createCollisionGeometry(obj->shapes_[i], obj);
    if (g)
    {
      auto co = new fcl::CollisionObjectd(g->collision_geometry_, transform2voxel(obj->shape_poses_[i]));
      voxel_obj.collision_objects_.push_back(VoxelCollisionObjectPtr(co));
      voxel_obj.collision_geometry_.push_back(g);
    }
  }
}

void CollisionWorldVoxel::updateVoxelObject(const std::string& id)
{
  // remove FCL objects that correspond to this object
  auto jt = voxel_objs_.find(id);
  if (jt != voxel_objs_.end())
  {
    jt->second.unregisterFrom(manager_.get());
    jt->second.clear();
  }

  // check to see if we have this object
  auto it = getWorld()->find(id);
  if (it != getWorld()->end())
  {
    // construct Voxel objects that correspond to this object
    if (jt != voxel_objs_.end())
    {
      constructVoxelObject(it->second.get(), jt->second);
      jt->second.registerTo(manager_.get());
    }
    else
    {
      constructVoxelObject(it->second.get(), voxel_objs_[id]);
      voxel_objs_[id].registerTo(manager_.get());
    }
  }
  else
  {
    if (jt != voxel_objs_.end())
      voxel_objs_.erase(jt);
  }

  // manager_->update();
}

void CollisionWorldVoxel::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  // clear out objects from old world
  manager_->clear();
  voxel_objs_.clear();
  cleanCollisionGeometryCache();

  CollisionWorld::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldVoxel::notifyObjectChange, this, _1, _2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionWorldVoxel::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  if (action == World::DESTROY)
  {
    auto it = voxel_objs_.find(obj->id_);
    if (it != voxel_objs_.end())
    {
      it->second.unregisterFrom(manager_.get());
      it->second.clear();
      voxel_objs_.erase(it);
    }
    cleanCollisionGeometryCache();
  }
  else
  {
    updateVoxelObject(obj->id_);
    if (action & (World::DESTROY | World::REMOVE_SHAPE))
      cleanCollisionGeometryCache();
  }
}

void CollisionWorldVoxel::distanceRobot(const DistanceRequest& req, DistanceResult& res, const CollisionRobot& robot,
                                      const robot_state::RobotState& state) const
{
  const CollisionRobotVoxel& robot_voxel = dynamic_cast<const CollisionRobotVoxel&>(robot);
  VoxelObject voxel_obj;
  robot_voxel.constructVoxelObject(state, voxel_obj);

  DistanceData drd(&req, &res);
  for (std::size_t i = 0; !drd.done && i < voxel_obj.collision_objects_.size(); ++i)
    manager_->distance(voxel_obj.collision_objects_[i].get(), &drd, &distanceCallback);
}

void CollisionWorldVoxel::distanceWorld(const DistanceRequest& req, DistanceResult& res, const CollisionWorld& world) const
{
  const CollisionWorldVoxel& other_voxel_world = dynamic_cast<const CollisionWorldVoxel&>(world);
  DistanceData drd(&req, &res);
  manager_->distance(other_voxel_world.manager_.get(), &drd, &distanceCallback);
}

}  // end of namespace collision_detection
