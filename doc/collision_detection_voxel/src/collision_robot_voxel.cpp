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

#include <moveit_tutorials/collision_detection_voxel/collision_robot_voxel.h>
#include <moveit_tutorials/collision_detection_voxel/voxel_compat.h>

#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#endif

namespace collision_detection
{
CollisionRobotVoxel::CollisionRobotVoxel(const robot_model::RobotModelConstPtr& model, double padding, double scale)
  : CollisionRobot(model, padding, scale)
{
  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  std::size_t index;
  geoms_.resize(robot_model_->getLinkGeometryCount());
  voxel_objs_.resize(robot_model_->getLinkGeometryCount());
  // we keep the same order of objects as what RobotState *::getLinkState() returns
  for (auto link : links)
    for (std::size_t j = 0; j < link->getShapes().size(); ++j)
    {
      VoxelGeometryConstPtr g = createCollisionGeometry(link->getShapes()[j], getLinkScale(link->getName()),
                                                      getLinkPadding(link->getName()), link, j);
      if (g)
      {
        index = link->getFirstCollisionBodyTransformIndex() + j;
        geoms_[index] = g;

        // Need to store the FCL object so the AABB does not get recreated every time.
        // Every time this object is created, g->computeLocalAABB() is called  which is
        // very expensive and should only be calculated once. To update the AABB, use the
        // collObj->setTransform and then call collObj->computeAABB() to transform the AABB.
        voxel_objs_[index] = VoxelCollisionObjectConstPtr(new fcl::CollisionObjectd(g->collision_geometry_));
      }
      else
        ROS_ERROR_NAMED("collision_detection.voxel", "Unable to construct collision geometry for link '%s'",
                        link->getName().c_str());
    }
}

CollisionRobotVoxel::CollisionRobotVoxel(const CollisionRobotVoxel& other) : CollisionRobot(other)
{
  geoms_ = other.geoms_;
  voxel_objs_ = other.voxel_objs_;
}

void CollisionRobotVoxel::getAttachedBodyObjects(const robot_state::AttachedBody* ab,
                                               std::vector<VoxelGeometryConstPtr>& geoms) const
{
  const std::vector<shapes::ShapeConstPtr>& shapes = ab->getShapes();
  for (std::size_t i = 0; i < shapes.size(); ++i)
  {
    VoxelGeometryConstPtr co = createCollisionGeometry(shapes[i], getLinkScale(ab->getAttachedLinkName()),
                                                     getLinkPadding(ab->getAttachedLinkName()), ab, i);
    if (co)
      geoms.push_back(co);
  }
}

void CollisionRobotVoxel::constructVoxelObject(const robot_state::RobotState& state, VoxelObject& voxel_obj) const
{
  voxel_obj.collision_objects_.reserve(geoms_.size());
  fcl::Transform3d voxel_tf;

  for (std::size_t i = 0; i < geoms_.size(); ++i)
    if (geoms_[i] && geoms_[i]->collision_geometry_)
    {
      transform2voxel(state.getCollisionBodyTransform(geoms_[i]->collision_geometry_data_->ptr.link,
                                                    geoms_[i]->collision_geometry_data_->shape_index),
                    voxel_tf);
      auto coll_obj = new fcl::CollisionObjectd(*voxel_objs_[i]);
      coll_obj->setTransform(voxel_tf);
      coll_obj->computeAABB();
      voxel_obj.collision_objects_.push_back(VoxelCollisionObjectPtr(coll_obj));
    }

  // TODO: Implement a method for caching fcl::CollisionObject's for robot_state::AttachedBody's
  std::vector<const robot_state::AttachedBody*> ab;
  state.getAttachedBodies(ab);
  for (auto& body : ab)
  {
    std::vector<VoxelGeometryConstPtr> objs;
    getAttachedBodyObjects(body, objs);
    const EigenSTL::vector_Isometry3d& ab_t = body->getGlobalCollisionBodyTransforms();
    for (std::size_t k = 0; k < objs.size(); ++k)
      if (objs[k]->collision_geometry_)
      {
        transform2voxel(ab_t[k], voxel_tf);
        voxel_obj.collision_objects_.push_back(
            VoxelCollisionObjectPtr(new fcl::CollisionObjectd(objs[k]->collision_geometry_, voxel_tf)));
        // we copy the shared ptr to the CollisionGeometryData, as this is not stored by the class itself,
        // and would be destroyed when objs goes out of scope.
        voxel_obj.collision_geometry_.push_back(objs[k]);
      }
  }
}

void CollisionRobotVoxel::allocSelfCollisionBroadPhase(const robot_state::RobotState& state, VoxelManager& manager) const
{
  auto m = new fcl::DynamicAABBTreeCollisionManagerd();
  // m->tree_init_level = 2;
  manager.manager_.reset(m);
  constructVoxelObject(state, manager.object_);
  manager.object_.registerTo(manager.manager_.get());
  // manager.manager_->update();
}

void CollisionRobotVoxel::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void CollisionRobotVoxel::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state,
                                           const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void CollisionRobotVoxel::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state1,
                                           const robot_state::RobotState& state2) const
{
  ROS_ERROR_NAMED("collision_detection.voxel", "Voxel continuous collision checking not yet implemented");
}

void CollisionRobotVoxel::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                           const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.voxel", "Voxel continuous collision checking not yet implemented");
}

void CollisionRobotVoxel::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                 const robot_state::RobotState& state,
                                                 const AllowedCollisionMatrix* acm) const
{
  VoxelManager manager;
  allocSelfCollisionBroadPhase(state, manager);
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  manager.manager_->collide(&cd, &collisionCallback);
  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(getRobotModel());
    distanceSelf(dreq, dres, state);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionRobotVoxel::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                            const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                            const robot_state::RobotState& other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, nullptr);
}

void CollisionRobotVoxel::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                            const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                            const robot_state::RobotState& other_state,
                                            const AllowedCollisionMatrix& acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void CollisionRobotVoxel::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                            const robot_state::RobotState& state1,
                                            const robot_state::RobotState& state2, const CollisionRobot& other_robot,
                                            const robot_state::RobotState& other_state1,
                                            const robot_state::RobotState& other_state2) const
{
  ROS_ERROR_NAMED("collision_detection.voxel", "Voxel continuous collision checking not yet implemented");
}

void CollisionRobotVoxel::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                            const robot_state::RobotState& state1,
                                            const robot_state::RobotState& state2, const CollisionRobot& other_robot,
                                            const robot_state::RobotState& other_state1,
                                            const robot_state::RobotState& other_state2,
                                            const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.voxel", "Voxel continuous collision checking not yet implemented");
}

void CollisionRobotVoxel::checkOtherCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const robot_state::RobotState& state,
                                                  const CollisionRobot& other_robot,
                                                  const robot_state::RobotState& other_state,
                                                  const AllowedCollisionMatrix* acm) const
{
  VoxelManager manager;
  allocSelfCollisionBroadPhase(state, manager);

  const CollisionRobotVoxel& voxel_rob = dynamic_cast<const CollisionRobotVoxel&>(other_robot);
  VoxelObject other_voxel_obj;
  voxel_rob.constructVoxelObject(other_state, other_voxel_obj);

  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  for (std::size_t i = 0; !cd.done_ && i < other_voxel_obj.collision_objects_.size(); ++i)
    manager.manager_->collide(other_voxel_obj.collision_objects_[i].get(), &cd, &collisionCallback);

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(getRobotModel());
    distanceOther(dreq, dres, state, other_robot, other_state);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionRobotVoxel::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  std::size_t index;
  for (const auto& link : links)
  {
    const robot_model::LinkModel* lmodel = robot_model_->getLinkModel(link);
    if (lmodel)
    {
      for (std::size_t j = 0; j < lmodel->getShapes().size(); ++j)
      {
        VoxelGeometryConstPtr g = createCollisionGeometry(lmodel->getShapes()[j], getLinkScale(lmodel->getName()),
                                                        getLinkPadding(lmodel->getName()), lmodel, j);
        if (g)
        {
          index = lmodel->getFirstCollisionBodyTransformIndex() + j;
          geoms_[index] = g;
          voxel_objs_[index] = VoxelCollisionObjectConstPtr(new fcl::CollisionObjectd(g->collision_geometry_));
        }
      }
    }
    else
      ROS_ERROR_NAMED("collision_detection.voxel", "Updating padding or scaling for unknown link: '%s'", link.c_str());
  }
}

void CollisionRobotVoxel::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                     const robot_state::RobotState& state) const
{
  VoxelManager manager;
  allocSelfCollisionBroadPhase(state, manager);
  DistanceData drd(&req, &res);

  manager.manager_->distance(&drd, &distanceCallback);
}

void CollisionRobotVoxel::distanceOther(const DistanceRequest& req, DistanceResult& res,
                                      const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                      const robot_state::RobotState& other_state) const
{
  VoxelManager manager;
  allocSelfCollisionBroadPhase(state, manager);

  const CollisionRobotVoxel& voxel_rob = dynamic_cast<const CollisionRobotVoxel&>(other_robot);
  VoxelObject other_voxel_obj;
  voxel_rob.constructVoxelObject(other_state, other_voxel_obj);

  DistanceData drd(&req, &res);
  for (std::size_t i = 0; !drd.done && i < other_voxel_obj.collision_objects_.size(); ++i)
    manager.manager_->distance(other_voxel_obj.collision_objects_[i].get(), &drd, &distanceCallback);
}

}  // end of namespace collision_detection
