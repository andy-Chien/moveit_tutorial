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
  voxelInit();
  auto m = new fcl::DynamicAABBTreeCollisionManagerd();
  // m->tree_init_level = 2;
  manager_.reset(m);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldVoxel::notifyObjectChange, this, _1, _2));
}

CollisionWorldVoxel::CollisionWorldVoxel(const WorldPtr& world) : CollisionWorld(world)
{
  voxelInit();
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
  voxelInit();
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
  BitVectorVoxel coll_bits = bits_in_collision;
  size_t num_colls;
  num_colls = gvl->getMap("myHandVoxellist")->as<voxelmap::BitVectorVoxelMap>()->collideWithTypes(gvl->getMap("myHandVoxellist_2")->as<voxelmap::BitVectorVoxelMap>(), coll_bits);
  num_colls += gvl->getMap("myHandVoxellist")->as<voxelmap::BitVectorVoxelMap>()->collideWithTypes(gvl->getMap("countingVoxelList")->as<voxelmap::BitVectorVoxelMap>(), coll_bits);
  num_colls += gvl->getMap("myHandVoxellist_2")->as<voxelmap::BitVectorVoxelMap>()->collideWithTypes(gvl->getMap("countingVoxelList")->as<voxelmap::BitVectorVoxelMap>(), coll_bits);
  std::cout << "Detected " << num_colls << " collisions" << std::endl;
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
  BitVectorVoxel coll_bits = bits_in_collision;
  size_t num_colls;
  num_colls = gvl->getMap("myHandVoxellist")->as<voxelmap::BitVectorVoxelMap>()->collideWithTypes(gvl->getMap("myHandVoxellist_2")->as<voxelmap::BitVectorVoxelMap>(), coll_bits);
  num_colls += gvl->getMap("myHandVoxellist")->as<voxelmap::BitVectorVoxelMap>()->collideWithTypes(gvl->getMap("countingVoxelList")->as<voxelmap::BitVectorVoxelMap>(), coll_bits);
  num_colls += gvl->getMap("myHandVoxellist_2")->as<voxelmap::BitVectorVoxelMap>()->collideWithTypes(gvl->getMap("countingVoxelList")->as<voxelmap::BitVectorVoxelMap>(), coll_bits);
  std::cout << "Detected " << num_colls << " collisions" << std::endl;
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

void CollisionWorldVoxel::voxelInit()
{
  std::cout<<"yoyoyoyoyoyoyoyoyoyoyoyoyoyoyoyoyoy"<<std::endl;
  Vector3ui map_dim(256, 256, 256);
  map_dimensions =  map_dim;
  voxel_side_length = 0.01f; // 1 cm voxel size

  icl_core::logging::initialize();
  /*
   * First, we generate an API class, which defines the
   * volume of our space and the resolution.
   * Be careful here! The size is limited by the memory
   * of your GPU. Even if an empty Octree is small, a
   * Voxelmap will always require the full memory.
   */

  icl_core::config::GetoptParameter points_parameter("points-topic:", "t",
                                                    "Identifer of the pointcloud topic");
  icl_core::config::GetoptParameter roll_parameter  ("roll:", "r",
                                                    "Camera roll in degrees");
  icl_core::config::GetoptParameter pitch_parameter ("pitch:", "p",
                                                    "Camera pitch in degrees");
  icl_core::config::GetoptParameter yaw_parameter   ("yaw:", "y",
                                                    "Camera yaw in degrees");
  icl_core::config::GetoptParameter voxel_side_length_parameter("voxel_side_length:", "s",
                                                                "Side length of a voxel, default 0.01");
  icl_core::config::GetoptParameter filter_threshold_parameter ("filter_threshold:", "f",
                                                                "Density filter threshold per voxel, default 1");
  icl_core::config::GetoptParameter erode_threshold_parameter  ("erode_threshold:", "e",
                                                                "Erode voxels with fewer occupied neighbors (percentage)");
  icl_core::config::addParameter(points_parameter);
  icl_core::config::addParameter(roll_parameter);
  icl_core::config::addParameter(pitch_parameter);
  icl_core::config::addParameter(yaw_parameter);
  icl_core::config::addParameter(voxel_side_length_parameter);
  icl_core::config::addParameter(filter_threshold_parameter);
  icl_core::config::addParameter(erode_threshold_parameter);
  icl_core::logging::initialize();

  voxel_side_length = icl_core::config::paramOptDefault<float>("voxel_side_length", 0.01f);

  // setup "tf" to transform from camera to world / gpu-voxels coordinates

  //const Vector3f camera_offsets(2, 0, 1); // camera located at y=0, x_max/2, z_max/2
  // const Vector3f camera_offsets(map_dimensions.x * voxel_side_length * 0.5f, -0.2f, map_dimensions.z * voxel_side_length * 0.5f); // camera located at y=-0.2m, x_max/2, z_max/2
  const Vector3f camera_offsets(-0.3, 0.7, 0.5);
  float roll = icl_core::config::paramOptDefault<float>("roll", 0.0f) * 3.141592f / 180.0f;
  float pitch = icl_core::config::paramOptDefault<float>("pitch", 90.0f) * 3.141592f / 180.0f;
  float yaw = icl_core::config::paramOptDefault<float>("yaw", 0.0f) * 3.141592f / 180.0f;
  tf = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(roll, pitch, yaw), camera_offsets);

  std::string point_cloud_topic = icl_core::config::paramOptDefault<std::string>("points-topic", "/camera/depth/color/points");
  LOGGING_INFO(Gpu_voxels, "DistanceROSDemo start. Point-cloud topic: " << point_cloud_topic << endl);

  gvl = GpuVoxels::getInstance();
  gvl->initialize(map_dimensions.x, map_dimensions.y, map_dimensions.z, voxel_side_length); // ==> 200 Voxels, each one is 1 cm in size so the map represents 20x20x20 centimeter

  //Vis Helper
  gvl->addPrimitives(primitive_array::ePRIM_SPHERE, "measurementPoints");

  // Add a map:
  gvl->addMap(MT_PROBAB_VOXELMAP, "myObjectVoxelmap");
  gvl->addMap(MT_BITVECTOR_VOXELLIST, "myHandVoxellist");
  gvl->addMap(MT_BITVECTOR_VOXELLIST, "myHandVoxellist_2");

  gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
  shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaDistanceVoxmap"));

  gvl->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap1");
  shared_ptr<ProbVoxelMap> erodeTempVoxmap1 = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("erodeTempVoxmap1"));
  gvl->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap2");
  shared_ptr<ProbVoxelMap> erodeTempVoxmap2 = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("erodeTempVoxmap2"));

  gvl->addMap(MT_COUNTING_VOXELLIST, "countingVoxelList");
  shared_ptr<CountingVoxelList> countingVoxelList = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelList"));

  gvl->addMap(MT_COUNTING_VOXELLIST, "countingVoxelListFiltered");
  shared_ptr<CountingVoxelList> countingVoxelListFiltered = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelListFiltered"));

  //PBA map clone for visualization without artifacts
  gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmapVisual");
  shared_ptr<DistanceVoxelMap> pbaDistanceVoxmapVisual = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaDistanceVoxmapVisual"));
  pbaDistanceVoxmapVisual->clearMap();

  // And a robot, generated from a ROS URDF file:
  gvl->addRobot("myUrdfRobot", "ur5_50/ur5.urdf", true);
  gvl->addRobot("myUrdfRobot2", "ur5_50/ur5_2.urdf", true);
  std::cout<<"sub  1 gogogogogog"<<std::endl;
  robot_sub = n.subscribe("/joint_states", 1, &CollisionWorldVoxel::jointStateCallback, this);
  // ros::Subscriber sub2 = n.subscribe("obstacle_pose", 1, obstaclePoseCallback);
  // ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >(point_cloud_topic, 1, &CollisionWorldVoxel::pointCloudCallback, this);
  std::cout<<"sub  2 gogogogogog"<<std::endl;
  point_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/camera/depth/color/points", 1, &CollisionWorldVoxel::pointCloudCallback, this);
  std::cout<<" gogogogogog"<<std::endl;
  // ros::NodeHandle n;&BaseModule::vectorMoveMsgCallback
  // ros::Subscriber sub1 = n.subscribe("joint_states", 1, jointStateCallback);
  // ros::Subscriber sub2 = n.subscribe("obstacle_pose", 1, obstaclePoseCallback);
  // ros::Subscriber sub = n.subscribe<pcl::Poiicl_coree ROS callback
  // gvl->insertPointCloudFromFile("myObjectVoxelmap", "hollie/hals_vereinfacht.binvox", true,
  //                               eBVM_OCCUPIED, false, object_position, 0.3);
  // update the robot joints:
  // gvl->setRobotConfiguration("myUrdfRobot", myRobotJointValues);
  // insert the robot into the map:
  // gvl->insertRobotIntoMap("myUrdfRobot", "myHandVoxellist", eBVM_OCCUPIED);
  // update the robot joints:
  // gvl->setRobotConfiguration("myUrdfRobot2", myRobotJointValues);
  // insert the robot into the map:
  // gvl->insertRobotIntoMap("myUrdfRobot2", "myHandVoxellist_2", eBVM_OCCUPIED);
  /*
   * Now we start the main loop, that will read ROS messages and update the Robot.
   */
  // Define two measurement points:
  // std::vector<Vector3i> measurement_points;
  // measurement_points.push_back(Vector3i(40, 100, 50));
  // measurement_points.push_back(Vector3i(160, 100, 50));
  // gvl->modifyPrimitives("measurementPoints", measurement_points, 5);

  int filter_threshold = icl_core::config::paramOptDefault<int>("filter_threshold", 0);
  std::cout << "Remove voxels containing less points than: " << filter_threshold << std::endl;

  float erode_threshold = icl_core::config::paramOptDefault<float>("erode_threshold", 0.0f);
  std::cout << "Erode voxels with neighborhood occupancy ratio less or equal to: " << erode_threshold << std::endl;
  iteration = 0;
  new_data_received = false; // call visualize on the first iteration

}  // end of namespace collision_detection

void CollisionWorldVoxel::pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  std::vector<Vector3f> point_data;
  point_data.resize(msg->points.size());

  for (uint32_t i = 0; i < msg->points.size(); i++)
  {
    point_data[i].x = msg->points[i].x;
    point_data[i].y = msg->points[i].y;
    point_data[i].z = msg->points[i].z;
  }

  //my_point_cloud.add(point_data);
  my_point_cloud.update(point_data);

  // transform new pointcloud to world coordinates
  my_point_cloud.transformSelf(&tf);
  
  new_data_received = true;

  LOGGING_INFO(Gpu_voxels, "DistanceROSDemo camera callback. PointCloud size: " << msg->points.size() << endl);
}

void CollisionWorldVoxel::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // std::cout << "Got JointStateMessage" << std::endl;
  gvl->clearMap("myHandVoxellist");
  gvl->clearMap("myHandVoxellist_2");

  for(size_t i = 0; i < msg->name.size(); i++)
  {
    myRobotJointValues[msg->name[i]] = msg->position[i];
  }
  // update the robot joints:
  gvl->setRobotConfiguration("myUrdfRobot", myRobotJointValues);
  // insert the robot into the map:
  gvl->insertRobotIntoMap("myUrdfRobot", "myHandVoxellist", eBVM_OCCUPIED);
  // update the robot joints:
  gvl->setRobotConfiguration("myUrdfRobot2", myRobotJointValues);
  // insert the robot into the map:
  gvl->insertRobotIntoMap("myUrdfRobot2", "myHandVoxellist_2", eBVM_OCCUPIED);
}
};