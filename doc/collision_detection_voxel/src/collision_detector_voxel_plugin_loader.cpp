#include <moveit_tutorials/collision_detection_voxel/collision_detector_voxel_plugin_loader.h>
#include <pluginlib/class_list_macros.h>

namespace collision_detection
{
bool CollisionDetectorVoxelPluginLoader::initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const
{
  scene->setActiveCollisionDetector(CollisionDetectorAllocatorVoxel::create(), exclusive);
  return true;
}
}  // namespace collision_detection

PLUGINLIB_EXPORT_CLASS(collision_detection::CollisionDetectorVoxelPluginLoader, collision_detection::CollisionPlugin)
