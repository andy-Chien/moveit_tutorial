#include <moveit_tutorials/collision_detection_fcll/collision_detector_fcl_plugin_loader.h>
#include <pluginlib/class_list_macros.h>

namespace collision_detection
{
bool CollisionDetectorFCLLPluginLoader::initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const
{
  scene->setActiveCollisionDetector(CollisionDetectorAllocatorFCLL::create(), exclusive);
  return true;
}
}  // namespace collision_detection

PLUGINLIB_EXPORT_CLASS(collision_detection::CollisionDetectorFCLLPluginLoader, collision_detection::CollisionPlugin)