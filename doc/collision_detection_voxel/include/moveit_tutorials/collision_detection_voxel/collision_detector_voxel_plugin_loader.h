/*
 * collision_detector_voxel_plugin_loader.h
 */

#ifndef MOVEIT_COLLISION_DETECTION_VOXEL_COLLISION_DETECTOR_VOXEL_PLUGIN_LOADER_H_
#define MOVEIT_COLLISION_DETECTION_VOXEL_COLLISION_DETECTOR_VOXEL_PLUGIN_LOADER_H_

#include <moveit/collision_detection/collision_plugin.h>
#include <moveit_tutorials/collision_detection_voxel/collision_detector_allocator_voxel.h>

namespace collision_detection
{
class CollisionDetectorVoxelPluginLoader : public CollisionPlugin
{
public:
  virtual bool initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const;
};
}  // namespace collision_detection
#endif  // MOVEIT_COLLISION_DETECTION_VOXEL_COLLISION_DETECTOR_VOXEL_PLUGIN_LOADER_H_
