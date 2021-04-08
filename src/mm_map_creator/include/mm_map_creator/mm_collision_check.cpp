#include "mm_map_creator/mm_collision_check.h"

namespace mm_collision_check {
mm_collision_check::mm_collision_check()
    : group("arm"),
      robot_model_loader("robot_description"),
      kinematic_model(robot_model_loader.getModel()),
      planning_scene(kinematic_model),
      kinematic_state(*group.getCurrentState()),
      joint_model_group(kinematic_model->getJointModelGroup("arm")) {
}

bool mm_collision_check::isCollision(std::vector<double> joint_values) {
  collision_request.group_name = "arm";
  collision_result.clear();
  kinematic_state.setJointGroupPositions(joint_model_group, joint_values);
  planning_scene.checkSelfCollision(collision_request, collision_result, kinematic_state);
  return collision_result.collision;
}
}  // namespace mm_collision_check
