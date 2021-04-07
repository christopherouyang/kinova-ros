#ifndef MM_COLLISION_CHECK_H
#define MM_COLLISION_CHECK_H

#include "mm_map_creator/head_file.h"

namespace mm_collision_check
{
    class mm_collision_check
    {
    public:

        mm_collision_check();

        bool isCollision(std::vector<double> joint_values);
    
    private:
        moveit::planning_interface::MoveGroupInterface group;
        robot_model_loader::RobotModelLoader robot_model_loader;
        robot_model::RobotModelPtr kinematic_model;
        planning_scene::PlanningScene planning_scene;
        moveit::core::RobotState kinematic_state;
        const robot_state::JointModelGroup* joint_model_group;

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        
    };
}
#endif