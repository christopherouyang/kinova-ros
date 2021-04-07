#ifndef MM_MOTION_PLANNER_H
#define MM_MOTION_PLANNER_H

#include "mm_map_creator/head_file.h"

namespace mm_motion_planner
{
    class mm_motion_planner
    {
    public:
        bool chooseProperPoseVector(std::vector<double>& pose_vector);
        bool chooseProperPoseVector(Eigen::Affine3d end_pose_matrix, std::vector<double>& end_pose_vector);
        Eigen::Affine3d getPoseMatrixFromPoseVector(std::vector<double> pose_vector);
        geometry_msgs::Pose getPoseFromPoseVector(std::vector<double> pose_vector);
        void GetAllRPYFromPoseMatrix(Eigen::Affine3d pose_matrix, std::vector<double>& rpy_angle);

    /******************************插值函数******************************/
    public:
        bool cartesionInter(geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose, std::vector<std::vector<double> >& pose_vector);
    private:
        void linearInter(geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose, std::vector<geometry_msgs::Pose>& way_point_pose);
        void orientationInter(geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose, std::vector<geometry_msgs::Pose>& way_point_pose);
        void positionInter(geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose, std::vector<geometry_msgs::Pose>& way_point_pose);
    };
}

#endif