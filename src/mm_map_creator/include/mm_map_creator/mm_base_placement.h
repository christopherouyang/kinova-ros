#ifndef MM_BASE_PLACEMENT_H
#define MM_BASE_PLACEMENT_H

#include "mm_map_creator/mm_motion_planner.h"

namespace mm_base_placement
{
    class mm_base_placement
    {
    public:
        mm_base_placement();

    /******************************机器人基座放置相关函数******************************/
    public:
        bool load_rm(int point_number1, int point_number2);
        bool get_orm(std::vector<double> eef_pose_vector, pcl::PointCloud< pcl::PointNormal >::Ptr base_pose_cloud, int method = 1);
        bool get_orm_patch(std::vector<double> eef_pose_vector, std::vector<double> current_agv_pose, pcl::PointCloud< pcl::PointNormal >::Ptr base_pose_cloud);
    private:

        mm_motion_planner::mm_motion_planner mmp;

        double filter_orientation_resolution, filter_translation_resolution;
        pcl::PassThrough<pcl::PointNormal> pass;
        pcl::PointCloud< pcl::PointNormal >::Ptr rm_cloud;
        pcl::PointCloud< pcl::PointNormal >::Ptr des_rm_cloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        double agv_movement_delta;
        // 滤波函数
        void roll_filter(double angle, pcl::PointCloud< pcl::PointNormal >::Ptr source_cloud, pcl::PointCloud< pcl::PointNormal >::Ptr target_cloud);
        void pitch_filter(double angle, pcl::PointCloud< pcl::PointNormal >::Ptr source_cloud, pcl::PointCloud< pcl::PointNormal >::Ptr target_cloud);
        void z_filter(double z, pcl::PointCloud< pcl::PointNormal >::Ptr source_cloud, pcl::PointCloud< pcl::PointNormal >::Ptr target_cloud);
        void roll_filter2(double angle, pcl::PointCloud< pcl::PointNormal >::Ptr source_cloud, pcl::PointCloud< pcl::PointNormal >::Ptr target_cloud);
        void pitch_filter2(double angle, pcl::PointCloud< pcl::PointNormal >::Ptr source_cloud, pcl::PointCloud< pcl::PointNormal >::Ptr target_cloud);
        // 位姿相关的变换函数
        bool isPoseNearCurrentAgvPose(pcl::PointNormal point, std::vector<double> current_agv_pose, double threshold);
        void getAGVPosePointFromBasePosePoint(const Eigen::Affine3d target_pose_matrix, pcl::PointNormal& point);
        Eigen::Affine3d getPoseMatrixFromPointNormal(pcl::PointNormal point);
        std::vector<double> chooseProperAGVPoseVector(Eigen::Affine3d agv_pose_matrix);
    };
    
}

#endif
