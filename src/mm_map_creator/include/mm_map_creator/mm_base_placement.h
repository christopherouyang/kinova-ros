#ifndef MM_BASE_PLACEMENT_H
#define MM_BASE_PLACEMENT_H

#include "mm_map_creator/mm_motion_planner.h"

namespace mm_base_placement {
class mm_base_placement {
 public:
  mm_base_placement();

  /******************************机器人基座放置相关函数******************************/
 public:
  bool load_rm(int point_number1);
  void get_orm(std::vector<double> eef_pose_vector, std::vector<double> filter_res,
               pcl::PointCloud<pcl::PointNormal>::Ptr base_pose_cloud, int method = 1);
  void get_orm_patch(std::vector<double> eef_pose_vector, std::vector<double> filter_res,
                     std::vector<double> current_agv_pose, pcl::PointCloud<pcl::PointNormal>::Ptr base_pose_cloud);

 private:
  mm_motion_planner::mm_motion_planner mmp;

  double ori_res{0.1}, trans_res{0.1}, agv_movement_delta;
  pcl::PassThrough<pcl::PointNormal> pass;
  pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud, des_rm_cloud;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  // 滤波函数
  void pcl_filter(double left_angle_limit, double right_angle_limit, const std::string& filed_name,
                  pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud);
  void pcl_filter2(double angle, const std::string& filed_name, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                   pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud);
  void roll_filter(double angle, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                   pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud);
  void pitch_filter(double angle, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud);
  void z_filter(double z, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud);
  void roll_filter2(double angle, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud);
  void pitch_filter2(double angle, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                     pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud);
  // 位姿相关的变换函数
  bool isPoseNearCurrentAgvPose(pcl::PointNormal point, std::vector<double> current_agv_pose, double threshold);
  void getAGVPosePointFromBasePosePoint(const Eigen::Affine3d target_pose_matrix, pcl::PointNormal& point);
  Eigen::Affine3d getPoseMatrixFromPointNormal(pcl::PointNormal point);
  std::vector<double> chooseProperAGVPoseVector(Eigen::Affine3d agv_pose_matrix);
};

}  // namespace mm_base_placement

#endif
