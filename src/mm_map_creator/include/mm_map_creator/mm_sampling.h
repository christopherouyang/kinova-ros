#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "mm_map_creator/head_file.h"
#include "mm_map_creator/mm_collision_check.h"
#include "mm_map_creator/mm_kinematics.h"
#include "mm_map_creator/mm_inverse_kinematics.h"

namespace mm_sampling {
class mm_sampling {
 public:
  mm_sampling();
  bool rm_down_sample(pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud,
                      pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud_down_sample, int sample_number);
  bool makeCMbyCartesianSpaceSampling(pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud,
                                      pcl::PointCloud<pcl::PointNormal>::Ptr des_rm_cloud,
                                      double cm_translation_resolution, double cm_orientation_resolution);
  bool makeCMbyJointSapceSampling(pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud, int max_spl_pt);

 private:
  std::vector<std::string> joint_names;  // ur机器人的关节名称，用来从参数服务器加载ur机器人的关节极限
  std::vector<std::pair<double, double> > joint_limits;  // ur机器人的关节极限，用来限制ur机器人的关节采样
  mm_collision_check::mm_collision_check collision_check;  // 用来对特定的位型进行碰撞检测
  mm_kinematics::mm_kinematics kinematics;  // 正向运动学对象，用来计算齐次变换矩阵，雅克比矩阵
  mm_inverse_kinematics::mm_inverse_kinematics mik;  // 逆向运动学对象，用来计算逆解

  bool isProperIKExist(std::vector<double> target_pose_vector, std::vector<double> ur_ref_joints,
                       std::vector<double> agv_joint_values, std::vector<double>& joint_values);
  void joint_space_sampling(std::vector<double>& joint_values);
  int load_joint_names(std::vector<std::string>& joint_names);                   // 加载机器人名称
  int load_joint_limits(std::vector<std::pair<double, double> >& joint_limits);  // 加载机器人关节极限
  double mm_rand(double dmin, double dmax, int times);                           // 单个数值随机采样
  void GetAllRPYFromPoseMatrix(Eigen::Affine3d pose_matrix, std::vector<double>& rpy_angle);
  bool isPoseinROI(Eigen::Affine3d end_pose, std::vector<double>& end_pose_vector);
  void convertPoseToPoseVector(const Eigen::Affine3d pose, std::vector<double>& end_pose_vector);
};
}  // namespace mm_sampling

#endif