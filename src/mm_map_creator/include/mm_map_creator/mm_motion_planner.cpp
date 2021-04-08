#include "mm_map_creator/mm_motion_planner.h"

namespace mm_motion_planner {
Eigen::Affine3d mm_motion_planner::getPoseMatrixFromPoseVector(std::vector<double> pose_vector) {
  geometry_msgs::Pose pose;
  Eigen::Affine3d pose_matrix;
  pose.position.x = pose_vector[0];
  pose.position.y = pose_vector[1];
  pose.position.z = pose_vector[2];
  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_vector[3], pose_vector[4], pose_vector[5]);
  tf2::convert(pose, pose_matrix);
  return pose_matrix;
}

bool mm_motion_planner::chooseProperPoseVector(Eigen::Affine3d end_pose_matrix, std::vector<double>& end_pose_vector) {
  end_pose_vector[0] = end_pose_matrix(0, 3);
  end_pose_vector[1] = end_pose_matrix(1, 3);
  end_pose_vector[2] = end_pose_matrix(2, 3);

  std::vector<double> rpy_angle(6);
  GetAllRPYFromPoseMatrix(end_pose_matrix, rpy_angle);
  if (fabs(rpy_angle[1]) < fabs(rpy_angle[4]))  // 根据任务来的，如果工作位置变了，这个判断标准可能也需要修改
  {
    end_pose_vector[3] = rpy_angle[0];
    end_pose_vector[4] = rpy_angle[1];
    end_pose_vector[5] = rpy_angle[2];
  } else {
    end_pose_vector[3] = rpy_angle[3];
    end_pose_vector[4] = rpy_angle[4];
    end_pose_vector[5] = rpy_angle[5];
  }
  return true;
}

bool mm_motion_planner::chooseProperPoseVector(std::vector<double>& pose_vector) {
  Eigen::Affine3d pose_matrix = getPoseMatrixFromPoseVector(pose_vector);
  return chooseProperPoseVector(pose_matrix, pose_vector);
}

void mm_motion_planner::GetAllRPYFromPoseMatrix(Eigen::Affine3d pose_matrix, std::vector<double>& rpy_angle) {
  // atan2(y,x)
  double pitch1 = atan2(-pose_matrix(2, 0), -sqrt(1 - pose_matrix(2, 0) * pose_matrix(2, 0)));
  double roll1 = atan2(pose_matrix(2, 1) / cos(pitch1), pose_matrix(2, 2) / cos(pitch1));
  double yaw1 = atan2(pose_matrix(1, 0) / cos(pitch1), pose_matrix(0, 0) / cos(pitch1));

  double pitch2 = atan2(-pose_matrix(2, 0), sqrt(1 - pose_matrix(2, 0) * pose_matrix(2, 0)));
  double roll2 = atan2(pose_matrix(2, 1) / cos(pitch2), pose_matrix(2, 2) / cos(pitch2));
  double yaw2 = atan2(pose_matrix(1, 0) / cos(pitch2), pose_matrix(0, 0) / cos(pitch2));

  rpy_angle[0] = roll1;
  rpy_angle[1] = pitch1;
  rpy_angle[2] = yaw1;

  rpy_angle[3] = roll2;
  rpy_angle[4] = pitch2;
  rpy_angle[5] = yaw2;
}

geometry_msgs::Pose mm_motion_planner::getPoseFromPoseVector(std::vector<double> pose_vector) {
  geometry_msgs::Pose pose;
  pose.position.x = pose_vector[0];
  pose.position.y = pose_vector[1];
  pose.position.z = pose_vector[2];
  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_vector[3], pose_vector[4], pose_vector[5]);
  return pose;
}

/******************************插值函数******************************/
bool mm_motion_planner::cartesionInter(geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose,
                                       std::vector<std::vector<double> >& pose_vector) {
  std::vector<geometry_msgs::Pose> way_point_pose;
  linearInter(start_pose, target_pose, way_point_pose);
  pose_vector.resize(way_point_pose.size());

  Eigen::Affine3d pose_matrix;
  for (int i = 0; i < pose_vector.size(); i++) {
    tf2::convert(way_point_pose[i], pose_matrix);
    pose_vector[i].resize(6);
    chooseProperPoseVector(pose_matrix, pose_vector[i]);
  }
}

void mm_motion_planner::linearInter(geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose,
                                    std::vector<geometry_msgs::Pose>& way_point_pose) {
  std::vector<double> time_cost(3);
  double agv_max_vel = 0.1;
  double period_time = 0.05;
  time_cost[0] = fabs(target_pose.position.x - start_pose.position.x) / agv_max_vel;
  time_cost[1] = fabs(target_pose.position.y - start_pose.position.y) / agv_max_vel;
  time_cost[2] = fabs(target_pose.position.z - start_pose.position.z) / agv_max_vel;
  // 按照50ms的周期进行轨迹离散，因为运算取整的问题，最终的笛卡尔空间轨迹不是严格按照agv_max_vel以及50ms的间隔，不过实际控制时，仍可以按照50ms的周期运行
  int way_point_number = (*std::max_element(std::begin(time_cost), std::end(time_cost))) / period_time;
  way_point_pose.resize(way_point_number);
  orientationInter(start_pose, target_pose, way_point_pose);
  positionInter(start_pose, target_pose, way_point_pose);
}

void mm_motion_planner::orientationInter(geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose,
                                         std::vector<geometry_msgs::Pose>& way_point_pose) {
  tf::Quaternion start_quaternion, target_quaternion, inter_quaternion;
  tf::quaternionMsgToTF(start_pose.orientation, start_quaternion);
  tf::quaternionMsgToTF(target_pose.orientation, target_quaternion);
  std::vector<double> t_array(way_point_pose.size());
  for (int i = 0; i < t_array.size(); i++) {
    t_array[i] = double(i) / (t_array.size() - 1);
    inter_quaternion = start_quaternion.slerp(target_quaternion, t_array[i]);
    tf::quaternionTFToMsg(inter_quaternion, way_point_pose[i].orientation);
  }
}

void mm_motion_planner::positionInter(geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose,
                                      std::vector<geometry_msgs::Pose>& way_point_pose) {
  for (int i = 0; i < way_point_pose.size(); i++) {
    double t_array = double(i) / (way_point_pose.size() - 1);
    way_point_pose[i].position.x = start_pose.position.x + t_array * (target_pose.position.x - start_pose.position.x);
    way_point_pose[i].position.y = start_pose.position.y + t_array * (target_pose.position.y - start_pose.position.y);
    way_point_pose[i].position.z = start_pose.position.z + t_array * (target_pose.position.z - start_pose.position.z);
  }
}

}  // namespace mm_motion_planner
