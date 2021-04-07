// 生成irm
// 用法： rosrun mm_map_creator mm_create_irm 500000
// 后面的数字代表需要加载的rm的名称

#include "mm_map_creator/head_file.h"

void convertPoseToRPY(const Eigen::Affine3d pose, std::vector<double>& end_pose_vector_and_manip) {
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << pose(0, 0), pose(0, 1), pose(0, 2), pose(1, 0), pose(1, 1), pose(1, 2), pose(2, 0), pose(2, 1),
      pose(2, 2);
  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
  end_pose_vector_and_manip[0] = pose(0, 3);
  end_pose_vector_and_manip[1] = pose(1, 3);
  end_pose_vector_and_manip[2] = pose(2, 3);
  end_pose_vector_and_manip[3] = euler_angles(2);  // roll
  end_pose_vector_and_manip[4] = euler_angles(1);  // pitch
  end_pose_vector_and_manip[5] = euler_angles(0);  // yaw
                                                   // 注意rpy的顺序
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mm_create_irm");
  ros::NodeHandle nh;

  std::string path(ros::package::getPath("mm_map_creator") + "/maps/");
  int max_point = atoi(argv[1]);
  std::string pcd_file = "mm_n" + std::to_string(max_point) + "_reachability.pcd";
  std::string pcd_filename = path + pcd_file;

  pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud(new pcl::PointCloud<pcl::PointNormal>);
  if (pcl::io::loadPCDFile<pcl::PointNormal>(pcd_filename, *rm_cloud) == -1) {
    ROS_ERROR("Can't load Reachability Map PCD file");
    return (-1);
  }
  ROS_INFO("Loaded PCD file successfully!");

  ros::Time start_time = ros::Time::now();

  pcl::PointCloud<pcl::PointNormal>::Ptr irm_cloud(new pcl::PointCloud<pcl::PointNormal>);

  for (int i = 0; i < rm_cloud->width; i++) {
    pcl::PointNormal pointnormal;
    geometry_msgs::Pose pose, inverse_pose;
    Eigen::Affine3d matrix, inverse_matrix;

    Eigen::Vector3d eulerAngle(rm_cloud->points[i].normal_z, rm_cloud->points[i].normal_y,
                               rm_cloud->points[i].normal_x);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle;

    pose.position.x = rm_cloud->points[i].x;
    pose.position.y = rm_cloud->points[i].y;
    pose.position.z = rm_cloud->points[i].z;
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    tf2::convert(pose, matrix);
    inverse_matrix = matrix.inverse();

    std::vector<double> end_pose_vector_and_manip(6);
    convertPoseToRPY(inverse_matrix, end_pose_vector_and_manip);

    pointnormal.x = end_pose_vector_and_manip[0];
    pointnormal.y = end_pose_vector_and_manip[1];
    pointnormal.z = end_pose_vector_and_manip[2];
    pointnormal.normal_x = end_pose_vector_and_manip[3];
    pointnormal.normal_y = end_pose_vector_and_manip[4];
    pointnormal.normal_z = end_pose_vector_and_manip[5];
    pointnormal.curvature = rm_cloud->points[i].curvature;
    irm_cloud->push_back(pointnormal);
  }

  ROS_INFO_STREAM("Transformation time is " << (ros::Time::now() - start_time).toSec());
  pcd_file = "mm_n" + std::to_string(max_point) + "_inverse_reachability.pcd";
  pcd_filename = path + pcd_file;
  ROS_INFO_STREAM("Saving PCD file to " << pcd_filename);
  pcl::io::savePCDFileASCII(pcd_filename, *irm_cloud);

  return 0;
}