// 生成rm
// 用法1，关节空间采样： rosrun mm_map_creator mm_creator_reachability_map 500000
// 后面的参数代表rm中存储的点数
// 用法2，笛卡尔空间采样： rosrun mm_map_creator mm_creator_reachability_map
// 其中点的数量以及采样区域有mm_sampling中的参数确定

#include "mm_map_creator/mm_sampling.h"
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mm_create_reachability_map");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  ros::Time start_time = ros::Time::now();
  std::string path(ros::package::getPath("mm_map_creator") + "/maps/");
  mm_sampling::mm_sampling sampling;

  if (argc == 2) {
    pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud(new pcl::PointCloud<pcl::PointNormal>);
    int max_point = atoi(argv[1]);
    ROS_INFO_STREAM("Creating CM by joint space sampling and the point is: " << max_point);
    sampling.makeCMbyJointSapceSampling(rm_cloud, max_point);
    std::string pcd_file = "mm_n" + std::to_string(rm_cloud->width) + "_reachability.pcd";
    std::string pcd_filename = path + pcd_file;
    ROS_INFO_STREAM("Saving PCD file to " << pcd_filename);
    pcl::io::savePCDFileASCII(pcd_filename, *rm_cloud);
    ROS_INFO_STREAM("RM_cloud number is " << rm_cloud->width);
    ROS_INFO_STREAM("Total time is: " << (ros::Time::now() - start_time).toSec());
  } else {
    std::vector<double> trans_vector = {7, 6, 5, 4, 3, 2};
    std::vector<double> rot_vector = {7, 6, 5, 4, 3, 2};
    for (int i = 0; i < trans_vector.size(); i++) {
      pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud(new pcl::PointCloud<pcl::PointNormal>);
      pcl::PointCloud<pcl::PointNormal>::Ptr des_rm_cloud(new pcl::PointCloud<pcl::PointNormal>);
      sampling.makeCMbyCartesianSpaceSampling(rm_cloud, des_rm_cloud, trans_vector[i], rot_vector[i]);

      std::string pcd_file = "mm_n" + std::to_string(rm_cloud->width) + "_reachability.pcd";
      std::string pcd_filename = path + pcd_file;
      ROS_INFO_STREAM("Saving PCD file to " << pcd_filename);
      pcl::io::savePCDFileASCII(pcd_filename, *rm_cloud);

      pcd_file = "mm_n" + std::to_string(des_rm_cloud->width) + "_des.pcd";
      pcd_filename = path + pcd_file;
      ROS_INFO_STREAM("Saving PCD file to " << pcd_filename);
      pcl::io::savePCDFileASCII(pcd_filename, *des_rm_cloud);
    }
  }
  return 0;
}