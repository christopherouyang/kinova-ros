// 对生成的rm进行降采样
// 用法： rosrun mm_map_creator mm_down_sampling 500000 10000
// 第一个参数是需要加载的rm的名称，第二个参数是保留的点数

#include "mm_map_creator/mm_sampling.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mm_load_reachability_map");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  mm_sampling::mm_sampling sampling;

  std::string path(ros::package::getPath("mm_map_creator") + "/maps/");
  std::string pcd_file = "mm_n" + std::to_string(*argv[1]) + "_reachability.pcd";
  std::string pcd_filename = path + pcd_file;
  pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud(new pcl::PointCloud<pcl::PointNormal>);
  if (pcl::io::loadPCDFile<pcl::PointNormal>(pcd_filename, *rm_cloud) == -1) {
    ROS_ERROR("Can't load Reachability Map PCD file");
    return (-1);
  }
  ROS_INFO("Loaded PCD file successfully!");
  ROS_INFO_STREAM("Point number is " << rm_cloud->width);

  pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud_down_sample(new pcl::PointCloud<pcl::PointNormal>);
  sampling.rm_down_sample(rm_cloud, rm_cloud_down_sample, atoi(argv[2]));
  pcd_file = "mm_n" + std::to_string(*argv[2]) + "_reachability.pcd";
  pcd_filename = path + pcd_file;
  ROS_INFO_STREAM("Saving PCD file to " << pcd_filename);
  pcl::io::savePCDFileASCII(pcd_filename, *rm_cloud_down_sample);
}