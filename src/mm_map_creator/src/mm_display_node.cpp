// rm数据显示
// 用法: rosrun mm_map_creator mm_display_node r 500000
// 参数代表rm中的点数

#include "mm_map_creator/mm_discretization.h"
#include "mm_map_creator/mm_display.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mm_display_node");
  ros::NodeHandle nh;

  std::string path(ros::package::getPath("mm_map_creator") + "/maps/");
  std::string map = *argv[1] == 'r' ? "_rm.pcd" : "_irm.pcd";

  std::string pcd_file = "mm_n" + std::to_string(atoi(argv[2])) + "_rm.pcd";
  std::string pcd_filename = path + pcd_file;

  pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud(new pcl::PointCloud<pcl::PointNormal>);
  if (pcl::io::loadPCDFile<pcl::PointNormal>(pcd_filename, *rm_cloud) == -1) {
    ROS_ERROR("Can't load Reachability Map PCD file");
    return -1;
  }
  ROS_INFO("Loaded PCD file successfully!");
  ROS_INFO_STREAM("Point number is " << rm_cloud->width);

  double translation_max_range = 4.5;
  double new_resolution = 0.08f;

  mm_display::mm_display display;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rm_cloud_display(new pcl::PointCloud<pcl::PointXYZRGB>);
  display.display_map(rm_cloud, translation_max_range, new_resolution, rm_cloud_display);

  rm_cloud_display->header.frame_id = *argv[1] == 'r' ? "robot_root" : "root";
  ROS_INFO_STREAM("Point cloud size is: " << rm_cloud_display->width);

  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("reachability_map", 1);

  // std::vector<double> d = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8,
  // 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8}; std::vector<pcl::PointCloud<pcl::PointXYZRGB>>
  // temp_cloud(d.size() - 1);

  // for (int i = 0; i < rm_cloud_display->width; i++) {
  //   double distance = sqrt(pow(rm_cloud_display->points[i].x, 2) + pow(rm_cloud_display->points[i].y, 2) +
  //                          pow(rm_cloud_display->points[i].z, 2));
  //   for (int j = 0; j < d.size() - 1; j++) {
  //     if (distance <= d[j + 1] && distance > d[j]) {
  //       temp_cloud[j].push_back(rm_cloud_display->points[i]);
  //     }
  //   }
  // }
  // rm_cloud_display->points.clear();
  // int k = 0;

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    // if (k < temp_cloud.size()) {
    //   ROS_INFO("Publishing point cloud!");
    //   *rm_cloud_display = *rm_cloud_display + temp_cloud[k];
    //   k++;
    // }
    pcl_conversions::toPCL(ros::Time::now(), rm_cloud_display->header.stamp);
    pub.publish(rm_cloud_display);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}