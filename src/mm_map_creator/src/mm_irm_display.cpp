// irm数据显示
// 用法： rosrun mm_map_creator mm_irm_display 500000
// 参数代表的是irm中的点数
#include "mm_map_creator/mm_discretization.h"
#include "mm_map_creator/mm_display.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mm_rm_display");
  ros::NodeHandle nh;

  std::string path(ros::package::getPath("mm_map_creator") + "/maps/");
  int max_point;
  if (argc == 2) {
    ROS_INFO("You have specified maximum sampling number!");
    max_point = atoi(argv[1]);
  } else {
    ROS_INFO("Use default parameter");
    ros::param::get("default_values/max_sampling_point", max_point);
  }

  std::string pcd_file = "mm_n" + std::to_string(max_point) + "_inverse_reachability.pcd";
  std::string pcd_filename = path + pcd_file;

  pcl::PointCloud<pcl::PointNormal>::Ptr irm_cloud(new pcl::PointCloud<pcl::PointNormal>);
  if (pcl::io::loadPCDFile<pcl::PointNormal>(pcd_filename, *irm_cloud) == -1) {
    ROS_ERROR("Can't load Reachability Map PCD file");
    return (-1);
  }
  ROS_INFO("Loaded PCD file successfully!");
  ROS_INFO_STREAM("Point number is " << irm_cloud->width);

  double translation_max_range = 2.5;
  double new_resolution = 0.1f;

  mm_display::mm_display display;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr irm_cloud_display(new pcl::PointCloud<pcl::PointXYZRGB>);
  display.display_irm(irm_cloud, translation_max_range, new_resolution, irm_cloud_display);

  irm_cloud_display->header.frame_id = "agv_base_link";
  ROS_INFO_STREAM("Point cloud size is: " << irm_cloud_display->width);

  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("reachability_map", 1);

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    ROS_INFO("Publishing point cloud!");
    pcl_conversions::toPCL(ros::Time::now(), irm_cloud_display->header.stamp);
    pub.publish(irm_cloud_display);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}