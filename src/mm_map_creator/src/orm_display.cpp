// 这个程序是使用改进之后的方法进行计算基座的位置，并进行数据显示，计算速度快
#include "mm_map_creator/head_file.h"
#include "mm_map_creator/mm_display.h"
#include "mm_map_creator/mm_kinematics.h"
#include <sensor_msgs/JointState.h>

std::vector<double> kinova_joint_state(JOINT_NUMBER, 0.0);

void joint_state_callback(sensor_msgs::JointStateConstPtr state) {
  for (int i = 0; i < JOINT_NUMBER; ++i)
    kinova_joint_state[i] = state->position[i];
  kinova_joint_state[0] -= PI;
  kinova_joint_state[6] -= PI / 2;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "orm_display");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  ros::ServiceClient orm_calculation_client = nh.serviceClient<mm_map_creator::orm_calculation>("orm_calculation");
  mm_kinematics::mm_kinematics kine;
  mm_map_creator::orm_calculation orm_cal;

  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 50, joint_state_callback);
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  orm_cal.request.method = 1;
  orm_cal.request.current_agv_pose = std::vector<double>(3, 0.0);
  orm_cal.request.target_eef_pose_vector = kine.GetTotalEndPose(kinova_joint_state);

  ros::Time start_time = ros::Time::now();
  orm_calculation_client.waitForExistence();
  orm_calculation_client.call(orm_cal);
  ROS_INFO_STREAM("Time cost: " << (ros::Time::now() - start_time).toSec() * 1000 << "ms");

  pcl::PointCloud<pcl::PointNormal>::Ptr base_pose_cloud(new pcl::PointCloud<pcl::PointNormal>);
  for (size_t i = 0; i < orm_cal.response.base_pose_cloud.size() / 4.0; i++) {
    pcl::PointNormal point;
    point.x = orm_cal.response.base_pose_cloud[4 * i + 0];
    point.y = orm_cal.response.base_pose_cloud[4 * i + 1];
    point.z = point.normal_x = point.normal_y = 0;
    point.normal_z = orm_cal.response.base_pose_cloud[4 * i + 2];
    point.curvature = orm_cal.response.base_pose_cloud[4 * i + 3];
    base_pose_cloud->push_back(point);
  }
  ROS_INFO_STREAM("base_pose_cloud size: " << base_pose_cloud->points.size());

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  visualization_msgs::MarkerArray marker_array;
  mm_display::mm_display display;
  display.display_arrow(base_pose_cloud, marker_array, base_pose_cloud->points.size());

  ros::Rate loop_rate(0.2);
  while (ros::ok()) {
    for (int i = 0; i < 2; i++) {
      marker_pub.publish(marker_array);
      ros::Duration(0.5).sleep();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}