// 接受ur机器人的数据,并发布到/joint_states上面
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
ros::Subscriber ur_joint_sub;
ros::Publisher joint_state_pub;
void urJointsubCallback(const sensor_msgs::JointStateConstPtr& joint_states) {
  joint_state_pub.publish(*joint_states);
}
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ur_joint_state_convert");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  ur_joint_sub = nh.subscribe("/ur_joint_states", 1000, urJointsubCallback);
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
  ros::waitForShutdown();
  return 0;
}
