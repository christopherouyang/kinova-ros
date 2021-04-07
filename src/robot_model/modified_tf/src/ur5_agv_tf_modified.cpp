// 可以将符合DH规范的UR机器人的坐标系发布出去。但是其实另一个节点中可以直接根据DH参数求取正逆运动学

#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/JointState.h"
#include "tf2_kdl/tf2_kdl.h"
#include "angles/angles.h"
#define JOINT_NUMBER 9
#define PI 3.14159265354
#define FRENQUENCY 100
using namespace std;

sensor_msgs::JointState global_joint_states;

void JointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state) {
  for (int i = 0; i < JOINT_NUMBER; i++) {
    global_joint_states.position[i] = joint_state->position[i];
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mobile_manipulator_modified_tf");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("joint_states", 1000, JointStateCallback);
  global_joint_states.position.resize(JOINT_NUMBER);
  ros::Duration(1.0).sleep();

  // set the first frame
  tf2_ros::TransformBroadcaster tfb1;
  geometry_msgs::TransformStamped transformStamped1;
  transformStamped1.header.frame_id = "odom";
  transformStamped1.child_frame_id = "y_move";
  transformStamped1.transform.translation.x = 0.0;
  transformStamped1.transform.translation.y = 0.0;
  transformStamped1.transform.translation.z = 0.0;
  tf2::Quaternion q1;
  q1.setRPY(-PI / 2, -PI / 2, 0);
  transformStamped1.transform.rotation.x = q1.x();
  transformStamped1.transform.rotation.y = q1.y();
  transformStamped1.transform.rotation.z = q1.z();
  transformStamped1.transform.rotation.w = q1.w();

  // set the second frame
  tf2_ros::TransformBroadcaster tfb2;
  geometry_msgs::TransformStamped transformStamped2;
  transformStamped2.header.frame_id = "y_move";
  transformStamped2.child_frame_id = "x_move";
  transformStamped2.transform.translation.x = 0.0;
  transformStamped2.transform.translation.y = 0.0;
  transformStamped2.transform.translation.z = 0.0;
  tf2::Quaternion q2;
  q2.setRPY(-PI / 2, PI / 2, 0);
  transformStamped2.transform.rotation.x = q2.x();
  transformStamped2.transform.rotation.y = q2.y();
  transformStamped2.transform.rotation.z = q2.z();
  transformStamped2.transform.rotation.w = q2.w();

  // set the third frame
  tf2_ros::TransformBroadcaster tfb3;
  geometry_msgs::TransformStamped transformStamped3;
  transformStamped3.header.frame_id = "x_move";
  transformStamped3.child_frame_id = "rotation_move";
  transformStamped3.transform.translation.x = 0.0;
  transformStamped3.transform.translation.y = 0.0;
  transformStamped3.transform.translation.z = 0.0;
  tf2::Quaternion q3;

  std::vector<tf2_ros::TransformBroadcaster> tfb;
  tfb.resize(JOINT_NUMBER);
  std::vector<geometry_msgs::TransformStamped> transformStamped;
  transformStamped.resize(JOINT_NUMBER);

  std::vector<string> joint_names;
  joint_names.resize(JOINT_NUMBER);
  joint_names = {"shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link",
                 "wrist_2_link",  "wrist_3_link",   "tool0"};
  string prefix = "modified_";

  std::vector<tf2::Quaternion> q;
  std::vector<tf2::Vector3> offset;
  offset.resize(JOINT_NUMBER);
  q.resize(JOINT_NUMBER);

  for (int i = 0; i < JOINT_NUMBER; i++) {
    offset[i] = {0, 0, 0};
  }

  offset[3] = {0, 0.093, 0};
  offset[4] = {0, 0, 0.095};
  q[0].setRPY(0, 0, PI);
  q[1].setRPY(-PI / 2, PI / 2, 0);
  q[2].setRPY(-PI / 2, PI / 2, 0);
  q[3].setRPY(-PI / 2, 0, 0);
  q[4].setRPY(0, 0, 0);
  q[5].setRPY(-PI / 2, 0, 0);
  q[6].setRPY(0, 0, 0);

  for (int i = 0; i < JOINT_NUMBER; i++) {
    transformStamped[i].header.frame_id = joint_names[i];
    transformStamped[i].child_frame_id = prefix + joint_names[i];
    transformStamped[i].transform.translation.x = offset[i].x();
    transformStamped[i].transform.translation.y = offset[i].y();
    transformStamped[i].transform.translation.z = offset[i].z();
    transformStamped[i].transform.rotation.x = q[i].x();
    transformStamped[i].transform.rotation.y = q[i].y();
    transformStamped[i].transform.rotation.z = q[i].z();
    transformStamped[i].transform.rotation.w = q[i].w();
  }

  ros::Rate loop_rate(FRENQUENCY);
  while (ros::ok()) {
    transformStamped1.transform.translation.y = global_joint_states.position[1];
    transformStamped1.header.stamp = ros::Time::now();
    tfb1.sendTransform(transformStamped1);

    transformStamped2.transform.translation.y = global_joint_states.position[0];
    transformStamped2.header.stamp = ros::Time::now();
    tfb2.sendTransform(transformStamped2);

    q3.setRPY(PI / 2, -global_joint_states.position[2] + PI / 2, 0);
    transformStamped3.transform.rotation.x = q3.x();
    transformStamped3.transform.rotation.y = q3.y();
    transformStamped3.transform.rotation.z = q3.z();
    transformStamped3.transform.rotation.w = q3.w();
    transformStamped3.header.stamp = ros::Time::now();
    tfb3.sendTransform(transformStamped3);

    for (int i = 0; i < 6; i++) {
      transformStamped[i].header.stamp = ros::Time::now();
      tfb[i].sendTransform(transformStamped[i]);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}