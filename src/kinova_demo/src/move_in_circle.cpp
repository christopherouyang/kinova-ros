#include <ros/ros.h>
#include <kinova_msgs/ArmPoseGoal.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/ArmPoseActionResult.h>
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/ArmPoseActionGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

constexpr double SIN_QUAT_PI = 0.70710678;
constexpr double DOOR_RADIUS = 0.4;

bool pose_update = false;
geometry_msgs::PoseStamped curPose;
geometry_msgs::PoseStamped startPose;

void toolPoseCallback(const geometry_msgs::PoseStamped& msg) {
  curPose = pose_update ? curPose : msg;
  pose_update = true;
}

void LogCurPose(const geometry_msgs::PoseStamped& msg) {
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO_THROTTLE(0.5, "Now the EEF pose(xyz-rpy) is (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", msg.pose.position.x,
                    msg.pose.position.y, msg.pose.position.z, roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
}

void LogGoalPose(const kinova_msgs::ArmPoseGoal& msg) {
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO_THROTTLE(0.5, "the goal pose(xyz-rpy) is (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", msg.pose.pose.position.x,
                    msg.pose.pose.position.y, msg.pose.pose.position.z, roll * 180 / M_PI, pitch * 180 / M_PI,
                    yaw * 180 / M_PI);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "move_in_circle_node");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> client("/j2s7s300_driver/pose_action/tool_pose");
  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  kinova_msgs::ArmPoseGoal poseGoal;
  kinova_msgs::ArmPoseActionGoal poseActionGoal;
  poseActionGoal.header.frame_id = "j2s7s300_link_base";
  ros::Publisher poseGoalPub =
      nh.advertise<kinova_msgs::ArmPoseActionGoal>("/j2s7s300_driver/pose_action/tool_pose/goal", 100);
  ros::Subscriber poseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 1, toolPoseCallback);
  startPose.header.frame_id = "j2s7s300_link_base";
  startPose.pose.position.x = 0.05;
  startPose.pose.position.y = -0.4;
  startPose.pose.position.z = 0.46;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-M_PI / 2, M_PI / 6, M_PI), startPose.pose.orientation);
  poseGoal.pose = startPose;
  poseActionGoal.goal = poseGoal;
  poseActionGoal.header.seq = 0;
  poseActionGoal.header.stamp = ros::Time::now();
  poseActionGoal.goal_id.id = "j2s7s300_tool_pose";
  poseActionGoal.goal_id.stamp = ros::Time::now();
  LogGoalPose(poseGoal);
  // poseGoalPub.publish(poseActionGoal);
  client.sendGoalAndWait(poseGoal);
  pose_update = false;
  ros::Rate loop_rate(10);
  while (!pose_update) {
    ros::spinOnce();
    ROS_WARN_THROTTLE(2.0, "wait for the tool pose callback");
    loop_rate.sleep();
  }

  double theta = 0.0;
  while (ros::ok() && theta < M_PI / 4) {
    pose_update = false;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-M_PI / 2, M_PI / 6, M_PI - theta),
                          poseGoal.pose.pose.orientation);
    poseGoal.pose.pose.position.x = startPose.pose.position.x + DOOR_RADIUS * (1 - std::cos(theta));
    poseGoal.pose.pose.position.y = startPose.pose.position.y + DOOR_RADIUS * std::sin(theta);
    client.sendGoal(poseGoal);
    LogGoalPose(poseGoal);
    LogCurPose(curPose);
    theta += M_PI / 315;
    ros::spinOnce();
    loop_rate.sleep();
  }

  LogGoalPose(poseGoal);

  return 0;
}