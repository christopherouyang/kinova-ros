#include <ros/ros.h>
#include <ros/service_client.h>
#include <kinova_msgs/ArmPoseGoal.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/ArmPoseActionResult.h>
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/JointTorque.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/ArmPoseActionGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <kinova_msgs/Start.h>
#include <kinova_msgs/Stop.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <common/RosLogger.h>
#include <sys/stat.h>
#include <sys/types.h>

constexpr double START_ROLL_ARM = -M_PI / 2;
constexpr double START_PITCH_ARM = M_PI / 2;
constexpr double START_YAW_ARM = M_PI / 2;

double cur_tool_yaw{0.0};
geometry_msgs::PoseStamped toolPose, toolPoseOdom, wchairPoseOdom, wchairOriginPose, goalPoseOdom, goalPoseArm,
    startPose, startPoseOdom;

void toolPoseCallback(const geometry_msgs::PoseStamped& msg) {
  toolPose = msg;
  double roll, pitch;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, cur_tool_yaw);
  ROS_INFO_THROTTLE(1.0, "the curr pose(xyz-rpy) in arm coordinate is (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll * 180 / M_PI,
                    pitch * 180 / M_PI, cur_tool_yaw * 180 / M_PI);
}

void LogGoalPose(const kinova_msgs::ArmPoseGoal& msg) {
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO_THROTTLE(1.0, "the goal pose(xyz-rpy) is (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f),%.2f, %.2f, %.2f, %.2f",
                    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, roll * 180 / M_PI,
                    pitch * 180 / M_PI, yaw * 180 / M_PI, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "right_door_open");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> arm_client("/j2s7s300_driver/pose_action/tool_pose");
  ROS_INFO("Waiting for arm action server to start.");
  arm_client.waitForServer();
  ROS_INFO("Arm action server started, sending goal.");

  ros::Subscriber poseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 1, toolPoseCallback);
  ros::Publisher fingerGoalPub = nh.advertise<kinova_msgs::SetFingersPositionActionGoal>(
      "/j2s7s300_driver/fingers_action/finger_positions/goal", 100);
  ros::Publisher wchairVelPub = nh.advertise<geometry_msgs::Twist>("/wheelchair/cmd_vel", 1);

  ros::Duration(0.5).sleep();
  ros::spinOnce();

  startPose.header.frame_id = "j2s7s300_link_base";
  startPose.pose.position.x = 0.2;
  startPose.pose.position.y = -0.6;
  startPose.pose.position.z = 0.52;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(START_ROLL_ARM, START_PITCH_ARM, START_YAW_ARM),
                        startPose.pose.orientation);
  kinova_msgs::ArmPoseGoal poseGoal;
  poseGoal.pose = startPose;
  LogGoalPose(poseGoal);
  arm_client.sendGoalAndWait(poseGoal);
  ros::spinOnce();
  ROS_INFO("Move the end effector to the right door");

  return 0;
}