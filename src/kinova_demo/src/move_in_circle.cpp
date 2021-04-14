#include <ros/ros.h>
#include <kinova_msgs/ArmPoseGoal.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/ArmPoseActionResult.h>
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/ArmPoseActionGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

constexpr double DOOR_RADIUS = 0.4;
constexpr double DYAW_DT = -0.15;

double cur_yaw = M_PI;
bool pose_update = false;
geometry_msgs::PoseStamped curPose;
geometry_msgs::PoseStamped startPose;

void toolPoseCallback(const geometry_msgs::PoseStamped& msg) {
  curPose = pose_update ? curPose : msg;
  pose_update = true;
}

void LogCurPose(const geometry_msgs::PoseStamped& msg) {
  double roll, pitch;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, cur_yaw);
  ROS_INFO_THROTTLE(1.0, "Now the EEF pose(xyz-rpy) is (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", msg.pose.position.x,
                    msg.pose.position.y, msg.pose.position.z, roll * 180 / M_PI, pitch * 180 / M_PI,
                    cur_yaw * 180 / M_PI);
  pose_update = false;
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
  startPose.pose.position.x = -0.1;
  startPose.pose.position.y = -0.1;
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
  LogCurPose(curPose);

  ros::Rate loopRate(100.0);
  kinova_msgs::PoseVelocity velMsg;
  ros::Publisher velPub = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 100);

  ROS_INFO("cur_yaw is %.3f, threshold is %.3f", cur_yaw, 3 * M_PI / 4);
  while (ros::ok() && (cur_yaw > 2 * M_PI / 3 || cur_yaw < -2 * M_PI / 3)) {
    LogCurPose(curPose);
    ROS_INFO_THROTTLE(2.0, "cur_yaw is %.3f, threshold is %.3f", cur_yaw, 3 * M_PI / 4);
    // tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-M_PI / 2, M_PI / 6, M_PI - theta),
    //                       poseGoal.pose.pose.orientation);
    // poseGoal.pose.pose.position.x = startPose.pose.position.x + DOOR_RADIUS * (1 - std::cos(theta));
    // poseGoal.pose.pose.position.y = startPose.pose.position.y + DOOR_RADIUS * std::sin(theta);
    // client.sendGoal(poseGoal);
    // LogGoalPose(poseGoal);
    velMsg.twist_linear_x = DOOR_RADIUS * std::sin(cur_yaw) * DYAW_DT;
    velMsg.twist_linear_y = -DOOR_RADIUS * std::cos(cur_yaw) * DYAW_DT;

    velMsg.twist_angular_x = -DYAW_DT / 2;
    velMsg.twist_angular_y = -DYAW_DT / 2 * sqrt(3);

    velMsg.twist_linear_z = 0;
    velMsg.twist_angular_z = 0;

    ROS_INFO_STREAM_THROTTLE(2.0, "the vel in cartesian space is " << std::endl << velMsg);

    velPub.publish(velMsg);
    ros::spinOnce();
    loopRate.sleep();
  }
  ROS_INFO_THROTTLE(2.0, "cur_yaw is %.3f, threshold is %.3f", cur_yaw, 2 * M_PI / 3);

  LogCurPose(curPose);

  return 0;
}