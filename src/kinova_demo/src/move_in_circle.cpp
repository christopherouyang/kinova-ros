#include <ros/ros.h>
#include <kinova_msgs/ArmPoseGoal.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/ArmPoseActionResult.h>
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/ArmPoseActionGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

constexpr double DOOR_RADIUS = 0.565;
constexpr double DYAW_DT_WCHAIR_LINEAR = -0.14;
constexpr double DYAW_DT = -0.20;
constexpr double DYAW_DT_WCHAIR_ANGULAR = -0.10;
constexpr double RATE = 7;

double yaw_rate{0.0}, linear_vel{0.0};
double cur_yaw = M_PI;
bool pose_update = false;
geometry_msgs::PoseStamped curPose, startPose, goalPoseOdom, startPoseOdom, goalPoseArm;

void toolPoseCallback(const geometry_msgs::PoseStamped& msg) {
  curPose = pose_update ? curPose : msg;
  pose_update = true;
}

void velCallback(const geometry_msgs::Twist& cmd_vel) {
  linear_vel = cmd_vel.linear.x;
  yaw_rate = cmd_vel.angular.z;
}

void LogCurPose(const geometry_msgs::PoseStamped& msg) {
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO_THROTTLE(1.0, "Now the EEF pose(xyz-rpy) is (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", msg.pose.position.x,
                    msg.pose.position.y, msg.pose.position.z, roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
  pose_update = false;
}

void LogGoalPose(const kinova_msgs::ArmPoseGoal& msg) {
  double roll, pitch;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, cur_yaw);
  ROS_INFO_THROTTLE(0.5, "the goal pose(xyz-rpy) is (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", msg.pose.pose.position.x,
                    msg.pose.pose.position.y, msg.pose.pose.position.z, roll * 180 / M_PI, pitch * 180 / M_PI,
                    cur_yaw * 180 / M_PI);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "move_in_circle_node");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> arm_client("/j2s7s300_driver/pose_action/tool_pose");
  ROS_INFO("Waiting for action server to start.");
  arm_client.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> finger_client(
      "/j2s7s300_driver/fingers_action/finger_positions");
  ROS_INFO("Waiting for finger action server to start.");
  finger_client.waitForServer();
  ROS_INFO("Finger action server started, sending goal.");

  kinova_msgs::ArmPoseGoal poseGoal;
  ros::Subscriber poseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 1, toolPoseCallback);
  startPose.header.frame_id = "j2s7s300_link_base";
  startPose.pose.position.x = -0.3;
  startPose.pose.position.y = -0.6;
  startPose.pose.position.z = 0.46;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-M_PI / 2, M_PI / 6, M_PI), startPose.pose.orientation);
  poseGoal.pose = startPose;
  LogCurPose(curPose);
  arm_client.sendGoalAndWait(poseGoal);

  ros::Publisher fingerGoalPub = nh.advertise<kinova_msgs::SetFingersPositionActionGoal>(
      "/j2s7s300_driver/fingers_action/finger_positions/goal", 100);
  kinova_msgs::SetFingersPositionActionGoal fingerGoal;
  fingerGoal.goal.fingers.finger1 = 5000;
  fingerGoal.goal.fingers.finger2 = 5000;
  fingerGoal.goal.fingers.finger3 = 5000;
  fingerGoalPub.publish(fingerGoal);
  sleep(1);

  // kinova_msgs::PoseVelocity velMsg;
  // ros::Publisher velPub = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 100);

  ros::Publisher wchairVelPub = nh.advertise<geometry_msgs::Twist>("/wheelchair/cmd_vel", 1);
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = DOOR_RADIUS * DYAW_DT_WCHAIR_LINEAR;
  cmd_vel.angular.z = DYAW_DT_WCHAIR_ANGULAR;

  tf::TransformListener listener;
  listener.waitForTransform("odom", "j2s7s300_link_base", ros::Time::now(), ros::Duration(0.5));
  listener.transformPose("odom", startPose, startPoseOdom);

  ros::Subscriber wchairVelSub = nh.subscribe("/wheelchair/cur_vel", 1000, velCallback);

  double theta = 0.0;

  double start_roll, start_pitch, start_yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(startPoseOdom.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(start_roll, start_pitch, start_yaw);
  double delta_yaw = start_yaw + M_PI / 2;

  double x_chair{0.0}, y_chair{0.0}, yaw_chair{0.0};
  ros::Time current_time, last_time;
  ros::Rate loopRate(RATE);
  while (ros::ok() && theta > -M_PI / 2) {
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;
    x_chair += linear_vel * cos(yaw_chair) * dt;
    y_chair += linear_vel * sin(yaw_chair) * dt;
    yaw_chair += yaw_rate * dt;

    ROS_INFO_THROTTLE(2.0, "the wheehchair odom is (%.2f,%.2f,%.2f)", x_chair, y_chair, yaw_chair);

    LogCurPose(curPose);

    // double delta_x_wchair = -DOOR_RADIUS * std::sin(theta);
    // double delta_y_wchair = DOOR_RADIUS * (1 - std::cos(theta));
    // goalPoseOdom.pose.position.x =
    //     startPoseOdom.pose.position.x + delta_x_wchair * std::cos(delta_yaw) - delta_y_wchair * std::sin(delta_yaw);
    // goalPoseOdom.pose.position.y =
    //     startPoseOdom.pose.position.y + delta_x_wchair * std::sin(delta_yaw) + delta_y_wchair * std::cos(delta_yaw);

    // tf::quaternionTFToMsg(tf::createQuaternionFromRPY(start_roll, start_pitch, start_yaw - theta),
    //                       goalPoseOdom.pose.orientation);

    // listener.waitForTransform("odom", "j2s7s300_link_base", ros::Time::now(), ros::Duration(0.5));
    // listener.transformPose("j2s7s300_link_base", goalPoseOdom, goalPoseArm);

    // poseGoal.pose.header.frame_id = "j2s7s300_link_base";
    // poseGoal.pose.pose.position.x = goalPoseArm.pose.position.x;
    // poseGoal.pose.pose.position.y = goalPoseArm.pose.position.y;
    // poseGoal.pose.pose.orientation = goalPoseArm.pose.orientation;

    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-M_PI / 2, M_PI / 6, M_PI + theta - yaw_chair),
                          poseGoal.pose.pose.orientation);
    poseGoal.pose.pose.position.x = startPose.pose.position.x + DOOR_RADIUS * (1 - std::cos(theta)) - y_chair;
    poseGoal.pose.pose.position.y = startPose.pose.position.y - DOOR_RADIUS * std::sin(theta) + x_chair;
    arm_client.sendGoal(poseGoal);
    LogGoalPose(poseGoal);
    theta += DYAW_DT / RATE;
    // velMsg.twist_linear_x = DOOR_RADIUS * std::sin(cur_yaw) * DYAW_DT_ARM;
    // velMsg.twist_linear_y = -DOOR_RADIUS * std::cos(cur_yaw) * DYAW_DT_ARM;

    // velMsg.twist_angular_x = DYAW_DT_ARM / 2;
    // velMsg.twist_angular_y = DYAW_DT_ARM / 2 * sqrt(3);

    // velMsg.twist_linear_z = 0;
    // velMsg.twist_angular_z = 0;

    // ROS_INFO_STREAM_THROTTLE(2.0, "the vel in cartesian space is " << std::endl << velMsg);

    // velPub.publish(velMsg);

    wchairVelPub.publish(cmd_vel);
    ros::spinOnce();
    loopRate.sleep();
  }

  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  wchairVelPub.publish(cmd_vel);

  fingerGoal.goal.fingers.finger1 = 0;
  fingerGoal.goal.fingers.finger2 = 0;
  fingerGoal.goal.fingers.finger3 = 0;
  // LogCurPose(curPose);
  fingerGoalPub.publish(fingerGoal);

  return 0;
}