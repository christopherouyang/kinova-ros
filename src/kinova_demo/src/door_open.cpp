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

constexpr double R_DOOR_RADIUS = 0.67;
constexpr double DOOR_RADIUS = 0.51;
constexpr double DOOR_HANDLE_R = 0.083;
constexpr double PULL_VEL = -0.1;
constexpr double PULL_VEL_WCHAIR_LINEAR = PULL_VEL * 0.45;
constexpr double PULL_VEL_WCHAIR_ANGULAR = PULL_VEL * 0.0;
constexpr double ROTATE_VEL = 0.35;
constexpr double RATE = 6;
constexpr double START_ROLL_ARM = -M_PI / 2;
constexpr double START_PITCH_ARM = M_PI / 6;
constexpr double START_YAW_ARM = M_PI;

unsigned long long startTimeMs{0};
double cur_tool_yaw{0.0};
geometry_msgs::PoseStamped toolPose, toolPoseOdom, wchairPoseOdom, wchairOriginPose, goalPoseOdom, goalPoseArm,
    startPose, startPoseOdom;
std::ofstream armPoseFile, armOdomPoseFile, wchairPoseFile, jointTorqueFile, toolWrenchFile;

void toolPoseCallback(const geometry_msgs::PoseStamped& msg) {
  toolPose = msg;
  double roll, pitch;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, cur_tool_yaw);
  ROS_INFO_THROTTLE(1.0, "Now the EEF pose(xyz-rpy) in arm coordinate is (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll * 180 / M_PI,
                    pitch * 180 / M_PI, cur_tool_yaw * 180 / M_PI);

  int timeNow = static_cast<int>(COMMON::TimeUtil::GetTimestampMs() - startTimeMs);
  armPoseFile << timeNow << ',' << msg.pose.position.x << ',' << msg.pose.position.y << ',' << msg.pose.position.z
              << ',' << roll * 180 / M_PI << ',' << pitch * 180 / M_PI << ',' << cur_tool_yaw * 180 / M_PI << ','
              << std::endl;
}

void toolWrenchCallback(const geometry_msgs::WrenchStamped& msg) {
  int timeNow = static_cast<int>(COMMON::TimeUtil::GetTimestampMs() - startTimeMs);
  toolWrenchFile << timeNow << ',' << msg.wrench.force.x << ',' << msg.wrench.force.y << ',' << msg.wrench.force.z
                 << ',' << msg.wrench.torque.x << ',' << msg.wrench.torque.y << ',' << msg.wrench.torque.z << ','
                 << std::endl;
}

void jointTorqueCallback(const kinova_msgs::JointTorque& msg) {
  int timeNow = static_cast<int>(COMMON::TimeUtil::GetTimestampMs() - startTimeMs);
  jointTorqueFile << timeNow << ',' << msg.joint1 << ',' << msg.joint2 << ',' << msg.joint3 << ',' << msg.joint4 << ','
                  << msg.joint5 << ',' << msg.joint6 << ',' << msg.joint7 << ',' << std::endl;
}

void LogGoalPose(const kinova_msgs::ArmPoseGoal& msg) {
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO_THROTTLE(1.0, "the goal pose(xyz-rpy) is (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", msg.pose.pose.position.x,
                    msg.pose.pose.position.y, msg.pose.pose.position.z, roll * 180 / M_PI, pitch * 180 / M_PI,
                    yaw * 180 / M_PI);
}

void TransformPose(const tf::TransformListener& arm_listener, const tf::TransformListener& wchair_listener) {
  try {
    arm_listener.waitForTransform("odom", "j2s7s300_link_base", ros::Time::now(), ros::Duration(0.5));
    arm_listener.transformPose("j2s7s300_link_base", goalPoseOdom, goalPoseArm);
    arm_listener.transformPose("odom", toolPose, toolPoseOdom);
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
  }

  try {
    wchair_listener.waitForTransform("odom", "base_footprint", ros::Time::now(), ros::Duration(0.5));
    wchair_listener.transformPose("odom", wchairOriginPose, wchairPoseOdom);
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
  }
}

void LogCurrentPose() {
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(toolPoseOdom.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  int timeNow = static_cast<int>(COMMON::TimeUtil::GetTimestampMs() - startTimeMs);
  armOdomPoseFile << timeNow << ',' << toolPoseOdom.pose.position.x << ',' << toolPoseOdom.pose.position.y << ','
                  << toolPoseOdom.pose.position.z << ',' << roll * 180 / M_PI << ',' << pitch * 180 / M_PI << ','
                  << yaw * 180 / M_PI << ',' << std::endl;

  yaw = tf::getYaw(wchairPoseOdom.pose.orientation);
  timeNow = static_cast<int>(COMMON::TimeUtil::GetTimestampMs() - startTimeMs);
  wchairPoseFile << timeNow << ',' << wchairPoseOdom.pose.position.x << ',' << wchairPoseOdom.pose.position.y << ','
                 << yaw * 180 / M_PI << ',' << std::endl;
}

void ConfigStartPoseOdom(const tf::TransformListener& arm_listener, const tf::TransformListener& wchair_listener,
                         std::vector<double>& start_pos, std::vector<double>& start_ori) {
  try {
    wchair_listener.waitForTransform("odom", "base_footprint", ros::Time::now(), ros::Duration(1.0));
    wchair_listener.transformPose("odom", wchairOriginPose, wchairPoseOdom);
    wchair_listener.transformPose("odom", wchairOriginPose, wchairPoseOdom);
    arm_listener.waitForTransform("odom", "j2s7s300_link_base", ros::Time::now(), ros::Duration(1.0));
    arm_listener.transformPose("odom", startPose, startPoseOdom);
    arm_listener.transformPose("odom", startPose, startPoseOdom);
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
  }

  tf::Quaternion quat;
  tf::quaternionMsgToTF(startPoseOdom.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(start_ori[0], start_ori[1], start_ori[2]);
  start_pos[0] = startPoseOdom.pose.position.x;
  start_pos[1] = startPoseOdom.pose.position.y;
}

void CloseFinger(const ros::Publisher& fingerGoalPub) {
  kinova_msgs::SetFingersPositionActionGoal fingerGoal;
  fingerGoal.goal.fingers.finger1 = 5500;
  fingerGoal.goal.fingers.finger2 = 5500;
  fingerGoal.goal.fingers.finger3 = 5700;
  fingerGoalPub.publish(fingerGoal);
  ROS_INFO("Close finger");
  ros::Duration(1.0).sleep();
  ros::spinOnce();
}

void OpenFinger(const ros::Publisher& fingerGoalPub) {
  kinova_msgs::SetFingersPositionActionGoal fingerGoal;
  fingerGoal.goal.fingers.finger1 = 0;
  fingerGoal.goal.fingers.finger2 = 0;
  fingerGoal.goal.fingers.finger3 = 0;
  fingerGoalPub.publish(fingerGoal);
  ROS_INFO("Open finger");
  ros::Duration(1.0).sleep();
  ros::spinOnce();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "door_open");
  ros::NodeHandle nh;
  std::string folderAddr = "/home/wmrm/Documents/data/door_open/" + COMMON::TimeUtil::GetDateStr() + "/";
  mkdir(folderAddr.c_str(), 0755);

  goalPoseOdom.header.frame_id = "odom";
  goalPoseArm.header.frame_id = "j2s7s300_link_base";
  wchairOriginPose.header.frame_id = "base_footprint";
  wchairOriginPose.pose.orientation.w = 1;

  armPoseFile.open(folderAddr + "arm_pose.csv", std::ios::out);
  armPoseFile << "time(ms),x(m),y(m),z(m),roll(deg),pitch(deg),yaw(deg)" << std::endl;

  wchairPoseOdom.header.frame_id = "odom";
  wchairPoseFile.open(folderAddr + "wheelchair_pose.csv", std::ios::out);
  wchairPoseFile << "time(ms),x(m),y(m),yaw(deg)" << std::endl;

  toolPoseOdom.header.frame_id = "odom";
  armOdomPoseFile.open(folderAddr + "arm_odom_pose.csv", std::ios::out);
  armOdomPoseFile << "time(ms),x(m),y(m),z(m),roll(deg),pitch(deg),yaw(deg)" << std::endl;

  jointTorqueFile.open(folderAddr + "joint_torque.csv", std::ios::out);
  jointTorqueFile << "time(ms),joint1(Nm),joint2(Nm),joint3(Nm),joint4(Nm),joint5(Nm),joint6(Nm),joint7(Nm)"
                  << std::endl;

  toolWrenchFile.open(folderAddr + "tool_wrench.csv", std::ios::out);
  toolWrenchFile << "time(ms),x(N),y(N),z(N),roll(Nm),pitch(Nm),yaw(Nm)" << std::endl;

  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> arm_client("/j2s7s300_driver/pose_action/tool_pose");
  ROS_INFO("Waiting for arm action server to start.");
  arm_client.waitForServer();
  ROS_INFO("Arm action server started, sending goal.");

  actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> finger_client(
      "/j2s7s300_driver/fingers_action/finger_positions");
  ROS_INFO("Waiting for finger action server to start.");
  finger_client.waitForServer();
  ROS_INFO("Finger action server started, sending goal.");

  ros::ServiceClient startForceClient = nh.serviceClient<kinova_msgs::Start>("/j2s7s300_driver/in/start_force_control");
  ros::ServiceClient stopForceClient = nh.serviceClient<kinova_msgs::Stop>("/j2s7s300_driver/in/stop_force_control");
  kinova_msgs::Start startSrv;
  kinova_msgs::Stop stopSrv;

  ros::Subscriber poseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 1, toolPoseCallback);
  ros::Subscriber toolWrenchSub = nh.subscribe("/j2s7s300_driver/out/tool_wrench", 1, toolWrenchCallback);
  ros::Subscriber jointTorqueSub = nh.subscribe("/j2s7s300_driver/out/joint_torques", 1, jointTorqueCallback);

  ros::Publisher fingerGoalPub = nh.advertise<kinova_msgs::SetFingersPositionActionGoal>(
      "/j2s7s300_driver/fingers_action/finger_positions/goal", 100);
  ros::Publisher wchairVelPub = nh.advertise<geometry_msgs::Twist>("/wheelchair/cmd_vel", 1);

  startTimeMs = COMMON::TimeUtil::GetTimestampMs();

  startPose.header.frame_id = "j2s7s300_link_base";
  startPose.pose.position.x = -0.48;
  startPose.pose.position.y = -0.60;
  startPose.pose.position.z = 0.46;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(START_ROLL_ARM, START_PITCH_ARM, START_YAW_ARM),
                        startPose.pose.orientation);
  kinova_msgs::ArmPoseGoal poseGoal;
  poseGoal.pose = startPose;
  arm_client.sendGoalAndWait(poseGoal);
  ROS_INFO("Move the end effector to the door handle");
  ros::spinOnce();

  CloseFinger(fingerGoalPub);

  ros::Rate loopRate(RATE);
  double rotate_theta = 0.0;
  while (ros::ok() && rotate_theta < 7 * M_PI / 24) {
    poseGoal.pose.pose.position.x = startPose.pose.position.x + DOOR_HANDLE_R * (std::cos(rotate_theta) - 1);
    poseGoal.pose.pose.position.z = startPose.pose.position.z - DOOR_HANDLE_R * std::sin(rotate_theta);
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(START_ROLL_ARM, START_PITCH_ARM - rotate_theta, START_YAW_ARM),
                          poseGoal.pose.pose.orientation);
    rotate_theta += ROTATE_VEL / RATE;
    arm_client.sendGoal(poseGoal);
    LogGoalPose(poseGoal);
  }

  arm_client.sendGoalAndWait(poseGoal);
  ROS_INFO("Finish turning the door handle");
  poseGoal.pose.pose.position.y += 0.04;
  poseGoal.pose.pose.position.x += 0.012;
  startPose.pose.position.y += 0.04;
  startPose.pose.position.x += 0.012;
  arm_client.sendGoalAndWait(poseGoal);
  LogGoalPose(poseGoal);
  ROS_INFO("pull the door handle open");

  while (ros::ok() && rotate_theta > 0) {
    poseGoal.pose.pose.position.x = startPose.pose.position.x + DOOR_HANDLE_R * (std::cos(rotate_theta) - 1);
    poseGoal.pose.pose.position.z = startPose.pose.position.z - DOOR_HANDLE_R * std::sin(rotate_theta);
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(START_ROLL_ARM, START_PITCH_ARM - rotate_theta, START_YAW_ARM),
                          poseGoal.pose.pose.orientation);
    rotate_theta -= ROTATE_VEL / RATE;
    arm_client.sendGoal(poseGoal);
    LogGoalPose(poseGoal);
  }

  arm_client.sendGoalAndWait(poseGoal);
  LogGoalPose(poseGoal);
  ROS_INFO("start pull left door");
  // startForceClient.call(startSrv);
  ROS_INFO("start force control");

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = DOOR_RADIUS * PULL_VEL_WCHAIR_LINEAR;
  cmd_vel.angular.z = PULL_VEL_WCHAIR_ANGULAR;

  tf::TransformListener arm_listener, wchair_listener;
  std::vector<double> start_pos{0.0, 0.0}, start_ori{0.0, 0.0, 0.0};
  ConfigStartPoseOdom(arm_listener, wchair_listener, start_pos, start_ori);
  double delt_yaw = tf::getYaw(wchairPoseOdom.pose.orientation);
  ROS_INFO("the start yaw odom is %.2f", delt_yaw);

  rotate_theta = 0.0;
  while (ros::ok() && rotate_theta > -M_PI / 12 * 7) {
    double delt_x = (DOOR_RADIUS - 0.045) * std::sin(rotate_theta);
    double delt_y = DOOR_RADIUS * (1 - std::cos(rotate_theta));
    goalPoseOdom.pose.position.x = start_pos[0] + delt_x * std::cos(delt_yaw) - delt_y * std::sin(delt_yaw);
    goalPoseOdom.pose.position.y = start_pos[1] + delt_x * std::sin(delt_yaw) + delt_y * std::cos(delt_yaw);
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(start_ori[0], start_ori[1], start_ori[2] + rotate_theta),
                          goalPoseOdom.pose.orientation);
    TransformPose(arm_listener, wchair_listener);
    poseGoal.pose.pose = goalPoseArm.pose;
    poseGoal.pose.pose.position.z = 0.472;
    arm_client.sendGoal(poseGoal);
    wchairVelPub.publish(cmd_vel);

    if (rotate_theta < -M_PI / 2) {
      cmd_vel.linear.x = -DOOR_RADIUS * PULL_VEL_WCHAIR_LINEAR;
      cmd_vel.linear.z = -PULL_VEL_WCHAIR_ANGULAR;
    }

    LogGoalPose(poseGoal);
    LogCurrentPose();

    rotate_theta += PULL_VEL / RATE;

    ros::spinOnce();
    loopRate.sleep();
  }

  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  wchairVelPub.publish(cmd_vel);

  arm_client.sendGoal(poseGoal);
  LogGoalPose(poseGoal);
  LogCurrentPose();
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  poseGoal.pose = toolPose;
  arm_client.sendGoal(poseGoal);
  ros::Duration(1.0).sleep();
  LogCurrentPose();

  OpenFinger(fingerGoalPub);
  stopForceClient.call(stopSrv);
  ROS_INFO("stop force control");
  ros::spinOnce();

  geometry_msgs::PoseStamped endPose = toolPose;
  TransformPose(arm_listener, wchair_listener);
  double door_x = start_pos[0] - toolPoseOdom.pose.position.x;
  double door_y = start_pos[1] - toolPoseOdom.pose.position.y;
  double yaw_door = atan2(fabs(door_x * std::cos(delt_yaw) + door_y * std::sin(delt_yaw)),
                          DOOR_RADIUS - fabs(-door_x * std::sin(delt_yaw) + door_y * std::cos(delt_yaw)));
  double end_yaw = M_PI - yaw_door;
  ROS_INFO("door_x is %.2f, door_y is %.2f, yaw_door is %.2f, end_yaw is %.2f",
           fabs(door_x * std::cos(delt_yaw) + door_y * std::sin(delt_yaw)),
           DOOR_RADIUS - fabs(-door_x * std::sin(delt_yaw) + door_y * std::cos(delt_yaw)), yaw_door * 180 / M_PI,
           end_yaw * 180 / M_PI);

  poseGoal.pose.pose.position.x = endPose.pose.position.x + 0.03 * sin(end_yaw);
  poseGoal.pose.pose.position.y = endPose.pose.position.y - 0.03 * cos(end_yaw);
  ROS_INFO("Move away from the door handle");
  LogGoalPose(poseGoal);
  arm_client.sendGoalAndWait(poseGoal);
  ros::spinOnce();

  poseGoal.pose.pose.position.x = endPose.pose.position.x + 0.3 * cos(end_yaw);
  poseGoal.pose.pose.position.y = endPose.pose.position.y + 0.3 * sin(end_yaw);
  poseGoal.pose.pose.orientation = startPose.pose.orientation;
  ROS_INFO("move the arm back");
  LogGoalPose(poseGoal);
  arm_client.sendGoalAndWait(poseGoal);
  ros::spinOnce();

  poseGoal.pose.pose.position.x = endPose.pose.position.x - 0.3 * sin(end_yaw) + 0.25 * cos(end_yaw);
  poseGoal.pose.pose.position.y = endPose.pose.position.y + 0.3 * cos(end_yaw) + 0.25 * sin(end_yaw);
  poseGoal.pose.pose.orientation = startPose.pose.orientation;
  ROS_INFO("move the arm to right");
  LogGoalPose(poseGoal);
  arm_client.sendGoalAndWait(poseGoal);
  ros::spinOnce();

  armPoseFile.close();
  armOdomPoseFile.close();
  jointTorqueFile.close();
  toolWrenchFile.close();

  return 0;
}