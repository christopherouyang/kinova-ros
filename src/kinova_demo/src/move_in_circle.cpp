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

constexpr double DOOR_RADIUS = 0.55;
constexpr double DYAW_DT = -0.1;
constexpr double DYAW_DT_WCHAIR_LINEAR = -0.044;
constexpr double DYAW_DT_WCHAIR_ANGULAR = -0.002;
constexpr double RATE = 6;

double yaw_rate{0.0}, linear_vel{0.0}, startTimeMs{0.0};
bool pose_update = false;
geometry_msgs::PoseStamped toolPoseOdom, wchairPoseOdom, wchairOriginPose;
std::ofstream armPoseFile, wchairPoseFile, armOdomPoseFile, jointTorqueFile, toolWrenchFile;

void toolPoseCallback(const geometry_msgs::PoseStamped& msg) {
  tf::TransformListener listener;
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO_THROTTLE(1.0, "Now the EEF pose(xyz-rpy) in arm coordinate is (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll * 180 / M_PI,
                    pitch * 180 / M_PI, yaw * 180 / M_PI);

  int timeNow = static_cast<int>(COMMON::TimeUtil::GetTimestampMs() - startTimeMs);
  armPoseFile << timeNow << ',' << msg.pose.position.x << ',' << msg.pose.position.y << ',' << msg.pose.position.z
              << ',' << roll * 180 / M_PI << ',' << pitch * 180 / M_PI << ',' << yaw * 180 / M_PI << ',' << std::endl;
  // try {
  //   listener.waitForTransform("odom", "j2s7s300_link_base", ros::Time::now(), ros::Duration(0.5));
  //   listener.transformPose("odom", msg, toolPoseOdom);
  // } catch (tf::TransformException& ex) {
  //   ROS_WARN("%s", ex.what());
  // }

  // tf::quaternionMsgToTF(toolPoseOdom.pose.orientation, quat);
  // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // timeNow = static_cast<int>(COMMON::TimeUtil::GetTimestampMs() - startTimeMs);
  // armOdomPoseFile << timeNow << ',' << toolPoseOdom.pose.position.x << ',' << toolPoseOdom.pose.position.y << ','
  //                 << toolPoseOdom.pose.position.z << ',' << roll * 180 / M_PI << ',' << pitch * 180 / M_PI << ','
  //                 << yaw * 180 / M_PI << ',' << std::endl;
  // try {
  //   listener.waitForTransform("odom", "base_footprint", ros::Time::now(), ros::Duration(0.5));
  //   listener.transformPose("odom", wchairOriginPose, wchairPoseOdom);
  // } catch (tf::TransformException& ex) {
  //   ROS_WARN("%s", ex.what());
  // }
  // yaw = tf::getYaw(wchairPoseOdom.pose.orientation);
  // timeNow = static_cast<int>(COMMON::TimeUtil::GetTimestampMs() - startTimeMs);
  // wchairPoseFile << timeNow << ',' << wchairPoseOdom.pose.position.x << ',' << wchairPoseOdom.pose.position.y << ','
  //                << wchairPoseOdom.pose.position.z << ',' << yaw * 180 / M_PI << ',' << std::endl;
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

void velCallback(const geometry_msgs::Twist& cmd_vel) {
  linear_vel = cmd_vel.linear.x;
  yaw_rate = cmd_vel.angular.z;
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

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "move_in_circle_node");
  ros::NodeHandle nh;
  std::string folderAddr = "/home/wmrm/Documents/data/door_open/" + COMMON::TimeUtil::GetDateStr() + "/";
  mkdir(folderAddr.c_str(), 0755);

  geometry_msgs::PoseStamped startPose, goalPoseOdom, startPoseOdom, goalPoseArm;
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
  ros::Subscriber wchairVelSub = nh.subscribe("/wheelchair/cur_vel", 1000, velCallback);

  ros::Publisher fingerGoalPub = nh.advertise<kinova_msgs::SetFingersPositionActionGoal>(
      "/j2s7s300_driver/fingers_action/finger_positions/goal", 100);
  ros::Publisher wchairVelPub = nh.advertise<geometry_msgs::Twist>("/wheelchair/cmd_vel", 1);

  startTimeMs = COMMON::TimeUtil::GetTimestampMs();

  startPose.header.frame_id = "j2s7s300_link_base";
  startPose.pose.position.x = -0.35;
  startPose.pose.position.y = -0.65;
  startPose.pose.position.z = 0.46;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-M_PI / 2, M_PI / 6, M_PI), startPose.pose.orientation);
  kinova_msgs::ArmPoseGoal poseGoal;
  poseGoal.pose = startPose;
  arm_client.sendGoalAndWait(poseGoal);

  ros::Duration(0.5).sleep();
  ros::spinOnce();

  kinova_msgs::SetFingersPositionActionGoal fingerGoal;
  fingerGoal.goal.fingers.finger1 = 5300;
  fingerGoal.goal.fingers.finger2 = 5300;
  fingerGoal.goal.fingers.finger3 = 5300;
  fingerGoalPub.publish(fingerGoal);
  ros::Duration(1).sleep();

  startForceClient.call(startSrv);
  ROS_INFO("start force control");

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = DOOR_RADIUS * DYAW_DT_WCHAIR_LINEAR;
  cmd_vel.angular.z = DYAW_DT_WCHAIR_ANGULAR;

  tf::TransformListener listener;
  listener.waitForTransform("odom", "j2s7s300_link_base", ros::Time::now(), ros::Duration(0.5));
  listener.transformPose("odom", startPose, startPoseOdom);

  double theta = 0.0;

  double start_roll, start_pitch, start_yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(startPoseOdom.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(start_roll, start_pitch, start_yaw);
  double delta_yaw = start_yaw + M_PI / 2;

  // double x_chair{0.0}, y_chair{0.0}, yaw_chair{0.0};
  // ros::Time current_time, last_time;
  ros::Rate loopRate(RATE);
  while (ros::ok() && theta > -M_PI / 36 * 17) {
    // current_time = ros::Time::now();
    // double dt = (current_time - last_time).toSec();
    // last_time = current_time;
    // x_chair += linear_vel * cos(yaw_chair) * dt;
    // y_chair += linear_vel * sin(yaw_chair) * dt;
    // yaw_chair += yaw_rate * dt;

    // ROS_INFO_THROTTLE(2.0, "the wheehchair odom is (%.2f,%.2f,%.2f)", x_chair, y_chair, yaw_chair);

    // tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-M_PI / 2, M_PI / 6, M_PI + theta - yaw_chair),
    //                       poseGoal.pose.pose.orientation);
    // poseGoal.pose.pose.position.x = startPose.pose.position.x + DOOR_RADIUS * (1 - std::cos(theta))` - y_chair;
    // poseGoal.pose.pose.position.y = startPose.pose.position.y - DOOR_RADIUS * std::sin(theta) + x_chair;

    goalPoseOdom.header.frame_id = "odom";
    goalPoseArm.header.frame_id = "j2s7s300_link_base";
    double delta_x_wchair = DOOR_RADIUS * std::sin(theta);
    double delta_y_wchair = DOOR_RADIUS * (1 - std::cos(theta));
    goalPoseOdom.pose.position.x =
        startPoseOdom.pose.position.x + delta_x_wchair * std::cos(delta_yaw) - delta_y_wchair * std::sin(delta_yaw);
    goalPoseOdom.pose.position.y =
        startPoseOdom.pose.position.y + delta_x_wchair * std::sin(delta_yaw) + delta_y_wchair * std::cos(delta_yaw);

    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(start_roll, start_pitch, start_yaw + theta),
                          goalPoseOdom.pose.orientation);

    listener.waitForTransform("odom", "j2s7s300_link_base", ros::Time::now(), ros::Duration(0.5));
    listener.transformPose("j2s7s300_link_base", goalPoseOdom, goalPoseArm);

    poseGoal.pose.header.frame_id = "j2s7s300_link_base";
    poseGoal.pose.pose.position.x = goalPoseArm.pose.position.x;
    poseGoal.pose.pose.position.y = goalPoseArm.pose.position.y;
    poseGoal.pose.pose.orientation = goalPoseArm.pose.orientation;

    arm_client.sendGoal(poseGoal);
    LogGoalPose(poseGoal);
    theta += DYAW_DT / RATE;

    wchairVelPub.publish(cmd_vel);
    ros::spinOnce();
    loopRate.sleep();
  }

  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  wchairVelPub.publish(cmd_vel);

  arm_client.sendGoalAndWait(poseGoal);

  fingerGoal.goal.fingers.finger1 = 0;
  fingerGoal.goal.fingers.finger2 = 0;
  fingerGoal.goal.fingers.finger3 = 0;
  fingerGoalPub.publish(fingerGoal);
  ros::spinOnce();
  ros::Duration(1).sleep();

  stopForceClient.call(stopSrv);
  ROS_INFO("stop force control");
  ros::spinOnce();

  armPoseFile.close();
  wchairPoseFile.close();
  armOdomPoseFile.close();
  jointTorqueFile.close();
  toolWrenchFile.close();

  return 0;
}