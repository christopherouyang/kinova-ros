#include <string>
#include <vector>
#include <memory>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <signal.h>

#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <kinova_msgs/ArmPoseActionGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/ArmJointAnglesActionGoal.h>
#include <kinova_msgs/JointAngles.h>

#define MAXLINE 8192
const std::string address = "192.168.2.172";
bool pose_update = false;
bool joint_pose_update = false;
int socketrqt;
kinova_msgs::JointAngles curAngles;
geometry_msgs::PoseStamped curPose;

void SigintHandler(int sig) {
  close(socketrqt);
  ROS_INFO_STREAM("tcp disconnected!");
  ros::shutdown();
}

void jointPosCallback(const kinova_msgs::JointAngles &msg) {
  if (!joint_pose_update) {
    curAngles = msg;
  }
  joint_pose_update = true;
}

void toolPoseCallback(const geometry_msgs::PoseStamped &msg) {
  if (!pose_update) {
    curPose = msg;
  }
  pose_update = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "jaka_state_node");
  ros::NodeHandle nh;
  signal(SIGINT, SigintHandler);
  std::string robot_ip = nh.param("robot_ip", address);

  //建立socket通讯
  struct sockaddr_in servaddr_rqt;
  ROS_INFO_STREAM("tcpListener is connecting to IP address : " << robot_ip);
  const char *address_ptr = robot_ip.c_str();
  // 创建socketrqt
  if ((socketrqt = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    ROS_ERROR_STREAM("create socket error:" << strerror(errno) << "(errno:" << errno << ")");
    exit(0);
  }
  memset(&servaddr_rqt, 0, sizeof(servaddr_rqt));
  // 指定IP地址版本为IPV4
  servaddr_rqt.sin_family = AF_INET;
  // 设置端口10001
  servaddr_rqt.sin_port = htons(10000);
  // IP地址转换函数
  if (inet_pton(AF_INET, address_ptr, &servaddr_rqt.sin_addr) <= 0) {
    ROS_ERROR_STREAM("inet_pton error for " << address_ptr);
    exit(-1);
  }

  if (connect(socketrqt, (struct sockaddr *)&servaddr_rqt, sizeof(servaddr_rqt)) < 0) {
    ROS_ERROR_STREAM("connect error:" << strerror(errno) << "(errno:" << errno << ")");
    exit(-1);
  }
  ROS_INFO_STREAM("tcp socket connects successfully!");

  kinova_msgs::ArmJointAnglesActionGoal jointGoal;
  ros::Publisher goalPub =
      nh.advertise<kinova_msgs::ArmJointAnglesActionGoal>("/j2s7s300_driver/joints_action/joint_angles/goal", 100);
  ros::Subscriber jointSub = nh.subscribe("/j2s7s300_driver/out/joint_angles", 1, jointPosCallback);

  kinova_msgs::ArmPoseActionGoal poseGoal;
  ros::Publisher poseGoalPub =
      nh.advertise<kinova_msgs::ArmPoseActionGoal>("/j2s7s300_driver/pose_action/tool_pose/goal", 100);
  ros::Subscriber poseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 1, toolPoseCallback);

  ros::Rate loopRate(5.0);
  while (ros::ok()) {
    char buf[MAXLINE];
    std::vector<int> instruct(2, 0);
    int rec_len = recv(socketrqt, buf, MAXLINE, 0);

    if (rec_len == 4) {
      instruct[0] = ((buf[0] - 48) * 10 + buf[1] - 48);
      instruct[1] = ((buf[2] - 48) * 10 + buf[3] - 48);
      ROS_INFO("the next instruction instruct is [%d ,%d]", instruct[0], instruct[1]);
    }
    static float last_inc1 = 0;
    static float last_inc2 = 0;
    // float inc1 = (instruct[0] / 34 - 1) * 20;
    // float inc2 = (instruct[1] / 34 - 1) * 45;

    float inc1 = (instruct[0] / 34 - 1) * 0.15;
    float inc2 = (instruct[1] / 34 - 1) * 0.2;

    if (pose_update && joint_pose_update) {
      if (last_inc1 != inc1 || last_inc2 != inc2) {
        // jointGoal.goal.angles = curAngles;
        // jointGoal.goal.angles.joint4 += inc1;
        // jointGoal.goal.angles.joint6 += inc2;
        // ROS_INFO_THROTTLE(1.0, "the goal of joint 5 is %f, joint6 is %f", jointGoal.goal.angles.joint5,
        //                   jointGoal.goal.angles.joint6);
        // goalPub.publish(jointGoal);

        poseGoal.goal.pose = curPose;
        poseGoal.goal.pose.pose.position.z += inc1;
        poseGoal.goal.pose.pose.position.x += inc2;
        ROS_INFO_THROTTLE(1.0, "the goal of x is %f, z is %f", poseGoal.goal.pose.pose.position.x,
                          poseGoal.goal.pose.pose.position.x);
        poseGoalPub.publish(poseGoal);
      }
      last_inc1 = inc1;
      last_inc2 = inc2;
    } else {
      ROS_WARN("wait to update new status of robot");
    }

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}