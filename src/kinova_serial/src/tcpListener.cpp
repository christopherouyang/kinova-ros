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
#include <kinova_msgs/PoseVelocityWithFingers.h>
#include <kinova_msgs/ArmJointAnglesActionGoal.h>
#include <kinova_msgs/JointAngles.h>

#define MAXLINE 8192
const std::string address = "192.168.2.172";
const float jointPos5[] = {0.0, 20, 40};
const float jointPos6[] = {0.0, 15, 30, 45};
bool pose_update = false;

int socketrqt;
kinova_msgs::JointAngles curAngles;

void SigintHandler(int sig) {
  close(socketrqt);
  ROS_INFO_STREAM("tcp disconnected!");
  ros::shutdown();
}

void posCallback(const kinova_msgs::JointAngles &msg) {
  curAngles = msg;
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

  ros::Rate loopRate(20.0);
  kinova_msgs::ArmJointAnglesActionGoal jointGoal;
  ros::Publisher goalPub =
      nh.advertise<kinova_msgs::ArmJointAnglesActionGoal>("/j2s7s300_driver/joints_action/joint_angles/goal", 100);

  ros::Subscriber posSub = nh.subscribe("/j2s7s300_driver/out/joint_angles", 1, posCallback);

  while (ros::ok()) {
    char buf[MAXLINE];
    std::vector<int> instruct = {0, 0};
    int rec_len = recv(socketrqt, buf, MAXLINE, 0);
    ROS_INFO_THROTTLE(0.5, "the length of tcp packet is %d", rec_len);

    if (rec_len == 2) {
      instruct[0] = buf[0] - 48;
      instruct[1] = buf[1] - 48;
      ROS_INFO("the next instruction instruct is [%d ,%d]", instruct[0], instruct[1]);
    }

    float inc1 = instruct[0] >= 1 && instruct[0] <= 2 ? jointPos5[instruct[0] - 1] : 0.0;
    float inc2 = 0.0;

    if (instruct[1] <= 3) {
      inc2 = jointPos6[instruct[1] - 0];
    } else if (instruct[1] <= 6) {
      inc2 = -jointPos6[instruct[1] - 3];
    }
    if (!pose_update) {
      continue;
    }
    jointGoal.goal.angles = curAngles;
    jointGoal.goal.angles.joint5 += inc1;
    jointGoal.goal.angles.joint6 += inc2;

    ROS_INFO_THROTTLE(1.0, "the goal of joint 5 is %f, joint6 is %f", jointGoal.goal.angles.joint5,
                      jointGoal.goal.angles.joint6);
    goalPub.publish(jointGoal);
    loopRate.sleep();
  }

  return 0;
}