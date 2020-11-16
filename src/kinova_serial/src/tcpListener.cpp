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

#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <kinova_msgs/PoseVelocityWithFingers.h>

#define MAXLINE 8192
const std::string address = "192.168.2.10";
const float fingerPos[] = {0.0, 50, 100};
const float carteVel[] = {0.0, 0.025, 0.04, 0.08};

int main(int argc, char **argv) {
  ros::init(argc, argv, "jaka_state_node");
  ros::NodeHandle nh;
  std::string robot_ip = nh.param("robot_ip", address);

  //建立socket通讯
  int socketrqt;
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
  ROS_INFO_STREAM("state_pub_node socket connects successfully!");

  ros::Rate loopRate(100.0);
  kinova_msgs::PoseVelocityWithFingers velMsg;
  ros::Publisher velPub =
      nh.advertise<kinova_msgs::PoseVelocityWithFingers>("/j2s7s300_driver/in/cartesian_velocity_with_fingers", 100);

  Json::Reader reader;
  Json::Value root;
  while (ros::ok()) {
    char buf[MAXLINE];
    int rec_len = recv(socketrqt, buf, MAXLINE, 0);
    buf[rec_len] = '\0';

    static std::vector<int> index = {0, 0};
    static float fingerPercent = 0.0;
    try {
      if (reader.parse(buf, root)) {
        index[0] = root["arm_velocity"].asInt();
        index[1] = root["finger_position"].asInt();
        ROS_INFO("the next instruction index is [%d ,%d]", index[0], index[1]);
      }
    } catch (...) {
    }

    fingerPercent = index[0] >= 1 && index[0] <= 3 ? fingerPos[index[0] - 1] : fingerPercent;

    if (index[1] <= 3) {
      velMsg.twist_linear_x = carteVel[index[1] - 0];
    } else if (index[1] <= 6) {
      velMsg.twist_linear_x = -carteVel[index[1] - 3];
    } else {
      velMsg.twist_linear_x = 0.0;
    }
    velMsg.fingers_closure_percentage = fingerPercent;

    ROS_INFO_THROTTLE(1.0, "the velocity of x is %f, the percentage of finger is %f", velMsg.twist_linear_x,
                      fingerPercent);
    velPub.publish(velMsg);
    loopRate.sleep();
  }

  //关闭socket连接
  close(socketrqt);
  ROS_INFO_STREAM("Robot disconnected!");
  return 0;
}