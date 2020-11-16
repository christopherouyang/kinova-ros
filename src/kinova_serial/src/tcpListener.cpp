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

#define MAXLINE 8192
const std::string address = "192.168.2.172";
const float fingerPos[] = {0.0, 100};
const float carteVel[] = {0.0, 0.025, 0.04, 0.08};
char buf[MAXLINE];
int rec_len = 0;
bool tcp_open = false;
std::vector<int> instruct = {0, 0};
int socketrqt;

void SigintHandler(int sig) {
  close(socketrqt);
  tcp_open = false;
  ROS_INFO_STREAM("tcp disconnected!");
  ros::shutdown();
}

void tcpRcvCallback() {
  while (ros::ok() && tcp_open) {
    rec_len = recv(socketrqt, buf, MAXLINE, 0);
    ROS_INFO_THROTTLE(0.5, "the length of tcp packet is %d", rec_len);

    if (rec_len == 2) {
      instruct[0] = buf[0] - 48;
      instruct[1] = buf[1] - 48;
      ROS_INFO("the next instruction instruct is [%d ,%d]", instruct[0], instruct[1]);
    }
  }
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
  tcp_open = true;

  ros::Rate loopRate(100.0);
  kinova_msgs::PoseVelocityWithFingers velMsg;
  ros::Publisher velPub =
      nh.advertise<kinova_msgs::PoseVelocityWithFingers>("/j2s7s300_driver/in/cartesian_velocity_with_fingers", 100);

  std::thread thread1(tcpRcvCallback);

  while (ros::ok()) {
    static float fingerPercent = 0.0;

    fingerPercent = instruct[0] >= 1 && instruct[0] <= 2 ? fingerPos[instruct[0] - 1] : fingerPercent;

    if (instruct[1] <= 3) {
      velMsg.twist_linear_x = carteVel[instruct[1] - 0];
    } else if (instruct[1] <= 6) {
      velMsg.twist_linear_x = -carteVel[instruct[1] - 3];
    } else {
      velMsg.twist_linear_x = 0.0;
    }
    velMsg.fingers_closure_percentage = fingerPercent;

    ROS_INFO_THROTTLE(1.0, "the velocity of x is %f, the percentage of finger is %f", velMsg.twist_linear_x,
                      fingerPercent);
    velPub.publish(velMsg);
    loopRate.sleep();
  }

  thread1.join();
  return 0;
}