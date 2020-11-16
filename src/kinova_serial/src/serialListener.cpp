#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <serialListener.h>
#include <kinova_msgs/PoseVelocityWithFingers.h>
#include <tf/tf.h>
#include <vector>

const std::string PORT_STR = "/dev/ttyUSB0";
constexpr uint32_t BAUD_RATE = 115200;
const float fingerPos[] = {0.0, 50, 100};
const float carteVel[] = {0.0, 0.025, 0.04, 0.08};

int main(int argc, char** argv) {
  ros::init(argc, argv, "serialListener");
  ros::NodeHandle nh;

  serial::Serial ser;
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  ser.setPort(PORT_STR);
  ser.setBaudrate(BAUD_RATE);
  ser.setTimeout(timeout);

  try {
    ser.open();
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open port " << PORT_STR);
    return -1;
  }

  if (!ser.isOpen()) {
    ROS_ERROR_STREAM("the port " << PORT_STR << " is not opened");
    return -1;
  }
  ROS_INFO_STREAM(PORT_STR << " is opened");

  ros::Rate loopRate(100.0);
  kinova_msgs::PoseVelocityWithFingers velMsg;
  ros::Publisher velPub =
      nh.advertise<kinova_msgs::PoseVelocityWithFingers>("/j2s7s300_driver/in/cartesian_velocity_with_fingers", 100);

  while (ros::ok()) {
    static std::vector<int> index = {0, 0};
    static float fingerPercent = 0.0;

    size_t n = ser.available();
    if (n != 0) {
      std_msgs::String result;
      result.data = ser.read(ser.available());
      if (result.data.length() != 2) {
        continue;
      }
      index[0] = result.data.at(0) - 48;
      index[1] = result.data.at(1) - 48;
      ROS_INFO("the next instruction index is [%d ,%d]", index[0], index[1]);
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
  return 0;
}