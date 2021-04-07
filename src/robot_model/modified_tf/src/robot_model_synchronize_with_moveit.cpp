// 将moveit规划的路径，通过数据转换，利用位置控制器，同步机器人模型
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#define JOINT_NUMBER 9
#define HUMAN_NUMBER 6

class robot_moveit_synchronization {
 public:
  robot_moveit_synchronization() {
    // 更新机器人关节
    joint_state_flag = false;
    joint_state_sub =
        nh.subscribe("/amm/joint_states", 1000, &robot_moveit_synchronization::JointStatesSubCallback, this);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    while (!joint_state_flag)
      ;

    // // 当moveit数据更新时，通过机器人位置控制接口，控制机器人到达指定位置
    // amm_moveit_joint_states_msgs.name.resize(JOINT_NUMBER);
    // amm_moveit_joint_states_msgs.position.resize(JOINT_NUMBER);
    // amm_moveit_joint_states_msgs.name = {"x_joint", "y_joint", "angle_joint", "shoulder_pan_joint",
    // "shoulder_lift_joint","elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}; amm_joint_pos_pub =
    // nh.advertise<std_msgs::Float32MultiArray>("amm/pos_controller/command",1000);

    // moveit_joint_states_sub = nh.subscribe("/move_group/fake_controller_joint_states", 1000,
    // &robot_moveit_synchronization::MoveitJointStatesSubCallback, this);

    // std_msgs::Float32MultiArray amm_joint_pos_msgs;
    // amm_joint_pos_msgs.data.resize(JOINT_NUMBER);
    // double time_out = 0.1;
    // ros::Rate loop_rate(25);
    // while (ros::ok())
    // {
    //     if((ros::Time::now() - moveit_joint_states_update_time).toSec() < time_out)
    //     {
    //         // 控制移动操作机器人
    //         for(int i = 0; i < JOINT_NUMBER; i ++)  amm_joint_pos_msgs.data[i] =
    //         amm_moveit_joint_states_msgs.position[i]; amm_joint_pos_pub.publish(amm_joint_pos_msgs);
    //     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
  }

 private:
  ros::NodeHandle nh;

  // ros::Time moveit_joint_states_update_time;
  // ros::Subscriber moveit_joint_states_sub;

  // sensor_msgs::JointState amm_moveit_joint_states_msgs;
  // ros::Publisher amm_joint_pos_pub;
  // void MoveitJointStatesSubCallback(const sensor_msgs::JointStateConstPtr& joint_state_msgs);

  bool joint_state_flag;
  ros::Subscriber joint_state_sub;
  ros::Publisher joint_state_pub;
  void JointStatesSubCallback(const sensor_msgs::JointStateConstPtr& joint_state_msgs);
};

void robot_moveit_synchronization::JointStatesSubCallback(const sensor_msgs::JointStateConstPtr& joint_state_msgs) {
  joint_state_pub.publish(*joint_state_msgs);
  if (joint_state_flag == false)
    joint_state_flag = true;
}

// void robot_moveit_synchronization::MoveitJointStatesSubCallback(const sensor_msgs::JointStateConstPtr&
// joint_state_msgs)
// {
//     moveit_joint_states_update_time = ros::Time::now();
//     for(int i = 0; i < joint_state_msgs->name.size(); i ++)
//     {
//         for(int j = 0; j < amm_moveit_joint_states_msgs.name.size(); j ++)
//         {
//             if(amm_moveit_joint_states_msgs.name[j] == joint_state_msgs->name[i])
//             {
//                 amm_moveit_joint_states_msgs.position[j] = joint_state_msgs->position[i];
//             }
//         }
//     }
// }

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "robot_model_synchronize_with_moveit");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  robot_moveit_synchronization rms;
  ros::waitForShutdown();
  return 0;
}
