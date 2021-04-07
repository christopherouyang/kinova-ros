// 接收/amm/joint_states的数据，然后发布到/joint_states上去，因为move_group会接收/joint_states的数据，否则move_group内部数据可能会不更新
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class sim_joint_state_convert {
 public:
  sim_joint_state_convert() {
    ros::Duration(3.0).sleep();
    human_joint_state_flag = amm_joint_state_flag = false;
    amm_joint_state_sub = nh.subscribe("/amm/pos_controller/joint_states", 1000,
                                       &sim_joint_state_convert::AMMJointStatesSubCallback, this);
    while (!amm_joint_state_flag)
      ;
    ROS_INFO("amm joint is received!");
    bool is_human_active;
    ros::param::get("default_values/human", is_human_active);
    if (is_human_active == true) {
      ROS_INFO("Human is active");
      human_joint_state_sub = nh.subscribe("/human/pos_controller/joint_states", 1000,
                                           &sim_joint_state_convert::HumanJointStatesSubCallback, this);
      while (!human_joint_state_flag)
        ;
      ROS_INFO("human joint is received!");
    }

    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    ros::Rate loop_rate(50);
    while (ros::ok()) {
      joint_state_msgs.header = amm_joint_state_msgs.header;
      joint_state_msgs.name.clear();
      joint_state_msgs.position.clear();
      joint_state_msgs.velocity.clear();
      joint_state_msgs.effort.clear();
      for (size_t i = 0; i < amm_joint_state_msgs.name.size(); i++) {
        joint_state_msgs.name.push_back(amm_joint_state_msgs.name[i]);
        joint_state_msgs.position.push_back(amm_joint_state_msgs.position[i]);
        joint_state_msgs.velocity.push_back(amm_joint_state_msgs.velocity[i]);
        joint_state_msgs.effort.push_back(amm_joint_state_msgs.effort[i]);
      }
      for (size_t i = 0; i < human_joint_state_msgs.name.size(); i++) {
        joint_state_msgs.name.push_back(human_joint_state_msgs.name[i]);
        joint_state_msgs.position.push_back(human_joint_state_msgs.position[i]);
        joint_state_msgs.velocity.push_back(human_joint_state_msgs.velocity[i]);
        joint_state_msgs.effort.push_back(human_joint_state_msgs.effort[i]);
      }
      joint_state_pub.publish(joint_state_msgs);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

 private:
  ros::NodeHandle nh;
  bool amm_joint_state_flag, human_joint_state_flag;
  ros::Subscriber amm_joint_state_sub;
  ros::Subscriber human_joint_state_sub;
  ros::Publisher joint_state_pub;
  sensor_msgs::JointState amm_joint_state_msgs, human_joint_state_msgs, joint_state_msgs;
  void AMMJointStatesSubCallback(const sensor_msgs::JointStateConstPtr& joint_state_msgs);
  void HumanJointStatesSubCallback(const sensor_msgs::JointStateConstPtr& joint_state_msgs);
};

void sim_joint_state_convert::HumanJointStatesSubCallback(const sensor_msgs::JointStateConstPtr& joint_state_msgs) {
  human_joint_state_msgs = *joint_state_msgs;
  if (human_joint_state_flag == false)
    human_joint_state_flag = true;
}

void sim_joint_state_convert::AMMJointStatesSubCallback(const sensor_msgs::JointStateConstPtr& joint_state_msgs) {
  amm_joint_state_msgs = *joint_state_msgs;
  if (amm_joint_state_flag == false)
    amm_joint_state_flag = true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sim_joint_state_convert");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sim_joint_state_convert sjsc;
  ros::waitForShutdown();
  return 0;
}
