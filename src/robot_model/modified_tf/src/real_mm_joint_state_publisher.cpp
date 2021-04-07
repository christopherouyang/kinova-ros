// 接受ur机器人的数据和agv的数据,并发布到/joint_states上面

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

sensor_msgs::JointState joint_state_msgs;
void AgvPosesubCallback(const nav_msgs::OdometryConstPtr& odom)
{
    joint_state_msgs.position[0] = odom->pose.pose.position.x;
    joint_state_msgs.position[1] = odom->pose.pose.position.y;
    joint_state_msgs.position[2] = odom->pose.pose.position.z;
}
void urJointsubCallback(const sensor_msgs::JointStateConstPtr& joint_states)
{
    for(int i = 0; i < 6; i ++)
    {
        joint_state_msgs.position[i+3] = joint_states->position[i];
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "real_mm_joint_state_publisher");
    ros::NodeHandle nh;
    joint_state_msgs.name.resize(9);
    joint_state_msgs.position.resize(9);
    joint_state_msgs.velocity.resize(9);
    joint_state_msgs.effort.resize(9);
    joint_state_msgs.name = {"x_joint", "y_joint", "angle_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    ros::Subscriber agv_joint_sub = nh.subscribe("odom", 1000, AgvPosesubCallback);
    ros::Subscriber ur_joint_sub = nh.subscribe("/ur_joint_states", 1000, urJointsubCallback);
    ros::Publisher agv_joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Duration(1.0).sleep();

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        joint_state_msgs.header.stamp = ros::Time::now();
        agv_joint_pub.publish(joint_state_msgs);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}