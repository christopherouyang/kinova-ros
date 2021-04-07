#ifndef DATA_TYPE_H
#define DATA_TYPE_H

// head file for bacis C++ function
#include <iostream>
#include "fstream"
#include "iomanip"
#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <map>
#include <set>

// head file for basic ros function
#include <ros/ros.h>
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/WrenchStamped.h"
#include <tf2_ros/transform_broadcaster.h>

// head file for Eigen
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

// head file for pcl
#include <pcl/io/pcd_io.h>
#include <pcl_ros/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <octomap/octomap.h>

// head file for moveit and arrow display
#include <moveit/move_group_interface/move_group_interface.h>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <moveit/planning_scene/planning_scene.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "moveit/robot_state/conversions.h"
#include "nav_msgs/Path.h"

// service file
#include "mm_map_creator/orm_calculation.h"

// parallel calculation
#include "omp.h"

using std::cout;
using std::endl;
using std::setw;
using std::string;

constexpr int JOINT_NUMBER = 7;
constexpr int UR_JOINT_NUMBER = 7;
constexpr int AGV_JOINT_NUMBER = 0;
constexpr double PI = 3.1415926536;

#endif