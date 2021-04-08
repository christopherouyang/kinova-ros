#include "mm_map_creator/mm_sampling.h"
#include "stdlib.h"

namespace mm_sampling {
mm_sampling::mm_sampling() {
  load_joint_names(joint_names);
  load_joint_limits(joint_limits);
  srand((int)time(0));
}

bool mm_sampling::rm_down_sample(pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud,
                                 pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud_down_sample, int sample_number) {
  std::multiset<std::pair<double, std::vector<double> > > rm_set;
  double manipulability;
  std::vector<double> pose_vector(6);
  for (int i = 0; i < rm_cloud->width; i++) {
    manipulability = rm_cloud->points[i].curvature;
    pose_vector[0] = rm_cloud->points[i].x;
    pose_vector[1] = rm_cloud->points[i].y;
    pose_vector[2] = rm_cloud->points[i].z;
    pose_vector[3] = rm_cloud->points[i].normal_x;
    pose_vector[4] = rm_cloud->points[i].normal_y;
    pose_vector[5] = rm_cloud->points[i].normal_z;
    rm_set.insert(std::make_pair(manipulability, pose_vector));
  }

  // 对点云进行排序
  int count = 0;
  for (auto begin = --rm_set.begin(), end = --rm_set.end(); end != begin; end--) {
    if (count++ == sample_number) {
      ROS_INFO_STREAM("Number is " << count);
      break;
    }
    pcl::PointNormal pointnormal;
    pointnormal.x = end->second[0];
    pointnormal.y = end->second[1];
    pointnormal.z = end->second[2];
    pointnormal.normal_x = end->second[3];
    pointnormal.normal_y = end->second[4];
    pointnormal.normal_z = end->second[5];
    pointnormal.curvature = end->first;
    rm_cloud_down_sample->push_back(pointnormal);
  }
}

/****************************************cartesian space sampling****************************************/
bool mm_sampling::makeCMbyCartesianSpaceSampling(pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud,
                                                 pcl::PointCloud<pcl::PointNormal>::Ptr des_rm_cloud,
                                                 double cm_translation_resolution, double cm_orientation_resolution) {
  cout << "Creating reachability map, please wait!" << endl;
  cm_translation_resolution = cm_translation_resolution / 100;
  cm_orientation_resolution = cm_orientation_resolution / 180 * PI;
  // 分辨率一方面会显著影响CM中点的数量，同时也会影响orm_patch的计算时间

  std::vector<double> agv_joint_values(3, 0);
  std::vector<double> joint_values(6, 0);
  std::vector<double> amm_joint_values(9, 0);
  std::vector<double> ur_ref_joints = {0, -0.6, 0.9, -0.3, 1.5708, 0};

  for (size_t i = 0; i < ur_ref_joints.size(); i++)
    amm_joint_values[i + 3] = ur_ref_joints[i];
  std::vector<double> ref_pose_vector;
  isPoseinROI(kinematics.GetTotalHomoMatrix(amm_joint_values), ref_pose_vector);

  double min_x = 0.5, max_x = ref_pose_vector[0] + 0.05;
  double min_y = -0.5, max_y = 0.5;
  double initial_height = ref_pose_vector[2], height_range = 0.5;
  double initial_roll = ref_pose_vector[3], roll_range = 15 * PI / 180;
  double initial_pitch = ref_pose_vector[4], pitch_range = 15 * PI / 180;
  double initial_yaw = ref_pose_vector[5], yaw_range = 15 * PI / 180;
  double agv_base_ray_range = 22.5 * PI / 180;  // 以基座为中心的扇形范围
  // 上面的几个range会影响CM中的点的数量，但是对orm_patch的计算时间影响很小

  pcl::PointNormal pointnormal;
  pcl::PointNormal des_point;
  des_point.curvature = -1;

  int count = 0;
  for (double x = min_x; x <= max_x; x += cm_translation_resolution) {
    for (double y = min_y; y <= max_y; y += cm_translation_resolution) {
      if (fabs(atan2(y, x)) > agv_base_ray_range)
        continue;
      for (double z = initial_height - height_range; z <= initial_height + height_range;
           z += cm_translation_resolution) {
        des_point.x = x;
        des_point.y = y;
        des_point.z = z;
        if (des_point.curvature != count) {
          des_point.curvature = count;
          des_rm_cloud->push_back(des_point);
        }
        for (double roll = initial_roll - roll_range; roll <= initial_roll + roll_range;
             roll += cm_orientation_resolution) {
          for (double pitch = initial_pitch - pitch_range; pitch <= initial_pitch + pitch_range;
               pitch += cm_orientation_resolution) {
            for (double yaw = initial_yaw - yaw_range; yaw <= initial_yaw + yaw_range;
                 yaw += cm_orientation_resolution) {
              std::vector<double> target_pose = {x, y, z, roll, pitch, yaw};
              if (isProperIKExist(target_pose, ur_ref_joints, agv_joint_values, joint_values) &&
                  !collision_check.isCollision(joint_values)) {
                for (int i = 0; i < 6; i++)
                  amm_joint_values[i + 3] = joint_values[i];
                double manipulability = kinematics.GetManipulability(amm_joint_values);
                if (manipulability > 60)  // 当可操作度大于一定数值时，才保存
                {
                  pointnormal.x = x;
                  pointnormal.y = y;
                  pointnormal.z = z;
                  pointnormal.normal_x = roll;
                  pointnormal.normal_y = pitch;
                  pointnormal.normal_z = yaw;
                  pointnormal.curvature = manipulability;
                  rm_cloud->push_back(pointnormal);
                  count++;
                }
              }
            }
          }
        }
      }
    }
  }
  des_point.x = 0;
  des_point.y = 0;
  des_point.z = 0;
  des_point.curvature = count;
  des_rm_cloud->push_back(des_point);
  ROS_INFO_STREAM("Index: " << des_point.curvature);
  // 其中des_rm_cloud表示笛卡尔空间xyz组成的点云，用来做近邻搜索用，第四个值代表的是该点对应的PointNormal的起始下标，其中最后一个点只是用来存储rm_cloud的最大索引
  // rm_cloud代表真正的能力图，其下标索引与des_rm_cloud的第四个数值有对应关系
  return true;
}

bool mm_sampling::isProperIKExist(std::vector<double> target_pose_vector, std::vector<double> ur_ref_joints,
                                  std::vector<double> agv_joint_values, std::vector<double>& joint_values) {
  joint_values.clear();
  geometry_msgs::Pose target_pose;
  target_pose.position.x = target_pose_vector[0];
  target_pose.position.y = target_pose_vector[1];
  target_pose.position.z = target_pose_vector[2];
  target_pose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(target_pose_vector[3], target_pose_vector[4], target_pose_vector[5]);
  if (mik.getIkFromPose(target_pose, agv_joint_values, joint_values) == false)
    return false;

  std::vector<double> max_error(joint_values.size() / 6);
  for (int i = 0; i < max_error.size(); i++) {
    std::vector<double> joint_error(6);
    for (int j = 0; j < 6; j++) {
      joint_error[j] = fabs(joint_values[6 * i + j] - ur_ref_joints[j]);
    }
    max_error[i] = *std::max_element(std::begin(joint_error), std::end(joint_error));
  }
  // 所有解中的最大关节位移的最小值
  double min_error = *std::min_element(std::begin(max_error), std::end(max_error));
  if (min_error > 1.0)
    return false;  // 这个值可能需要修改
  else {
    //  第k组解是最好的
    int k = std::distance(std::begin(max_error), std::min_element(std::begin(max_error), std::end(max_error)));
    if (joint_values[6 * k + 2] < 0.7 || joint_values[6 * k + 2] > 2.5)
      return false;  // 0.7和2.5可能需要修改
    for (int i = 0; i < 6; i++) {
      joint_values[i] = joint_values[6 * k + i];
    }
    joint_values.resize(6);  // 只保留前6个数值
    return true;
  }
}

bool mm_sampling::isPoseinROI(Eigen::Affine3d end_pose_matrix, std::vector<double>& end_pose_vector) {
  end_pose_vector.resize(6);

  double agv_base_ray_range = 90 * PI / 180;
  double agv_rpy_range = 45 * PI / 180;
  double agv_ref_height = 0;
  double height_range = 0.5;

  if (fabs(atan2(end_pose_matrix(1, 3), end_pose_matrix(0, 3))) > agv_base_ray_range)
    return false;
  if (end_pose_matrix(2, 3) > agv_ref_height + height_range || end_pose_matrix(2, 3) < agv_ref_height - height_range)
    return false;
  end_pose_vector[0] = end_pose_matrix(0, 3);
  end_pose_vector[1] = end_pose_matrix(1, 3);
  end_pose_vector[2] = end_pose_matrix(2, 3);

  std::vector<double> rpy_angle(6);
  GetAllRPYFromPoseMatrix(end_pose_matrix, rpy_angle);
  // 判断标准需要根据任务来确定
  if (fabs(rpy_angle[0] - PI / 2) < agv_rpy_range && fabs(rpy_angle[1]) < agv_rpy_range) {
    end_pose_vector[3] = rpy_angle[0];
    end_pose_vector[4] = rpy_angle[1];
    end_pose_vector[5] = rpy_angle[2];
    return true;
  }
  if (fabs(rpy_angle[3] - PI / 2) < agv_rpy_range && fabs(rpy_angle[4]) < agv_rpy_range) {
    end_pose_vector[3] = rpy_angle[3];
    end_pose_vector[4] = rpy_angle[4];
    end_pose_vector[5] = rpy_angle[5];
    return true;
  }
  return false;
}

void mm_sampling::GetAllRPYFromPoseMatrix(Eigen::Affine3d pose_matrix, std::vector<double>& rpy_angle) {
  // atan2(y,x)
  double pitch1 = atan2(-pose_matrix(2, 0), -sqrt(1 - pose_matrix(2, 0) * pose_matrix(2, 0)));
  double roll1 = atan2(pose_matrix(2, 1) / cos(pitch1), pose_matrix(2, 2) / cos(pitch1));
  double yaw1 = atan2(pose_matrix(1, 0) / cos(pitch1), pose_matrix(0, 0) / cos(pitch1));

  double pitch2 = atan2(-pose_matrix(2, 0), sqrt(1 - pose_matrix(2, 0) * pose_matrix(2, 0)));
  double roll2 = atan2(pose_matrix(2, 1) / cos(pitch2), pose_matrix(2, 2) / cos(pitch2));
  double yaw2 = atan2(pose_matrix(1, 0) / cos(pitch2), pose_matrix(0, 0) / cos(pitch2));

  rpy_angle[0] = roll1;
  rpy_angle[1] = pitch1;
  rpy_angle[2] = yaw1;

  rpy_angle[3] = roll2;
  rpy_angle[4] = pitch2;
  rpy_angle[5] = yaw2;
}

/****************************************joint space sampling****************************************/
// 目前关节生成的点云仅仅用来做可视化
bool mm_sampling::makeCMbyJointSapceSampling(pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud, int max_sample_point) {
  std::vector<double> joint_values(6, 0);
  std::vector<double> end_pose_vector(6, 0);  // 存储末端位姿
  Eigen::Affine3d end_pose_matrix;            // 以矩阵的形式存储末端位姿
  std::vector<double> manip_vector(max_sample_point);  // 存储机器人在不同位型下的可操作度，方便后续进行归一化

  int count = 0, percent_before = 0;
  cout << "Creating reachability map, please wait!" << endl;
  for (int i = 0; i < 100; i++)
    cout << "=";

  for (unsigned int i = 0; i < max_sample_point;) {
    joint_space_sampling(joint_values);  // 更新ur和amm关节值
    if (collision_check.isCollision(joint_values))
      continue;                                                     // 舍弃碰撞的点
    end_pose_matrix = kinematics.GetTotalHomoMatrix(joint_values);  // eef相对于agv_base_link的齐次变换矩阵

    convertPoseToPoseVector(end_pose_matrix, end_pose_vector);

    manip_vector[i] = kinematics.GetManipulability(joint_values);
    int percent = double(i) / max_sample_point * 100;
    if (percent > percent_before) {
      percent_before += 1;
      cout << "\033[32m"
           << "\r"
           << "[" << percent << "\%"
           << "]"
           << "\033[0m";
      for (int j = 0; j <= percent; j++)
        cout << "\033[32m"
             << ">"
             << "\033[0m";
    }
    pcl::PointNormal pointnormal;
    pointnormal.x = end_pose_vector[0];
    pointnormal.y = end_pose_vector[1];
    pointnormal.z = end_pose_vector[2];
    pointnormal.normal_x = end_pose_vector[3];
    pointnormal.normal_y = end_pose_vector[4];
    pointnormal.normal_z = end_pose_vector[5];
    pointnormal.curvature = manip_vector[i];
    rm_cloud->push_back(pointnormal);
    i++;
  }
  cout << endl;

  std::cout << "sampling completed" << std::endl;
  double max_manip = (*std::max_element(manip_vector.begin(), manip_vector.end()));
  ROS_INFO_STREAM("Max manipulability is: " << (max_manip));
  ROS_INFO("Reachability map completed!!!");
  return true;
}

void mm_sampling::convertPoseToPoseVector(const Eigen::Affine3d pose, std::vector<double>& end_pose_vector) {
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << pose(0, 0), pose(0, 1), pose(0, 2), pose(1, 0), pose(1, 1), pose(1, 2), pose(2, 0), pose(2, 1),
      pose(2, 2);
  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
  end_pose_vector[0] = pose(0, 3);
  end_pose_vector[1] = pose(1, 3);
  end_pose_vector[2] = pose(2, 3);
  end_pose_vector[3] = euler_angles(2);  // roll
  end_pose_vector[4] = euler_angles(1);  // pitch
  end_pose_vector[5] = euler_angles(0);  // yaw
                                         // 注意rpy的顺序
}

int mm_sampling::load_joint_names(std::vector<std::string>& joint_names) {
  joint_names.resize(ARM_JOINT_NUM);
  ros::param::get("kinova_joint_names", joint_names);
  if (joint_names[0] == "") {
    ROS_ERROR("There is no joint names in parameter server!!!");
    exit(0);
  }
  return 0;
}

int mm_sampling::load_joint_limits(std::vector<std::pair<double, double> >& joint_limits) {
  joint_limits.resize(ARM_JOINT_NUM);
  for (int i = 0; i < ARM_JOINT_NUM; i++) {
    ros::param::get("ur5_joint_limits/" + joint_names[i] + "/lower_limits", joint_limits[i].first);
    ros::param::get("ur5_joint_limits/" + joint_names[i] + "/upper_limits", joint_limits[i].second);
  }
  if (!joint_limits[0].first) {
    ROS_ERROR("There is no joint names in parameter server!!!");
    exit(0);
  }
  return 0;
}

void mm_sampling::joint_space_sampling(std::vector<double>& joint_values) {
  double times = 100000;
  for (int i = 0; i < ARM_JOINT_NUM; i++) {
    joint_values[i] = mm_rand(joint_limits[i].first, joint_limits[i].second, times);
  }
}

double mm_sampling::mm_rand(double dmin, double dmax, int times) {
  int max = dmax * times;
  int min = dmin * times;
  int i_value = rand() % (max - min + 1) + min;
  return double(i_value) / times;
}
}  // namespace mm_sampling
