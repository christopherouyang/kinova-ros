#include "mm_map_creator/mm_base_placement.h"

namespace mm_base_placement {
mm_base_placement::mm_base_placement()
    : rm_cloud(new pcl::PointCloud<pcl::PointNormal>), des_rm_cloud(new pcl::PointCloud<pcl::PointNormal>) {
  ros::param::get("default_values/agv_movement_delta", agv_movement_delta);
  ros::param::get("default_values/cm_translation_resolution", filter_translation_resolution);
  ros::param::get("default_values/cm_orientation_resolution", filter_orientation_resolution);
  filter_translation_resolution = filter_translation_resolution / 100;
  filter_orientation_resolution = filter_orientation_resolution * PI / 180;
}

bool mm_base_placement::load_rm(int point_number1, int point_number2) {
  // load des_rm_cloud
  std::string path(ros::package::getPath("mm_map_creator") + "/maps/");
  std::string pcd_file = "mm_n" + std::to_string(point_number2) + "_des.pcd";
  std::string pcd_file_name = path + pcd_file;
  if (pcl::io::loadPCDFile<pcl::PointNormal>(pcd_file_name, *des_rm_cloud) == -1)
    ROS_ERROR("Can't load Descarte Reachability Map PCD file");
  ROS_INFO_STREAM("Loaded " << pcd_file_name);

  // input des_rm_cloud to kdtree
  pcl::PointCloud<pcl::PointXYZ>::Ptr des_rm_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < des_rm_cloud->width; i++) {
    pcl::PointXYZ point;
    point.x = des_rm_cloud->points[i].x;
    point.y = des_rm_cloud->points[i].y;
    point.z = des_rm_cloud->points[i].z;
    des_rm_cloud_xyz->push_back(point);
  }
  kdtree.setInputCloud(des_rm_cloud_xyz);
  ROS_INFO("des_rm_cloud has been inserted to kdtree!");

  // load rm_cloud
  pcd_file = "mm_n" + std::to_string(point_number1) + "_reachability.pcd";
  pcd_file_name = path + pcd_file;
  ROS_INFO_STREAM("Loading " << pcd_file_name);
  if (pcl::io::loadPCDFile<pcl::PointNormal>(pcd_file_name, *rm_cloud) == -1)
    ROS_ERROR("Can't load Reachability Map PCD file");
  ROS_INFO_STREAM("Loaded " << pcd_file_name);
  return true;
}

/******************************机器人基座放置相关函数******************************/
bool mm_base_placement::get_orm(std::vector<double> eef_pose_vector,
                                pcl::PointCloud<pcl::PointNormal>::Ptr base_pose_cloud, int method) {
  ros::Time start_time = ros::Time::now();
  Eigen::Affine3d target_pose_matrix = mmp.getPoseMatrixFromPoseVector(eef_pose_vector);
  // filter_translation_resolution = 0.1;
  // filter_orientation_resolution = 0.1;
  if (method == 1) {
    pcl::PointCloud<pcl::PointNormal>::Ptr z_cloud(new pcl::PointCloud<pcl::PointNormal>);
    z_filter(eef_pose_vector[2], rm_cloud, z_cloud);
    if (1) {
      // 这个算法的核心思想是先得到所有的rpy角度，并对rpy进行两次滤波，同时在对rpy滤波的时候，考虑了边界的影响，可以保证从CM中得到完整的ORM
      std::vector<double> rpy_angle(6, 0);
      mmp.GetAllRPYFromPoseMatrix(target_pose_matrix, rpy_angle);
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_1(new pcl::PointCloud<pcl::PointNormal>);
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_2(new pcl::PointCloud<pcl::PointNormal>);
      roll_filter2(rpy_angle[0], z_cloud, cloud_1);
      pitch_filter2(rpy_angle[1], cloud_1, cloud_1);
      roll_filter2(rpy_angle[3], z_cloud, cloud_2);
      pitch_filter2(rpy_angle[4], cloud_2, cloud_2);
      *base_pose_cloud = *cloud_1 + *cloud_2;
      // 2. get base_pose_cloud
      for (int i = 0; i < base_pose_cloud->width; i++) {
        getAGVPosePointFromBasePosePoint(target_pose_matrix, base_pose_cloud->points[i]);
      }
      // ROS_INFO_STREAM("Cloud size: "<<base_pose_cloud->points.size());
      // ROS_INFO_STREAM("Computation time is: "<<(ros::Time::now() - start_time).toSec() * 1000<<" ms");
    } else {
      // 滤波方式有所差别，不过在正常工作区，以及CM按照笛卡尔空间采样的话，结果和上面的方法相同。一般情况下，得到的点云会比上面的方法少
      mmp.chooseProperPoseVector(target_pose_matrix, eef_pose_vector);
      pitch_filter(eef_pose_vector[4], z_cloud, base_pose_cloud);
      roll_filter(eef_pose_vector[3], base_pose_cloud, base_pose_cloud);
      // 2. get base_pose_cloud
      // #pragma omp parallel for
      for (int i = 0; i < base_pose_cloud->width; i++) {
        getAGVPosePointFromBasePosePoint(target_pose_matrix, base_pose_cloud->points[i]);
      }
      // ROS_INFO_STREAM("Cloud size: "<<base_pose_cloud->points.size());
      // ROS_INFO_STREAM("Computation time is: "<<(ros::Time::now() - start_time).toSec() * 1000<<" ms");
    }
  }
  if (method == 2) {
    // 传统的方法，把CM中的点先转换到地面坐标下，然后再进行滤波
    *base_pose_cloud = *rm_cloud;
    for (size_t i = 0; i < base_pose_cloud->points.size(); i++) {
      getAGVPosePointFromBasePosePoint(target_pose_matrix, base_pose_cloud->points[i]);
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr z_cloud(new pcl::PointCloud<pcl::PointNormal>);
    z_filter(0, base_pose_cloud, z_cloud);
    std::vector<double> rpy_angle(6, 0);
    mmp.GetAllRPYFromPoseMatrix(target_pose_matrix, rpy_angle);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_1(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_2(new pcl::PointCloud<pcl::PointNormal>);
    roll_filter2(0, z_cloud, cloud_1);
    pitch_filter2(0, cloud_1, cloud_1);
    roll_filter2(0, z_cloud, cloud_2);
    pitch_filter2(0, cloud_2, cloud_2);
    base_pose_cloud->clear();
    *base_pose_cloud = *cloud_1 + *cloud_2;
    // ROS_INFO_STREAM("Cloud size: "<<base_pose_cloud->points.size());
    // ROS_INFO_STREAM("Computation time is: "<<(ros::Time::now() - start_time).toSec() * 1000<<" ms");
  }
}

bool mm_base_placement::get_orm_patch(std::vector<double> eef_pose_vector, std::vector<double> current_agv_pose,
                                      pcl::PointCloud<pcl::PointNormal>::Ptr base_pose_cloud) {
  ros::Time start_time = ros::Time::now();
  // 0. 近邻搜索的时候，需要将eef_pose_vector转换到移动机器人坐标系下面
  std::vector<double> agv_pose_vector = {current_agv_pose[0], current_agv_pose[1], 0, 0, 0, current_agv_pose[2]};
  Eigen::Affine3d agv_pose_matrix = mmp.getPoseMatrixFromPoseVector(agv_pose_vector);
  Eigen::Affine3d eef_to_agv_matrix = agv_pose_matrix.inverse() * mmp.getPoseMatrixFromPoseVector(eef_pose_vector);

  // 1.
  // kdtree搜索末端最近的六个点（默认上下前后左右），同时确保这些点确实在近邻内（加一次距离判断），其实相当于在xyz方向做了一次滤波
  pcl::PointXYZ searchPoint;
  searchPoint.x = eef_to_agv_matrix(0, 3);
  searchPoint.y = eef_to_agv_matrix(1, 3);
  searchPoint.z = eef_to_agv_matrix(2, 3);
  std::vector<int> indices, point_id;
  std::vector<float> point_distance;
  kdtree.nearestKSearch(searchPoint, 6, indices, point_distance);
  for (size_t i = 0; i < indices.size(); i++) {
    double distance_to_search_point = sqrt(pow(des_rm_cloud->points[indices[i]].x - searchPoint.x, 2) +
                                           pow(des_rm_cloud->points[indices[i]].y - searchPoint.y, 2) +
                                           pow(des_rm_cloud->points[indices[i]].z - searchPoint.z, 2));
    if (distance_to_search_point < 2 * filter_translation_resolution)
      point_id.push_back(indices[i]);
  }
  // 2. 根据得到的球以及z方向的信息得到面片，并将面片里面的点装入rm_cloud_patch，方便进行进一步的滤波
  int count = 0;
  for (size_t i = 0; i < point_id.size(); i++) {
    count += (des_rm_cloud->points[point_id[i] + 1].curvature - des_rm_cloud->points[point_id[i]].curvature);
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud_patch(new pcl::PointCloud<pcl::PointNormal>);
  rm_cloud_patch->points.resize(count);  // 先resize，再按照下标往里面赋值会更快一些
  count = 0;
  for (size_t i = 0; i < point_id.size(); i++) {
    for (int j = des_rm_cloud->points[point_id[i]].curvature; j < des_rm_cloud->points[point_id[i] + 1].curvature;
         j++) {
      rm_cloud_patch->points[count++] = rm_cloud->points[j];
    }
  }

  // 3. 滤波
  Eigen::Affine3d target_pose_matrix = mmp.getPoseMatrixFromPoseVector(eef_pose_vector);
  mmp.chooseProperPoseVector(target_pose_matrix, eef_pose_vector);
  roll_filter(eef_pose_vector[3], rm_cloud_patch, rm_cloud_patch);
  pitch_filter(eef_pose_vector[4], rm_cloud_patch, rm_cloud_patch);
  // 4. calculating base_pose_cloud from rm_cloud_patch
  // 加并行运算
  omp_set_num_threads(4);
#pragma omp parallel for
  for (int i = 0; i < rm_cloud_patch->width; i++) {
    getAGVPosePointFromBasePosePoint(target_pose_matrix, rm_cloud_patch->points[i]);
  }
  base_pose_cloud->clear();
  for (int i = 0; i < rm_cloud_patch->width; i++) {
    if (isPoseNearCurrentAgvPose(rm_cloud_patch->points[i], current_agv_pose, agv_movement_delta)) {
      base_pose_cloud->push_back(rm_cloud_patch->points[i]);
    }
  }
  // ROS_INFO_STREAM("Point number near current mobile robot is: "<<base_pose_cloud->points.size());
  // ROS_INFO_STREAM("agv pose cal time is: "<<(ros::Time::now() - start_time).toSec() * 1000<<" ms");
  return true;
}

bool mm_base_placement::isPoseNearCurrentAgvPose(pcl::PointNormal point, std::vector<double> current_agv_pose,
                                                 double threshold) {
  bool status_x = (fabs(point.x - current_agv_pose[0]) < threshold);
  bool status_y = (fabs(point.y - current_agv_pose[1]) < threshold);
  bool status_theta = (fabs(point.normal_z - current_agv_pose[2]) < threshold);
  if (status_x && status_y && status_theta)
    return true;
  else
    return false;
}

void mm_base_placement::getAGVPosePointFromBasePosePoint(const Eigen::Affine3d target_pose_matrix,
                                                         pcl::PointNormal& point) {
  Eigen::Affine3d agv_pose_matrix = target_pose_matrix * getPoseMatrixFromPointNormal(point).inverse();
  std::vector<double> agv_pose_vector = chooseProperAGVPoseVector(agv_pose_matrix);
  point.x = agv_pose_vector[0];
  point.y = agv_pose_vector[1];
  point.z = agv_pose_vector[2];
  point.normal_x = agv_pose_vector[3];
  point.normal_y = agv_pose_vector[4];
  point.normal_z = agv_pose_vector[5];
}

std::vector<double> mm_base_placement::chooseProperAGVPoseVector(Eigen::Affine3d agv_pose_matrix) {
  // 把移动机器人的z，roll和pitch都设置成为0
  std::vector<double> agv_pose_vector(6, 0);
  agv_pose_vector[0] = agv_pose_matrix(0, 3);
  agv_pose_vector[1] = agv_pose_matrix(1, 3);
  agv_pose_vector[2] = 0;
  agv_pose_vector[3] = 0;
  agv_pose_vector[4] = 0;
  std::vector<double> rpy_angle(6);
  mmp.GetAllRPYFromPoseMatrix(agv_pose_matrix, rpy_angle);
  if (fabs(rpy_angle[0]) < fabs(rpy_angle[3]) && fabs(rpy_angle[1]) < fabs(rpy_angle[4]))
    agv_pose_vector[5] = rpy_angle[2];
  else
    agv_pose_vector[5] = rpy_angle[5];
  return agv_pose_vector;
}

Eigen::Affine3d mm_base_placement::getPoseMatrixFromPointNormal(pcl::PointNormal point) {
  geometry_msgs::Pose pose;
  Eigen::Affine3d pose_matrix;
  pose.position.x = point.x;
  pose.position.y = point.y;
  pose.position.z = point.z;
  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(point.normal_x, point.normal_y, point.normal_z);
  tf2::convert(pose, pose_matrix);
  return pose_matrix;
}

/*****************************************滤波函数*******************************************/
void mm_base_placement::roll_filter(double angle, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                                    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud) {
  pass.setInputCloud(source_cloud);
  pass.setFilterFieldName("normal_x");
  pass.setFilterLimits(angle - filter_orientation_resolution, angle + filter_orientation_resolution);
  pass.filter(*target_cloud);
}

void mm_base_placement::pitch_filter(double angle, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                                     pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud) {
  pass.setInputCloud(source_cloud);
  pass.setFilterFieldName("normal_y");
  pass.setFilterLimits(angle - filter_orientation_resolution, angle + filter_orientation_resolution);
  pass.filter(*target_cloud);
}

void mm_base_placement::z_filter(double z, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                                 pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud) {
  pass.setInputCloud(source_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z - filter_translation_resolution, z + filter_translation_resolution);
  pass.filter(*target_cloud);
}

void mm_base_placement::roll_filter2(double angle, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                                     pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud) {
  if (fabs(angle) < (PI - filter_orientation_resolution)) {
    pass.setInputCloud(source_cloud);
    pass.setFilterFieldName("normal_x");
    pass.setFilterLimits(angle - filter_orientation_resolution, angle + filter_orientation_resolution);
    pass.filter(*target_cloud);
  } else {
    pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud_filtered_temp1(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud_filtered_temp2(new pcl::PointCloud<pcl::PointNormal>);

    if (angle < -(PI - filter_orientation_resolution))  // 靠近-PI
    {
      pass.setInputCloud(source_cloud);
      pass.setFilterFieldName("normal_x");
      pass.setFilterLimits(-PI, angle + filter_orientation_resolution);
      pass.filter(*rm_cloud_filtered_temp1);

      pass.setInputCloud(source_cloud);
      pass.setFilterFieldName("normal_x");
      pass.setFilterLimits(angle - filter_orientation_resolution + 2 * PI, PI);
      pass.filter(*rm_cloud_filtered_temp2);

      *target_cloud = *rm_cloud_filtered_temp1 + *rm_cloud_filtered_temp2;
      // 点云和索引都没有排序，可以对应上的
    } else  // 靠近PI
    {
      pass.setInputCloud(source_cloud);
      pass.setFilterFieldName("normal_x");
      pass.setFilterLimits(angle - filter_orientation_resolution, PI);
      pass.filter(*rm_cloud_filtered_temp1);

      pass.setInputCloud(source_cloud);
      pass.setFilterFieldName("normal_x");
      pass.setFilterLimits(-PI, angle + filter_orientation_resolution - 2 * PI);
      pass.filter(*rm_cloud_filtered_temp2);

      *target_cloud = *rm_cloud_filtered_temp1 + *rm_cloud_filtered_temp2;
    }
  }
}

void mm_base_placement::pitch_filter2(double angle, pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                                      pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud) {
  if (fabs(angle) < (PI - filter_orientation_resolution)) {
    pass.setInputCloud(source_cloud);
    pass.setFilterFieldName("normal_y");
    pass.setFilterLimits(angle - filter_orientation_resolution, angle + filter_orientation_resolution);
    pass.filter(*target_cloud);
  } else {
    pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud_filtered_temp1(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud_filtered_temp2(new pcl::PointCloud<pcl::PointNormal>);

    if (angle < -(PI - filter_orientation_resolution))  // 靠近-PI
    {
      pass.setInputCloud(source_cloud);
      pass.setFilterFieldName("normal_y");
      pass.setFilterLimits(-PI, angle + filter_orientation_resolution);
      pass.filter(*rm_cloud_filtered_temp1);

      pass.setInputCloud(source_cloud);
      pass.setFilterFieldName("normal_y");
      pass.setFilterLimits(angle - filter_orientation_resolution + 2 * PI, PI);
      pass.filter(*rm_cloud_filtered_temp2);

      *target_cloud = *rm_cloud_filtered_temp1 + *rm_cloud_filtered_temp2;
    } else  // 靠近PI
    {
      pass.setInputCloud(source_cloud);
      pass.setFilterFieldName("normal_y");
      pass.setFilterLimits(angle - filter_orientation_resolution, PI);
      pass.filter(*rm_cloud_filtered_temp1);

      pass.setInputCloud(source_cloud);
      pass.setFilterFieldName("normal_y");
      pass.setFilterLimits(-PI, angle + filter_orientation_resolution - 2 * PI);
      pass.filter(*rm_cloud_filtered_temp2);

      *target_cloud = *rm_cloud_filtered_temp1 + *rm_cloud_filtered_temp2;
    }
  }
}

}  // namespace mm_base_placement
