// rosrun mm_map_creator orm_server 1000000

#include "mm_map_creator/mm_base_placement.h"

int request_count = 0;

class orm_server {
 public:
  orm_server(int cm_number) {
    mbp.load_rm(cm_number);
    orm_calculation_service = nh.advertiseService("orm_calculation", &orm_server::ormCalculation, this);
  }

 private:
  ros::NodeHandle nh;
  mm_base_placement::mm_base_placement mbp;
  ros::ServiceServer orm_calculation_service;
  bool ormCalculation(mm_map_creator::orm_calculation::Request& req, mm_map_creator::orm_calculation::Response& res);
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "orm_server");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  orm_server os(atoi(argv[1]));
  ROS_INFO("Waiting for orm calculation");
  ros::waitForShutdown();
  return 0;
}

bool orm_server::ormCalculation(mm_map_creator::orm_calculation::Request& req,
                                mm_map_creator::orm_calculation::Response& res) {
  pcl::PointCloud<pcl::PointNormal>::Ptr base_pose_cloud(new pcl::PointCloud<pcl::PointNormal>);
  std::vector<double> eef_pose = req.target_eef_pose_vector;
  if (eef_pose.size() != 6) {
    ROS_ERROR("The format of target pose is wrong!");
    return false;
  }
  ROS_INFO("The EEF pose is (%f, %f, %f, %f, %f, %f)", eef_pose[0], eef_pose[1], eef_pose[2], eef_pose[3], eef_pose[4],
           eef_pose[5]);
  if (req.method == 0) {
    if (req.current_agv_pose.size() != 3) {
      ROS_ERROR("The format of agv pose is wrong!");
      return false;
    }
    mbp.get_orm_patch(eef_pose, req.filter_res, req.current_agv_pose, base_pose_cloud);
  } else {
    mbp.get_orm(eef_pose, req.filter_res, base_pose_cloud, req.method);
  }

  ROS_INFO_STREAM("Point size: " << base_pose_cloud->points.size());
  res.base_pose_cloud.clear();
  res.base_pose_cloud.resize(base_pose_cloud->points.size() * 4);
  for (size_t i = 0; i < base_pose_cloud->points.size(); i++) {
    res.base_pose_cloud[4 * i + 0] = base_pose_cloud->points[i].x;
    res.base_pose_cloud[4 * i + 1] = base_pose_cloud->points[i].y;
    res.base_pose_cloud[4 * i + 2] = base_pose_cloud->points[i].normal_z;
    res.base_pose_cloud[4 * i + 3] = base_pose_cloud->points[i].curvature;
  }
  res.cloud_size = base_pose_cloud->points.size();
  request_count++;
  return true;
}