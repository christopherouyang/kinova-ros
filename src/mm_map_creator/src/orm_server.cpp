#include "mm_map_creator/mm_base_placement.h"
#include "mm_map_creator/mm_display.h"

int request_count = 0;

class orm_server {
 public:
  orm_server(int cm_number, int des_number) {
    mbp.load_rm(cm_number, des_number);
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

  // std::vector<int> cm_number_vector = {35438, 69055, 258656, 777335, 3998335, 43606025};
  // std::vector<int> des_number_vector = {552, 834, 1482, 2855, 6679, 22583};
  // std::vector<double> cm_resolution_vector = {7,6,5,4,3,2};
  // for(int i = 0; i < cm_resolution_vector.size(); i ++)
  // {
  //     ROS_INFO_STREAM("resolution: "<<cm_resolution_vector[i]);
  //     ros::param::set("default_values/cm_translation_resolution", cm_resolution_vector[i]);
  //     ros::param::set("default_values/cm_orientation_resolution", cm_resolution_vector[i]);
  //     orm_server os(cm_number_vector[i], des_number_vector[i]);
  //     while(ros::ok())
  //     {
  //         if(request_count == 2*30) {request_count = 0; break;}
  //     }
  // }

  int cm_number = 43606025;
  int des_number = 22583;
  orm_server os(cm_number, des_number);
  ROS_INFO("Waiting for orm calculation");
  ros::waitForShutdown();
  return 0;
}

bool orm_server::ormCalculation(mm_map_creator::orm_calculation::Request& req,
                                mm_map_creator::orm_calculation::Response& res) {
  pcl::PointCloud<pcl::PointNormal>::Ptr base_pose_cloud(new pcl::PointCloud<pcl::PointNormal>);
  if (req.target_eef_pose_vector.size() != 6) {
    ROS_ERROR("The format of target pose is wrong!");
    return false;
  }
  if (req.method == 0) {
    if (req.current_agv_pose.size() != 3) {
      ROS_ERROR("The format of agv pose is wrong!");
      return false;
    }
    mbp.get_orm_patch(req.target_eef_pose_vector, req.current_agv_pose, base_pose_cloud);
  } else {
    mbp.get_orm(req.target_eef_pose_vector, base_pose_cloud, req.method);
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