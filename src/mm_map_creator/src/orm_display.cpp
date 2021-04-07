// 这个程序是使用改进之后的方法进行计算基座的位置，并进行数据显示，计算速度快
#include "mm_map_creator/mm_base_placement.h"
#include "mm_map_creator/mm_communication.h"
#include "mm_map_creator/mm_display.h"

double mm_rand(double dmin, double dmax, int times) {
  int max = dmax * times;
  int min = dmin * times;
  int i_value = rand() % (max - min + 1) + min;
  return double(i_value) / times;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mobile_robot_planner2");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  ros::ServiceClient orm_calculation_client = nh.serviceClient<mm_map_creator::orm_calculation>("orm_calculation");
  mm_map_creator::orm_calculation orm_cal;
  mm_communication::mm_communication mm_com;
  Eigen::Matrix<double, JOINT_NUMBER, 1> mobile_manipulator_pos;

  mobile_manipulator_pos(0, 0) = 0.0;
  mobile_manipulator_pos(1, 0) = 0;
  mobile_manipulator_pos(2, 0) = 0.0;
  mobile_manipulator_pos(3, 0) = 0.0;
  mobile_manipulator_pos(4, 0) = -0.6;
  mobile_manipulator_pos(5, 0) = 0.9;
  mobile_manipulator_pos(6, 0) = -0.3;
  mobile_manipulator_pos(7, 0) = 1.5708;
  mobile_manipulator_pos(8, 0) = 0.0;

  // 下面这组值，用来进行orm显示，并且使用关节采样的点云
  mobile_manipulator_pos(0, 0) = 0;    // x
  mobile_manipulator_pos(1, 0) = 0.0;  // y
  mobile_manipulator_pos(2, 0) = 0.0;  // theta
  mobile_manipulator_pos(3, 0) = 0.0;
  mobile_manipulator_pos(4, 0) = -1.45;
  mobile_manipulator_pos(5, 0) = 1.35;
  mobile_manipulator_pos(6, 0) = -1.9;
  mobile_manipulator_pos(7, 0) = -1.5708;
  mobile_manipulator_pos(8, 0) = 0;
  mm_com.MobileManipulatorPosMove(mobile_manipulator_pos);

  /*************不同方法获取orm的时间***************/
  // // 1. 制作随机的末端位姿
  // double cm_translation_resolution = 0.1, cm_orientation_resolution = 0.1;
  // std::vector<double> target_eef_pose_vector = mm_com.GetCurrentPose(mobile_manipulator_pos);
  // for(int i = 0; i < 6; i ++) cout<<target_eef_pose_vector[i]<<setw(15); cout<<endl;
  // std::vector<std::vector<double> > eef_vector_vector(30, target_eef_pose_vector);
  // for(size_t i = 0; i < eef_vector_vector.size(); i ++)
  // {
  //     for(size_t j = 1; j < 3; j ++)
  //     {
  //         eef_vector_vector[i][j] += mm_rand(-cm_translation_resolution, cm_translation_resolution, 1000);
  //     }
  //     for(size_t j = 3; j < 6; j ++)
  //     {
  //         eef_vector_vector[i][j] += mm_rand(-cm_orientation_resolution, cm_orientation_resolution, 1000);
  //     }
  // }

  // // 2. 调用不同点云的程序
  // std::vector<int> res_vector = {7,6,5,4,3,2};
  // for(int k = 0; k < res_vector.size(); k ++)
  // {
  //     string path = ros::package::getPath("mm_map_creator") + "/data/";
  //     double mean_time = 0;
  //     // 2.1. orm patch
  //     std::string file_name0 = path + str(boost::format("%1%_method%2%.txt") % res_vector[k] % 0);
  //     std::ofstream data_cout0(file_name0);
  //     for(size_t i = 0; i < eef_vector_vector.size(); i ++)
  //     {
  //         std::vector<double> current_agv_pose = {mobile_manipulator_pos(0,0), mobile_manipulator_pos(1,0),
  //         mobile_manipulator_pos(2,0)}; orm_cal.request.method = 0; orm_cal.request.current_agv_pose =
  //         current_agv_pose; orm_cal.request.target_eef_pose_vector = eef_vector_vector[i];
  //         orm_calculation_client.waitForExistence();
  //         ros::Time start_time = ros::Time::now();
  //         orm_calculation_client.call(orm_cal);
  //         data_cout0<<(ros::Time::now() - start_time).toSec() * 1000<<endl;
  //         mean_time = mean_time + (ros::Time::now() - start_time).toSec() * 1000;
  //     }
  //     data_cout0<<mean_time / eef_vector_vector.size()<<endl;
  //     data_cout0.close();
  //     mean_time = 0;

  //     // 2.2. orm good method
  //     std::string file_name1 = path + str(boost::format("%1%_method%2%.txt") % res_vector[k] % 1);
  //     std::ofstream data_cout1(file_name1);
  //     for(size_t i = 0; i < eef_vector_vector.size(); i ++)
  //     {
  //         std::vector<double> current_agv_pose = {mobile_manipulator_pos(0,0), mobile_manipulator_pos(1,0),
  //         mobile_manipulator_pos(2,0)}; orm_cal.request.method = 1; orm_cal.request.current_agv_pose =
  //         current_agv_pose; orm_cal.request.target_eef_pose_vector = eef_vector_vector[i];
  //         orm_calculation_client.waitForExistence();
  //         ros::Time start_time = ros::Time::now();
  //         orm_calculation_client.call(orm_cal);
  //         data_cout1<<(ros::Time::now() - start_time).toSec() * 1000<<endl;
  //         mean_time = mean_time + (ros::Time::now() - start_time).toSec() * 1000;
  //     }
  //     data_cout1<<mean_time / eef_vector_vector.size()<<endl;
  //     data_cout1.close();

  //     // // 2.3. orm bad method
  //     // std::string file_name2 = path + str(boost::format("%1%_method%2%.txt") % res_vector[k] % 2);
  //     // std::ofstream data_cout2(file_name2);
  //     // for(size_t i = 0; i < eef_vector_vector.size(); i ++)
  //     // {
  //     //     std::vector<double> current_agv_pose = {mobile_manipulator_pos(0,0), mobile_manipulator_pos(1,0),
  //     mobile_manipulator_pos(2,0)};
  //     //     orm_cal.request.method = 2;
  //     //     orm_cal.request.current_agv_pose = current_agv_pose;
  //     //     orm_cal.request.target_eef_pose_vector = eef_vector_vector[i];
  //     //     orm_calculation_client.waitForExistence();
  //     //     ros::Time start_time = ros::Time::now();
  //     //     orm_calculation_client.call(orm_cal);
  //     //     data_cout2<<(ros::Time::now() - start_time).toSec() * 1000<<endl;
  //     //     // ROS_INFO_STREAM("Time cost: "<<(ros::Time::now() - start_time).toSec() * 1000<<"ms");
  //     // }
  //     // data_cout2.close();
  // }

  /*************当前位姿下orm的显示****************/
  // std::vector<double> target_eef_pose_vector = mm_com.GetCurrentPose(mobile_manipulator_pos);
  // std::vector<double> current_agv_pose = {mobile_manipulator_pos(0,0), mobile_manipulator_pos(1,0),
  // mobile_manipulator_pos(2,0)}; orm_cal.request.method = 1; orm_cal.request.current_agv_pose = current_agv_pose;
  // orm_cal.request.target_eef_pose_vector = target_eef_pose_vector;

  // ros::Time start_time = ros::Time::now();
  // orm_calculation_client.waitForExistence();
  // orm_calculation_client.call(orm_cal);
  // ROS_INFO_STREAM("Time cost: "<<(ros::Time::now() - start_time).toSec() * 1000<<"ms");

  // pcl::PointCloud< pcl::PointNormal >::Ptr base_pose_cloud(new pcl::PointCloud< pcl::PointNormal >);
  // for(size_t i = 0; i < orm_cal.response.base_pose_cloud.size() / 4.0; i ++)
  // {
  //     pcl::PointNormal point;
  //     point.x = orm_cal.response.base_pose_cloud[4*i + 0];
  //     point.y = orm_cal.response.base_pose_cloud[4*i + 1];
  //     point.z = point.normal_x = point.normal_y = 0;
  //     point.normal_z = orm_cal.response.base_pose_cloud[4*i + 2];
  //     point.curvature = orm_cal.response.base_pose_cloud[4*i + 3];
  //     base_pose_cloud->push_back(point);
  // }
  // ROS_INFO_STREAM("base_pose_cloud size: "<<base_pose_cloud->points.size());

  // 手动计算ORM进行显示，画图时使用
  double range = 0.15;
  pcl::PointCloud<pcl::PointNormal>::Ptr base_pose_cloud(new pcl::PointCloud<pcl::PointNormal>);
  for (int i = 0; i < 50;) {
    pcl::PointNormal point;
    point.x = mm_rand(-range, range, 10000);
    point.y = mm_rand(-range, range, 10000);
    point.z = 0;
    point.normal_x = point.normal_y = 0;
    point.normal_z = mm_rand(-range, range, 10000);
    point.curvature = mm_rand(70, 100, 10000);
    if (sqrt(point.x * point.x + point.y * point.y) < range) {
      base_pose_cloud->push_back(point);
      i++;
    }
  }

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  visualization_msgs::MarkerArray marker_array;
  mm_display::mm_display display;
  display.display_arrow(base_pose_cloud, marker_array, base_pose_cloud->points.size());
  for (int i = 0; i < 2; i++) {
    marker_pub.publish(marker_array);
    ros::Duration(0.5).sleep();
  }
  return 0;
}