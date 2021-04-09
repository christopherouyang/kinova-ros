#include "mm_map_creator/mm_display.h"

namespace mm_display {
mm_display::mm_display() {
}

void mm_display::SetColorByManipulability(double manipulability, std::vector<double>& rgb_color) {
  rgb_color[0] = std::max((fabs(manipulability - 60) - 20) / 20 * 255, 0.0);
  rgb_color[0] = std::min(rgb_color[0], 255.0);

  rgb_color[1] = std::max(255 - (fabs(manipulability - 40) - 20) / 20 * 255, 0.0);
  rgb_color[1] = std::min(rgb_color[1], 255.0);

  rgb_color[2] = std::max((manipulability - 40) / 20 * 255, 0.0);
  rgb_color[2] = std::min(rgb_color[2], 255.0);
}

void mm_display::display_map(pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud, double trans_rng, float res,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr rm_cloud_display) {
  unsigned char max_depth = 32;
  octomap::point3d origin = octomap::point3d(0, 0, 0);
  octomap::OcTree* translation_tree = md.generateBoxTree(origin, trans_rng, res);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_trans_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto ite = translation_tree->begin_leafs(max_depth); ite != translation_tree->end_leafs(); ++ite) {
    filter_trans_pcl->push_back(pcl::PointXYZ(ite.getCoordinate()(0), ite.getCoordinate()(1), ite.getCoordinate()(2)));
  }

  std::multimap<const std::vector<double>, double> multi_voxel_color;

  int K = 1;
  std::vector<int> translation_point_idx_vec(K);
  std::vector<float> k_near_trans_distance(K);
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> translation_octree(res);
  translation_octree.setInputCloud(filter_trans_pcl);
  translation_octree.addPointsFromInputCloud();

  int count = 0;

  for (int i = 0; i < rm_cloud->width; i++) {
    pcl::PointXYZ translation_search_point;
    std::vector<double> center_point(3);
    double manipulability;

    translation_search_point.x = rm_cloud->points[i].x;
    translation_search_point.y = rm_cloud->points[i].y;
    translation_search_point.z = rm_cloud->points[i].z;
    manipulability = rm_cloud->points[i].curvature;
    translation_octree.nearestKSearch(translation_search_point, K, translation_point_idx_vec, k_near_trans_distance);
    center_point[0] = filter_trans_pcl->points[translation_point_idx_vec[0]].x;
    center_point[1] = filter_trans_pcl->points[translation_point_idx_vec[0]].y;
    center_point[2] = filter_trans_pcl->points[translation_point_idx_vec[0]].z;
    multi_voxel_color.insert(std::make_pair(center_point, manipulability));
  }

  std::multiset<std::pair<double, std::vector<double> > > rm_set;
  for (int i = 0; i < filter_trans_pcl->size(); i++) {
    std::vector<double> center_point(3);
    center_point[0] = filter_trans_pcl->points[i].x;
    center_point[1] = filter_trans_pcl->points[i].y;
    center_point[2] = filter_trans_pcl->points[i].z;

    int voxel_number = multi_voxel_color.count(center_point);
    if (voxel_number > 0) {
      std::vector<double> color(voxel_number);
      int count = 0;
      for (auto it = multi_voxel_color.lower_bound(center_point); it != multi_voxel_color.upper_bound(center_point);
           ++it) {
        color[count++] = it->second;
      }
      double manip = *std::max_element(std::begin(color), std::end(color));
      // double manip = * std::min_element(std::begin(color), std::end(color));
      // double manip = std::accumulate(std::begin(color), std::end(color), 0.0) / color.size();
      rm_set.insert(make_pair(manip, center_point));
    }
  }
  rm_cloud_display->clear();
  rm_cloud_display->height = 1;

  for (auto begin = --rm_set.begin(), end = --rm_set.end(); end != begin; end--) {
    pcl::PointXYZRGB pointrgb;
    std::vector<double> rgb_color(3);
    SetColorByManipulability(end->first, rgb_color);

    pointrgb.x = end->second[0];
    pointrgb.y = end->second[1];
    pointrgb.z = end->second[2];
    pointrgb.r = rgb_color[0];
    pointrgb.g = rgb_color[1];
    pointrgb.b = rgb_color[2];
    rm_cloud_display->push_back(pointrgb);
  }
}

void mm_display::display_arrow(pcl::PointCloud<pcl::PointNormal>::Ptr orm_cloud,
                               visualization_msgs::MarkerArray& marker_array, int number) {
  display_arrow(*orm_cloud, marker_array, number);
}

void mm_display::display_arrow(pcl::PointCloud<pcl::PointNormal> orm_cloud,
                               visualization_msgs::MarkerArray& marker_array, int number) {
  marker_array.markers.clear();
  for (int i = 0; i < orm_cloud.width; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "root";  //这个和rm不一样，rm的基坐标系是agv_base_link，但是小车的位姿是的基础坐标系是odom
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = orm_cloud.points[i].x;
    marker.pose.position.y = orm_cloud.points[i].y;
    marker.pose.position.z = orm_cloud.points[i].z;
    Eigen::Quaterniond q;
    if (i < number) {
      Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(orm_cloud.points[i].normal_z, Eigen::Vector3d::UnitZ());
      q = yawAngle * pitchAngle * rollAngle;
      marker.pose.position.z = 0.03;
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();
      marker.scale.x = 0.05;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = 1.0;

      std::vector<double> rgb_color(3);
      SetColorByManipulability(orm_cloud.points[i].curvature, rgb_color);

      marker.color.r = rgb_color[0] / 255;
      marker.color.g = rgb_color[1] / 255;
      marker.color.b = rgb_color[2] / 255;
      marker_array.markers.push_back(marker);
    }
  }
}
}  // namespace mm_display
