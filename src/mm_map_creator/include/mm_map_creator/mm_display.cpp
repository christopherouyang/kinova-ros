#include "mm_map_creator/mm_display.h"

namespace mm_display {
mm_display::mm_display() {
}

void mm_display::SetColorByManipulability(double manipulability, std::vector<double>& rgb_color) {
  // std::vector<double> color_vector = {80, 65, 50};
  std::vector<double> color_vector = {75, 50, 25};
  if (manipulability >= color_vector[0]) {
    rgb_color = {0, 0, 255};  // 蓝色
  } else if (manipulability < color_vector[0] && manipulability >= color_vector[1]) {
    rgb_color = {0, 255, 255};  // 青色
  } else if (manipulability < color_vector[1] && manipulability >= color_vector[2]) {
    rgb_color = {0, 255, 0};  // 绿色
  } else {
    rgb_color = {255, 0, 0};  // 红色
  }
}

void mm_display::display_irm(pcl::PointCloud<pcl::PointNormal>::Ptr irm_cloud, double max_range, float new_resolution,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr rm_cloud_display) {
  display_rm(irm_cloud, max_range, new_resolution, rm_cloud_display);
}

void mm_display::display_rm(pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud, double translation_max_range,
                            float new_resolution, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rm_cloud_display) {
  bool patch = false;
  if (patch)
    new_resolution = 0.02;
  unsigned char max_depth = 32;
  octomap::point3d origin = octomap::point3d(0, 0, 0);
  octomap::OcTree* translation_tree = md.generateBoxTree(origin, translation_max_range, new_resolution);
  pcl::PointCloud<pcl::PointXYZ>::Ptr translation_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  for (octomap::OcTree::leaf_iterator it = translation_tree->begin_leafs(max_depth),
                                      end = translation_tree->end_leafs();
       it != end; ++it) {
    pcl::PointXYZ point;
    point.x = it.getCoordinate()(0);
    point.y = it.getCoordinate()(1);
    point.z = it.getCoordinate()(2);
    translation_cloud_filtered->push_back(point);
  }
  if (patch) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(translation_cloud_filtered);
    pcl::PointXYZ searchPoint;
    searchPoint.x = 0.868;
    searchPoint.y = 0.109;
    searchPoint.z = 1.005;
    std::vector<int> point_id;
    std::vector<float> point_distance;
    kdtree.radiusSearch(searchPoint, 0.15, point_id, point_distance);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_patch(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < point_id.size(); i++) {
      cloud_patch->push_back(translation_cloud_filtered->points[point_id[i]]);
    }
    translation_cloud_filtered->clear();
    *translation_cloud_filtered = *cloud_patch;
  }

  std::multimap<const std::vector<double>, double> multi_voxel_color;

  int K = 1;
  std::vector<int> translation_point_idx_vec(K);
  std::vector<float> k_near_trans_distance(K);
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> translation_octree(new_resolution);
  translation_octree.setInputCloud(translation_cloud_filtered);
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
    center_point[0] = translation_cloud_filtered->points[translation_point_idx_vec[0]].x;
    center_point[1] = translation_cloud_filtered->points[translation_point_idx_vec[0]].y;
    center_point[2] = translation_cloud_filtered->points[translation_point_idx_vec[0]].z;
    multi_voxel_color.insert(std::make_pair(center_point, manipulability));
  }

  std::multiset<std::pair<double, std::vector<double> > > rm_set;
  for (int i = 0; i < translation_cloud_filtered->size(); i++) {
    std::vector<double> center_point(3);
    center_point[0] = translation_cloud_filtered->points[i].x;
    center_point[1] = translation_cloud_filtered->points[i].y;
    center_point[2] = translation_cloud_filtered->points[i].z;

    int voxel_number = multi_voxel_color.count(center_point);
    if (voxel_number > 0) {
      std::vector<double> color(voxel_number);
      int count = 0;
      for (auto it = multi_voxel_color.lower_bound(center_point); it != multi_voxel_color.upper_bound(center_point);
           ++it) {
        color[count++] = it->second;
      }
      // double manip = * std::max_element(std::begin(color), std::end(color));
      // double manip = * std::min_element(std::begin(color), std::end(color));
      double manip = std::accumulate(std::begin(color), std::end(color), 0.0) / color.size();
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
