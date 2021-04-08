#ifndef MM_DISPLAY_H
#define MM_DISPLAY_H

#include "mm_map_creator/mm_discretization.h"

namespace mm_display {
class mm_display {
 public:
  mm_display();

  void display_map(pcl::PointCloud<pcl::PointNormal>::Ptr rm_cloud, double max_range, float new_resolution,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr rm_cloud_display);
  void display_arrow(pcl::PointCloud<pcl::PointNormal>::Ptr orm_cloud, visualization_msgs::MarkerArray& marker_array,
                     int number = 7);
  void display_arrow(pcl::PointCloud<pcl::PointNormal> orm_cloud, visualization_msgs::MarkerArray& marker_array,
                     int number = 7);

 private:
  mm_discretization::mm_discretization md;
  void SetColorByManipulability(double manipulability, std::vector<double>& rgb_color);  // 根据可操作度上色
};
}  // namespace mm_display
#endif