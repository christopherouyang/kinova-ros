#ifndef MM_DISCRETIZATION_H
#define MM_DISCRETIZATION_H

#include "mm_map_creator/head_file.h"

namespace mm_discretization {
class mm_discretization {
 public:
  mm_discretization();
  octomap::OcTree* generateBoxTree(const octomap::point3d origin, double max_range, double resolution);
  octomap::OcTree* generateRotationTree(const octomap::point3d origin, double max_range = PI, double resolution = 0.03);
};
}  // namespace mm_discretization

#endif
