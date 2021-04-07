#include "mm_map_creator/mm_discretization.h"

namespace mm_discretization
{
    mm_discretization::mm_discretization()
    {
        // 构造函数
    }

    // generate octree in cartesian space considering x,y and z, with specific resolution.
    octomap::OcTree* mm_discretization::generateBoxTree(const octomap::point3d origin, double max_range, double resolution)
    {
        octomap::OcTree* tree = new octomap::OcTree(resolution/2);
        octomap::Pointcloud p;
        for (float x = origin.x() - max_range; x <= origin.x() + max_range; x += resolution)
        {
            for (float y = origin.y() - max_range; y <= origin.y() + max_range; y += resolution)
            {
                for (float z = origin.z() - max_range; z <= origin.z() + max_range; z += resolution)
                {
                    octomap::point3d point;
                    point.x() = x;
                    point.y() = y;
                    point.z() = z;
                    tree->updateNode(point, true);
                }
            }
        }
        return tree;
    }

    // generate octree for rotation considering roll pitch and yaw, with specific resolution.
    octomap::OcTree* mm_discretization::generateRotationTree(const octomap::point3d origin, double max_range , double resolution)
    {
        octomap::OcTree* tree = new octomap::OcTree(resolution/2);
        octomap::Pointcloud p;
        for (float roll = origin.x() - max_range; roll <= origin.x() + max_range; roll += resolution)
        {
            for (float pitch = origin.y() - max_range; pitch <= origin.y() + max_range; pitch += resolution)
            {
                for (float yaw = origin.z() - max_range; yaw <= origin.z() + max_range; yaw += resolution)
                {
                    octomap::point3d point;
                    point.x() = roll;
                    point.y() = pitch;
                    point.z() = yaw;
                    tree->updateNode(point, true);
                }
            }
        }
        return tree;
    }
}
