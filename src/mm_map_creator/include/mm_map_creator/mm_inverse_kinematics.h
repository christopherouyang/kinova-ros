#ifndef MM_INVERSE_KINEMATICS_H
#define MM_INVERSE_KINEMATICS_H

#include "mm_map_creator/head_file.h"

namespace mm_inverse_kinematics
{
    class mm_inverse_kinematics
    {
    public:
        mm_inverse_kinematics();
        bool getIkFromPose(geometry_msgs::Pose pose, std::vector<double> agv_joint_values, std::vector<double>& joint_values);
    private:
        int inverse(const double* T, double* q_sols, double q6_des=0.0);
        void forward(const double* q, double* T);
        int SIGN(double x);
        Eigen::Affine3d rg2_to_ee_link_matrix, rg2_to_ee_link_matrix_inverse;
        Eigen::Affine3d base_link_to_agv_matrix;
        double ZERO_THRESH;
        double d1, a2, a3, d4, d5, d6;
        double ur5_support_height;
        double tool_offset;
    };
}
#endif