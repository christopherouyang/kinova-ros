#ifndef MM_KINEMATICS_H
#define MM_KINEMATICS_H

#include "mm_map_creator/head_file.h"

namespace mm_kinematics
{
    class mm_kinematics
    {
    public:
        mm_kinematics();
        Eigen::Affine3d GetTotalHomoMatrix(const std::vector<double> joint_values);
        Eigen::Affine3d GetTotalHomoMatrix(const Eigen::Matrix<double,JOINT_NUMBER,1> joint_values);
        Eigen::Matrix<double,6,JOINT_NUMBER> GetTotalJacobianMatrix(const std::vector<double> joint_values);
        Eigen::Matrix<double,6,JOINT_NUMBER> GetTotalJacobianMatrix(const Eigen::Matrix<double,JOINT_NUMBER,1> joint_values);
        double GetManipulability(const std::vector<double> joint_values);
        
    private:
        double ur5_support_height;
        double tool_offset;
        std::vector<std::vector<double> > dh_param;
        Eigen::Affine3d Dh2HomeMatrix(const std::vector<double> dh);  
        void set_dh_param(std::vector<std::vector<double> >& dh_param);
        void Convert2DhParam(const std::vector<double> joint_values, std::vector<std::vector<double> >& dh_param);
        void Convert2DhParam(const Eigen::Matrix<double,JOINT_NUMBER,1> joint_values, std::vector<std::vector<double> >& dh_param);
        Eigen::Affine3d GetTotalHomoMatrix(const std::vector<std::vector<double> > dh_param);
        Eigen::Matrix<double,6,JOINT_NUMBER> GetTotalJacobianMatrix(const std::vector<std::vector<double> > dh_param);
    };
}

#endif