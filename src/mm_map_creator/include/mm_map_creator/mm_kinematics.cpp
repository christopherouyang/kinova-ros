#include "mm_map_creator/mm_kinematics.h"

namespace mm_kinematics
{
    mm_kinematics::mm_kinematics()
    {
        set_dh_param(dh_param);
    }

    Eigen::Affine3d mm_kinematics::Dh2HomeMatrix(const std::vector<double> dh)
    {
        // 对于已经存在的元素可以使用下标对其进行访问
        Eigen::Affine3d transform_matrix = Eigen::Affine3d::Identity();
        transform_matrix(0,0) = cos(dh[3]);
        transform_matrix(0,1) = -sin(dh[3]);
        transform_matrix(0,2) = 0;
        transform_matrix(0,3) = dh[1];

        transform_matrix(1,0) = sin(dh[3]) * cos(dh[0]);
        transform_matrix(1,1) = cos(dh[3]) * cos(dh[0]);
        transform_matrix(1,2) = -sin(dh[0]);
        transform_matrix(1,3) = -sin(dh[0]) * dh[2];

        transform_matrix(2,0) = sin(dh[3]) * sin(dh[0]);
        transform_matrix(2,1) = cos(dh[3]) * sin(dh[0]);
        transform_matrix(2,2) = cos(dh[0]);
        transform_matrix(2,3) = cos(dh[0]) * dh[2];

        transform_matrix(3,0) = 0;
        transform_matrix(3,1) = 0;
        transform_matrix(3,2) = 0;
        transform_matrix(3,3) = 1;

        return transform_matrix;
    }

    Eigen::Affine3d mm_kinematics::GetTotalHomoMatrix(const std::vector<std::vector<double> > dh_param)
    {
        Eigen::Affine3d temp_matrix = Eigen::Affine3d::Identity();
        for(int i = 0; i < JOINT_NUMBER; i ++)
        {   
            temp_matrix = temp_matrix * Dh2HomeMatrix(dh_param[i]);
        }
        return temp_matrix;
    }

    Eigen::Affine3d mm_kinematics::GetTotalHomoMatrix(const std::vector<double> joint_values)
    {
        Convert2DhParam(joint_values, dh_param);
        return GetTotalHomoMatrix(dh_param);
    }

    Eigen::Affine3d mm_kinematics::GetTotalHomoMatrix(const Eigen::Matrix<double,JOINT_NUMBER,1> joint_values)
    {
        Convert2DhParam(joint_values, dh_param);
        return GetTotalHomoMatrix(dh_param);
    }

    Eigen::Matrix<double,6,JOINT_NUMBER> mm_kinematics::GetTotalJacobianMatrix(const std::vector<std::vector<double> > dh_param)
    {
        std::vector<Eigen::Affine3d> frame_matrix(JOINT_NUMBER);
        Eigen::Affine3d temp_matrix = Eigen::Affine3d::Identity();
        for(int i = 0; i < JOINT_NUMBER; i ++)
        {
            temp_matrix = temp_matrix * Dh2HomeMatrix(dh_param[i]);
            frame_matrix[i] = temp_matrix;
        }
        Eigen::Matrix<double,6,JOINT_NUMBER> jacobian_matrix = Eigen::Matrix<double,6,JOINT_NUMBER>::Zero();
        // translational joint
        for(int i = 0 ; i < 2; i ++)
        {
            jacobian_matrix(0,i) = frame_matrix[i](0,2);
            jacobian_matrix(1,i) = frame_matrix[i](1,2);
            jacobian_matrix(2,i) = frame_matrix[i](2,2);
            jacobian_matrix(3,i) = 0;
            jacobian_matrix(4,i) = 0;
            jacobian_matrix(5,i) = 0;
        }

        Eigen::Vector3d zi, on, oi, on_minus_oi, jvi, jwi;
        on(0) = temp_matrix(0,3); 
        on(1) = temp_matrix(1,3); 
        on(2) = temp_matrix(2,3); 

        // rotational joint
        for(int i = 2; i < JOINT_NUMBER; i ++)
        {
            zi(0) = frame_matrix[i](0,2);
            zi(1) = frame_matrix[i](1,2);
            zi(2) = frame_matrix[i](2,2);
            oi(0) = frame_matrix[i](0,3);
            oi(1) = frame_matrix[i](1,3);
            oi(2) = frame_matrix[i](2,3);

            on_minus_oi = on - oi;
            jvi = zi.cross(on_minus_oi);
            jwi = zi;

            jacobian_matrix(0,i) = jvi(0);
            jacobian_matrix(1,i) = jvi(1);
            jacobian_matrix(2,i) = jvi(2);
            jacobian_matrix(3,i) = jwi(0);
            jacobian_matrix(4,i) = jwi(1);
            jacobian_matrix(5,i) = jwi(2);
        }
        return jacobian_matrix;
    }

    Eigen::Matrix<double,6,JOINT_NUMBER> mm_kinematics::GetTotalJacobianMatrix(const std::vector<double> joint_values)
    {
        Convert2DhParam(joint_values, dh_param);
        return GetTotalJacobianMatrix(dh_param);
    }

    Eigen::Matrix<double,6,JOINT_NUMBER> mm_kinematics::GetTotalJacobianMatrix(const Eigen::Matrix<double,JOINT_NUMBER,1> joint_values)
    {
        Convert2DhParam(joint_values, dh_param);
        return GetTotalJacobianMatrix(dh_param);
    }

    double mm_kinematics::GetManipulability(const std::vector<double> joint_values)
    {
        Eigen::Matrix<double,6,JOINT_NUMBER> jacobian_matrix = GetTotalJacobianMatrix(joint_values);
        double manipulability = sqrt((jacobian_matrix * jacobian_matrix.transpose()).determinant());
        return 100 * manipulability / 2.23345;
    }

    void mm_kinematics::set_dh_param(std::vector<std::vector<double> >& dh_param)
    {
        if(!ros::param::get("default_values/ur5_support_height",ur5_support_height)) ROS_ERROR("No ur5_support_height specified!");
        ros::param::get("default_values/tool_offset",tool_offset);
        dh_param.resize(JOINT_NUMBER);
        for(int i = 0; i < JOINT_NUMBER; i ++)
        {
            dh_param[i].resize(4);
        }
        // 顺序是alpha, a, d, theta
        dh_param[0] = {-PI/2,   0,      0,      -PI/2};     // the third y is the variable
        dh_param[1] = {-PI/2,   0,      0,       PI/2};     // the third x is the variable
        dh_param[2] = {PI/2,    0,      0,      -PI/2};     // the fourth theta is the variable 这里有-PI/2的初始值
        dh_param[3] = {0,      -0.1395, 0.482 + ur5_support_height,      0};     // 考虑加高
        dh_param[4] = {PI/2,    0,      0.136,      0};     // the fourth theta2 is the variable
        dh_param[5] = {0,      -0.425,  -0.120,     0};     // the fourth theta3 is the variable
        dh_param[6] = {0,      -0.392,  0.093,      0};     // the fourth theta4 is the variable
        dh_param[7] = {PI/2,    0,      0.095,      0};     // the fourth theta5 is the variable
        dh_param[8] = {-PI/2,   0,      0.082 + tool_offset,      0};     // the fourth theta6 is the variable and it's the dh param of modified_tool0 frame.
        // 0.082 0.312 
    }

    void mm_kinematics::Convert2DhParam(const std::vector<double> joint_values, std::vector<std::vector<double> >& dh_param)
    {
        dh_param[0][2] = joint_values[1];
        dh_param[1][2] = joint_values[0];
        dh_param[2][3] = joint_values[2] - PI/2;
        for(int i = 3; i < JOINT_NUMBER; i ++)
        {
            dh_param[i][3] = joint_values[i];
        }
    }

    void mm_kinematics::Convert2DhParam(const Eigen::Matrix<double,JOINT_NUMBER,1> joint_values, std::vector<std::vector<double> >& dh_param)
    {
        std::vector<double> joint_values_vector(JOINT_NUMBER);
        for(int i = 0; i < JOINT_NUMBER; i ++)
        {
            joint_values_vector[i] = joint_values(i);
        }
        Convert2DhParam(joint_values_vector, dh_param);
    }
}
