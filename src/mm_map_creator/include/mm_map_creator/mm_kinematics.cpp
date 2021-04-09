#include "mm_map_creator/mm_kinematics.h"

namespace mm_kinematics {
mm_kinematics::mm_kinematics() {
  set_dh_param();
}

static std::vector<double> convertPoseToPoseVector(const Eigen::Affine3d pose) {
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << pose(0, 0), pose(0, 1), pose(0, 2), pose(1, 0), pose(1, 1), pose(1, 2), pose(2, 0), pose(2, 1),
      pose(2, 2);
  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
  return std::vector<double>{pose(0, 3), pose(1, 3), pose(2, 3), euler_angles(2), euler_angles(1), euler_angles(0)};
}

Eigen::Affine3d mm_kinematics::Dh2HomeMatrix(const std::vector<double> dh) {
  // 对于已经存在的元素可以使用下标对其进行访问
  Eigen::Affine3d transform_matrix = Eigen::Affine3d::Identity();
  transform_matrix(0, 0) = cos(dh[3]);
  transform_matrix(0, 1) = -sin(dh[3]) * cos(dh[0]);
  transform_matrix(0, 2) = sin(dh[3]) * sin(dh[0]);
  transform_matrix(0, 3) = dh[1] * cos(dh[3]);

  transform_matrix(1, 0) = sin(dh[3]);
  transform_matrix(1, 1) = cos(dh[3]) * cos(dh[0]);
  transform_matrix(1, 2) = -cos(dh[3]) * sin(dh[0]);
  transform_matrix(1, 3) = dh[1] * sin(dh[3]);

  transform_matrix(2, 0) = 0;
  transform_matrix(2, 1) = sin(dh[0]);
  transform_matrix(2, 2) = cos(dh[0]);
  transform_matrix(2, 3) = dh[2];

  transform_matrix(3, 0) = 0;
  transform_matrix(3, 1) = 0;
  transform_matrix(3, 2) = 0;
  transform_matrix(3, 3) = 1;

  return transform_matrix;
}

Eigen::Affine3d mm_kinematics::GetTotalHomoMatrix() {
  Eigen::Affine3d temp_matrix = Eigen::Affine3d::Identity();
  temp_matrix(0, 0) = 0;
  temp_matrix(0, 1) = 1;
  temp_matrix(1, 0) = 1;
  temp_matrix(1, 1) = 0;
  temp_matrix(2, 2) = -1;
  for (int i = 0; i < JOINT_NUMBER; i++) {
    temp_matrix = temp_matrix * Dh2HomeMatrix(dh_param[i]);
  }
  return temp_matrix;
}

Eigen::Affine3d mm_kinematics::GetTotalHomoMatrix(const std::vector<double> joint_values) {
  Convert2DhParam(joint_values);
  return GetTotalHomoMatrix();
}

Eigen::Affine3d mm_kinematics::GetTotalHomoMatrix(const Eigen::Matrix<double, JOINT_NUMBER, 1> joint_values) {
  Convert2DhParam(joint_values);
  return GetTotalHomoMatrix();
}

std::vector<double> mm_kinematics::GetTotalEndPose(const std::vector<double> joint_values) {
  return convertPoseToPoseVector(GetTotalHomoMatrix(joint_values));
}

std::vector<double> mm_kinematics::GetTotalEndPose(const Eigen::Matrix<double, JOINT_NUMBER, 1> joint_values) {
  return convertPoseToPoseVector(GetTotalHomoMatrix(joint_values));
}

Eigen::Matrix<double, 6, JOINT_NUMBER> mm_kinematics::GetTotalJacobianMatrix() {
  std::vector<Eigen::Affine3d> frame_matrix(JOINT_NUMBER);
  Eigen::Affine3d temp_matrix = Eigen::Affine3d::Identity();
  for (int i = 0; i < JOINT_NUMBER; i++) {
    temp_matrix = temp_matrix * Dh2HomeMatrix(dh_param[i]);
    frame_matrix[i] = temp_matrix;
  }
  Eigen::Matrix<double, 6, JOINT_NUMBER> jacobian_matrix = Eigen::Matrix<double, 6, JOINT_NUMBER>::Zero();
  // translational joint
  for (int i = 0; i < AGV_JOINT_NUMBER; i++) {
    jacobian_matrix(0, i) = frame_matrix[i](0, 2);
    jacobian_matrix(1, i) = frame_matrix[i](1, 2);
    jacobian_matrix(2, i) = frame_matrix[i](2, 2);
    jacobian_matrix(3, i) = 0;
    jacobian_matrix(4, i) = 0;
    jacobian_matrix(5, i) = 0;
  }

  Eigen::Vector3d zi, on, oi, on_minus_oi, jvi, jwi;
  on(0) = temp_matrix(0, 3);
  on(1) = temp_matrix(1, 3);
  on(2) = temp_matrix(2, 3);

  // rotational joint
  for (int i = AGV_JOINT_NUMBER; i < JOINT_NUMBER; i++) {
    zi(0) = frame_matrix[i](0, 2);
    zi(1) = frame_matrix[i](1, 2);
    zi(2) = frame_matrix[i](2, 2);
    oi(0) = frame_matrix[i](0, 3);
    oi(1) = frame_matrix[i](1, 3);
    oi(2) = frame_matrix[i](2, 3);

    on_minus_oi = on - oi;
    jvi = zi.cross(on_minus_oi);
    jwi = zi;

    jacobian_matrix(0, i) = jvi(0);
    jacobian_matrix(1, i) = jvi(1);
    jacobian_matrix(2, i) = jvi(2);
    jacobian_matrix(3, i) = jwi(0);
    jacobian_matrix(4, i) = jwi(1);
    jacobian_matrix(5, i) = jwi(2);
  }
  return jacobian_matrix;
}

Eigen::Matrix<double, 6, JOINT_NUMBER> mm_kinematics::GetTotalJacobianMatrix(const std::vector<double> joint_values) {
  Convert2DhParam(joint_values);
  return GetTotalJacobianMatrix();
}

Eigen::Matrix<double, 6, JOINT_NUMBER> mm_kinematics::GetTotalJacobianMatrix(
    const Eigen::Matrix<double, JOINT_NUMBER, 1> joint_values) {
  Convert2DhParam(joint_values);
  return GetTotalJacobianMatrix();
}

double mm_kinematics::GetManipulability(const std::vector<double> joint_values) {
  Eigen::Matrix<double, 6, JOINT_NUMBER> jacobian_matrix = GetTotalJacobianMatrix(joint_values);
  double manipulability = sqrt((jacobian_matrix * jacobian_matrix.transpose()).determinant()) * 100 / 0.056102;
  return manipulability;
}

void mm_kinematics::set_dh_param() {
  dh_param.resize(JOINT_NUMBER);
  for (int i = 0; i < JOINT_NUMBER; i++) {
    dh_param[i].resize(4);
  }
  // 顺序是alpha, a, d, theta
  dh_param[0] = {PI / 2, 0, -0.2755, 0};
  dh_param[1] = {PI / 2, 0, 0, 0};
  dh_param[2] = {PI / 2, 0, -0.41, 0};
  dh_param[3] = {PI / 2, 0, -0.0098, 0};
  dh_param[4] = {PI / 2, 0, -0.3111, 0};
  dh_param[5] = {PI / 2, 0, 0, 0};
  dh_param[6] = {PI, 0, -0.1638, 0};
}

void mm_kinematics::Convert2DhParam(const std::vector<double> joint_values) {
  for (int i = 0; i < JOINT_NUMBER; i++) {
    dh_param[i][3] = joint_values[i];
  }
}

void mm_kinematics::Convert2DhParam(const Eigen::Matrix<double, JOINT_NUMBER, 1> joint_values) {
  std::vector<double> joint_values_vector(JOINT_NUMBER);
  for (int i = 0; i < JOINT_NUMBER; i++) {
    joint_values_vector[i] = joint_values(i);
  }
  Convert2DhParam(joint_values_vector);
}
}  // namespace mm_kinematics
