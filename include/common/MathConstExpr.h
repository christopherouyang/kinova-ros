#pragma once

// const paramter in math
constexpr double PI = 3.14159265358979323846;

constexpr int ANGLES_OF_LOOP = 360;
constexpr double RAD_TO_DEG = 180 / PI;
constexpr double DEG_TO_RAD = PI / 180;

/*****************
 * Transform the angle to [-pi, pi]
 * @param input the input angle which can be any value
 * @return the angle in range [-pi, pi] corresponded to the input angle
 * */
inline double tranformAngle(double input) {
  double output = input;
  while (output > PI) {
    output -= 2 * PI;
  }
  while (output < -PI) {
    output += 2 * PI;
  }
  return output;
}

/*****************
 * Transform the angle to [-pi/2, pi/2] and return the direction
 * @param input the input angle which can be any value
 * @return the direction of the output angle
 * */
inline int computeDirection(double &input) {
  if (input < PI / 2 && input > -PI / 2) {
    return 1;
  }
  input += input > PI / 2 ? -PI : PI;
  return -1;
}

inline int sign(double input) {
  return input < 0 ? -1 : 1;
}