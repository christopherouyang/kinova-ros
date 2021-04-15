#pragma once

#include <common/MathConstExpr.h>
#include <cmath>

// const parameter for the wheelchair
constexpr double WHEEL_RADIUS = 0.1778;
constexpr double WHEEL_BASE = 0.5692;
constexpr double HALF_WHEEL_BASE = WHEEL_BASE / 2;

constexpr double WHEELCHAIR_RADIUS = 0.95;
constexpr double MAX_LINEAR_VEL = 0.8;
constexpr double DEFAULT_ACC_TIME = 0.7;
constexpr double DEFAULT_LINEAR_VEL = 0.15;
constexpr double DEFAULT_ANGULAR_VEL = 15;

constexpr float FRONT_DIST = 0.9;
constexpr float SIDE_DIST = 0.4;
constexpr float BACK_DIST = 0.25;

const float THETA_FRONT = atan2(SIDE_DIST, 0.88);
const float THETA_BACK = atan2(SIDE_DIST, BACK_DIST);
const float CRITICAL_ANGLE[] = {THETA_FRONT, PI - THETA_BACK, PI + THETA_BACK, 2 * PI - THETA_FRONT};
constexpr float SAFE_DIST[] = {FRONT_DIST, SIDE_DIST, BACK_DIST};

struct WheelchairVel {
  double linearVel;
  double angularVel;
  WheelchairVel() : linearVel(0.0), angularVel(0.0) {
  }
  WheelchairVel(double a, double b) : linearVel(a), angularVel(b) {
  }
};

struct WheelchairPos {
  double x;
  double y;
  double angle;
  WheelchairPos() : x(0.0), y(0.0), angle(0.0) {
  }
  WheelchairPos(double a, double b, double c) : x(a), y(b), angle(c) {
  }
};

inline float GetSafeDistance(float angle) {
  if (angle <= CRITICAL_ANGLE[0] || angle > CRITICAL_ANGLE[3]) {
    return SAFE_DIST[0] / cos(angle);
  }
  if (angle <= CRITICAL_ANGLE[1]) {
    return SAFE_DIST[1] / sin(angle);
  }
  if (angle <= CRITICAL_ANGLE[2]) {
    return -SAFE_DIST[2] / cos(angle);
  }
  return -SAFE_DIST[1] / sin(angle);
}
