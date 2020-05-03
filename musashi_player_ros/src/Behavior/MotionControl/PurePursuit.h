#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <iostream>
#include <math.h>
#include <vector>

namespace PP {

struct Config {
  float look_ahead_distance;
};

struct Velo {
  float liner;
  float angular;
};

struct Point {
  float x, y;
};

struct Pose {
  Point point;
  float theta;
};

static Pose motion(Pose X, Velo U, float dt) {
  Pose _X;
  _X.theta = X.theta + U.angular * dt;
  _X.point.x = X.point.x + U.liner * cosf(X.theta) * dt;
  _X.point.y = X.point.y + U.liner * sinf(X.theta) * dt;
  return _X;
}

static Velo planning(Pose _X, Point _goal, Config _cfg) {
  float dx = _goal.x - _X.point.x;
  float dy = _goal.y - _X.point.y;
  float _x = dx * cos(-_X.theta) - dy * sin(-_X.theta);
  float _y = dx * sin(-_X.theta) + dy * cos(-_X.theta);

  // float L = sqrt(_x * _x + _y * _y);
  float L =
      sqrt(pow(_goal.x - _X.point.x, 2.0) + pow(_goal.y - _X.point.y, 2.0));
  float alpha = atan2(_goal.y - _X.point.y, _goal.x - _X.point.x);
  
  float vr = 0.3; // const
  float Kp = 0.3;
  vr = Kp*(L);

  float w = 2 * vr * sin(alpha) / L;

  Velo _v;
  _v.liner = vr;
  _v.angular = w;

  return _v;
}
}

#endif