/* Dynamic Window Approach */
/*
  Refference link:
    Fox, D.; Burgard, W.; Thrun, S. (1997). "The dynamic window approach to
  collision avoidance". IEEE Robotics & Automation Magazine. 4 (1): 23–33

  This algorithm to determine the control input vector [v, omega],
  that enables the robot to avoid obstacles while approaching a target point (x,
  y)
  from a set of possible control inputs in the current robot state ([x_t, y_t,
  theta_t] with [v_t, omega_t]).
 */

#ifndef DWA_H
#define DWA_H

#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>

namespace DWA {

struct Rect {
  // rectangle parameter for local map
  float xmin, xmax;
  float ymin, ymax;
};

struct Config {
  /* paramaters for calculation dynamic window
    max_v, min_v, max_w : fow window Vs
    max_acc, max_dw : fow window Vr
    v_reso, w_reso : sampling resolution of control input from window Vs & Vd &
    Vr

    dt :  predict time slice [sec]
    T : predict time [sec]

    weight_v : weight for velocity cost
    weight_h : weight for heading cost
    weight_c : weight for clearance cost

    base : rectangule difinition local map range
  */
  float max_v, min_v;
  float max_w;
  float max_acc, max_dw;
  float v_reso, w_reso;
  int v_sample_num, w_sample_num;

  float dt, T;

  float weight_v;
  float weight_h;
  float weight_c;

  Rect base;
};

struct Velo {
  float liner;
  float angular;
};

struct Point {
  float x, y;
};

struct PointSet {
  std::vector<Point> points;
};

struct Pose {
  float x, y;
  float theta;
};

struct DynamicWindow {
  std::vector<float> possibleV;
  std::vector<float> possibleW;
};

static void createDynamicWindow(Velo velo, Config cfg, DynamicWindow *dw) {

  // Window Vs (definition by original paper)
  float Vs[4] = {cfg.min_v, cfg.max_v, -cfg.max_w, cfg.max_w};

  // Window Vd (definition by original paper)
  float Vd[4] = {
      velo.liner - cfg.max_acc * cfg.dt, velo.liner + cfg.max_acc * cfg.dt,
      velo.angular - cfg.max_dw * cfg.dt, velo.angular + cfg.max_dw * cfg.dt,
  };

  // Dynamic Window
  float DW[4] = {0.0};
  DW[0] = std::max<float>(Vs[0], Vd[0]);
  DW[1] = std::min<float>(Vs[1], Vd[1]);
  DW[2] = std::max<float>(Vs[2], Vd[2]);
  DW[3] = std::min<float>(Vs[3], Vd[3]);

  int nPossibleV = (int)((DW[1] - DW[0]) / cfg.v_reso);
  int nPossibleW = (int)((DW[3] - DW[2]) / cfg.w_reso);

  /*
  *   Sampling from Dynamic Window (Control Input Space)
  */
  for (int i = 0; i <= nPossibleV; i++) {
    float _v = DW[0] + (float)i * cfg.v_reso;
    dw->possibleV.push_back(_v);
  }

  for (int i = 0; i <= nPossibleW; i++) {
    float _w = DW[2] + (float)i * cfg.w_reso;
    dw->possibleW.push_back(_w);
  }

  // for (int i = 0; i < cfg.v_sample_num; i++) {
  //   float _v = (DW[1] - DW[0]) / (float)(cfg.v_sample_num) * (float)(i + 1);
  //   dw->possibleV.push_back(_v);
  // }
  // for (int i = 0; i < cfg.w_sample_num; i++) {
  //   float _w = (DW[3] - DW[2]) / (float)(cfg.w_sample_num) * (float)(i + 1);
  //   dw->possibleW.push_back(_w);
  // }

  return;
}

// Motion model Xt+1=f(Xt, Ut)
static Pose motion(Pose X, Velo U, float dt, float T) {
  Pose _X = X;

  for (float t = 0.0; t <= T; t += dt) {
    Pose tmp;
    tmp.theta = _X.theta + U.angular * dt;
    tmp.x = _X.x + U.liner * cosf(tmp.theta) * dt;
    tmp.y = _X.y + U.liner * sinf(tmp.theta) * dt;
    _X = tmp;
  }

  return _X;
}

static float calcVelocityCost(Velo U, Config cfg) { return U.liner; }

static float calcHeadingCost(Pose X, Point goal) {
  float dx = goal.x - X.x;
  float dy = goal.y - X.y;
  float alpha = atan2(dy, dx);
  float c = alpha - X.theta;
  return M_PI - fabs(atan2(sinf(c), cosf(c)));
}

static float calcCrearnaceCost(Pose X, Velo U, PointSet set_obs, Config cfg) {
  float t = 0.0;
  Pose _X = X;
  float minr = INFINITY;

  while (t < cfg.T) {
    _X = motion(_X, U, cfg.dt, cfg.T);

    for (int i = 0; i < set_obs.points.size(); i++) {
      float dx = _X.x - set_obs.points[i].x;
      float dy = _X.y - set_obs.points[i].y;

      float x = -dx * cos(_X.theta) - dy * sin(_X.theta);
      float y = -dx * (-sin(_X.theta)) - dy * cos(_X.theta);

      if (x <= cfg.base.xmax && x >= cfg.base.xmin && y <= cfg.base.ymax &&
          y >= cfg.base.ymin) {
        return INFINITY;
      }

      float r = sqrtf(dx * dx + dy * dy);
      if (r < minr) {
        minr = r;
      }
    }
    t += cfg.dt;
  }
  return 1.0 / minr;
}

static Velo planning(Pose X, Velo U, Point goal, PointSet obs, Config cfg) {
  // Create Dynamic Window
  // DynamicWindow dw;
  // createDynamicWindow(U, cfg, &dw);
  DynamicWindow dw;
  createDynamicWindow(U, cfg, &dw);

  // Evaluate
  float total_cost = -INFINITY;
  Velo bestV;
  bestV.liner = 0.0;
  bestV.angular = 0.0;

  std::vector<Velo> v_set;
  std::vector<float> cost_velo;
  std::vector<float> cost_heading;

  for (int v = 0; v < dw.possibleV.size(); v++) {
    // std::cout << dw.possibleV[v] << " : ";

    for (int w = 0; w < dw.possibleW.size(); w++) {
      // std::cout << dw.possibleV[w] << " ";
      Velo tmp_velo;
      tmp_velo.liner = dw.possibleV[v];
      tmp_velo.angular = dw.possibleW[w];

      // calculate next pose by motion model
      Pose _X = motion(X, tmp_velo, cfg.dt, cfg.T);

      // calculate cost function
      v_set.push_back(tmp_velo);
      cost_velo.push_back(calcVelocityCost(tmp_velo, cfg));
      cost_heading.push_back(calcHeadingCost(_X, goal));
    }
    // std::cout << std::endl;
  }

  // min-max normalization
  float cost_velo_max = *std::max_element(cost_velo.begin(), cost_velo.end());
  float cost_velo_min = *std::min_element(cost_velo.begin(), cost_velo.end());

  float cost_heading_max =
      *std::max_element(cost_heading.begin(), cost_heading.end());
  float cost_heading_min =
      *std::min_element(cost_heading.begin(), cost_heading.end());

  for (int i = 0; i < v_set.size(); i++) {
    cost_velo[i] =
        (cost_velo[i] - cost_velo_min) / (cost_velo_max - cost_velo_min);

    cost_heading[i] = (cost_heading[i] - cost_heading_min) /
                      (cost_heading_max - cost_heading_min);
  }

  for (int i = 0; i < v_set.size(); i++) {
    float cost = cfg.weight_v * cost_velo[i] + cfg.weight_h * cost_heading[i];
    /* + cfg.clearance * calcCrearnaceCost(X, tmp_velo, obs, cfg); */

    if (cost > total_cost) {
      total_cost = cost;
      bestV = v_set[i];
    }
  }

  return bestV;
}

} // namespace DWA
#endif
