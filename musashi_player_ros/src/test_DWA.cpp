
#include <iostream>
#include <ros/ros.h>
#include <string>

#include "Behavior/MotionControl/DynamicWindowApproach.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_dwa");
  ros::NodeHandle nh;

  DWA::Config _cfg;
  _cfg.max_v = 1.0;
  _cfg.min_v = 0.0;
  _cfg.max_w = M_PI;
  _cfg.max_acc = 10.0;
  _cfg.max_dw = 30.0 * M_PI / 180.0;
  _cfg.v_reso = 0.01;
  _cfg.w_reso = 0.1 * M_PI / 180.0;
  _cfg.v_sample_num = 20;
  _cfg.w_sample_num = 20;
  _cfg.dt = 0.01;
  _cfg.T = 1.0;
  _cfg.weight_v = 0.1;
  _cfg.weight_h = 0.1;
  _cfg.weight_c = 0.3;

  DWA::Pose _pose = DWA::Pose{0.0, 0.0, 0.0};
  DWA::Velo _velo = DWA::Velo{0.0, 0.0};
  DWA::Point _goal = DWA::Point{1.0, 1.0};
  DWA::PointSet obs;
  obs.points.clear();

  DWA::DynamicWindow dw;
  DWA::createDynamicWindow(_velo, _cfg, &dw);

  float cost_max = -INFINITY;
	DWA::Velo best_velo;
  std::vector<DWA::Velo> v_set;
  std::vector<float> cost_velo;
  std::vector<float> cost_heading;

  for (int v = 0; v < dw.possibleV.size(); v++) {
    for (int w = 0; w < dw.possibleW.size(); w++) {
      DWA::Velo tmp;
      tmp.liner = dw.possibleV[v];
      tmp.angular = dw.possibleW[w];

      // calculate next pose by motion model using the control input
      DWA::Pose _X = motion(_pose, tmp, _cfg.dt, _cfg.T);

      v_set.push_back(tmp);
      cost_velo.push_back(DWA::calcVelocityCost(tmp, _cfg));
      cost_heading.push_back(calcHeadingCost(_X, _goal));
    }
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
    float cost = _cfg.weight_v * cost_velo[i] + _cfg.weight_h * cost_heading[i];
    //   /* + cfg.clearance * calcCrearnaceCost(X, tmp_velo, obs, cfg); */

    std::cout << v_set[i].liner << "," << v_set[i].angular << "," << cost << std::endl;

    if (cost > cost_max) {
      cost_max = cost;
      best_velo = v_set[i];
    }
  }

	std::cout << "Best:" << best_velo.liner << "," << best_velo.angular << std::endl;

  return 0;
}