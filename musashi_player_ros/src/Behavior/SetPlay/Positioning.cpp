#include "Positioning.h"

Positioning::Positioning() {
  t_start = 0.0;

  // make keys
  map2position = {
      /*
       * regist for My setplay positions
       *
       */
      {{Action::SetPlay::MyKickOff, Role::Alpha},
       Point{SetPlayPositions::My::KickOff::Alpha_x,
             SetPlayPositions::My::KickOff::Alpha_y}},
      {{Action::SetPlay::MyKickOff, Role::Beta},
       Point{SetPlayPositions::My::KickOff::Beta_x,
             SetPlayPositions::My::KickOff::Beta_y}},
      {{Action::SetPlay::MyKickOff, Role::Gamma},
       Point{SetPlayPositions::My::KickOff::Gamma_x,
             SetPlayPositions::My::KickOff::Gamma_y}},
      {{Action::SetPlay::MyKickOff, Role::Delta},
       Point{SetPlayPositions::My::KickOff::Delta_x,
             SetPlayPositions::My::KickOff::Delta_y}},

      {{Action::SetPlay::MyFreeKick, Role::Alpha},
       Point{SetPlayPositions::My::FreeKick::Alpha_x,
             SetPlayPositions::My::FreeKick::Alpha_y}},
      {{Action::SetPlay::MyFreeKick, Role::Beta},
       Point{SetPlayPositions::My::FreeKick::Beta_x,
             SetPlayPositions::My::FreeKick::Beta_y}},
      {{Action::SetPlay::MyFreeKick, Role::Gamma},
       Point{SetPlayPositions::My::FreeKick::Gamma_x,
             SetPlayPositions::My::FreeKick::Gamma_y}},
      {{Action::SetPlay::MyFreeKick, Role::Delta},
       Point{SetPlayPositions::My::FreeKick::Delta_x,
             SetPlayPositions::My::FreeKick::Delta_y}},

      {{Action::SetPlay::MyThrowIn, Role::Alpha},
       Point{SetPlayPositions::My::ThrowIn::Alpha_x,
             SetPlayPositions::My::ThrowIn::Alpha_y}},
      {{Action::SetPlay::MyThrowIn, Role::Beta},
       Point{SetPlayPositions::My::ThrowIn::Beta_x,
             SetPlayPositions::My::ThrowIn::Beta_y}},
      {{Action::SetPlay::MyThrowIn, Role::Gamma},
       Point{SetPlayPositions::My::ThrowIn::Gamma_x,
             SetPlayPositions::My::ThrowIn::Gamma_y}},
      {{Action::SetPlay::MyThrowIn, Role::Delta},
       Point{SetPlayPositions::My::ThrowIn::Delta_x,
             SetPlayPositions::My::ThrowIn::Delta_y}},

      {{Action::SetPlay::MyGoalKick, Role::Alpha},
       Point{SetPlayPositions::My::GoalKick::Alpha_x,
             SetPlayPositions::My::GoalKick::Alpha_y}},
      {{Action::SetPlay::MyGoalKick, Role::Beta},
       Point{SetPlayPositions::My::GoalKick::Beta_x,
             SetPlayPositions::My::GoalKick::Beta_y}},
      {{Action::SetPlay::MyGoalKick, Role::Gamma},
       Point{SetPlayPositions::My::GoalKick::Gamma_x,
             SetPlayPositions::My::GoalKick::Gamma_y}},
      {{Action::SetPlay::MyGoalKick, Role::Delta},
       Point{SetPlayPositions::My::GoalKick::Delta_x,
             SetPlayPositions::My::GoalKick::Delta_y}},

      {{Action::SetPlay::MyCornerKick, Role::Alpha},
       Point{SetPlayPositions::My::CornerKick::Alpha_x,
             SetPlayPositions::My::CornerKick::Alpha_y}},
      {{Action::SetPlay::MyCornerKick, Role::Beta},
       Point{SetPlayPositions::My::CornerKick::Beta_x,
             SetPlayPositions::My::CornerKick::Beta_y}},
      {{Action::SetPlay::MyCornerKick, Role::Gamma},
       Point{SetPlayPositions::My::CornerKick::Gamma_x,
             SetPlayPositions::My::CornerKick::Gamma_y}},
      {{Action::SetPlay::MyCornerKick, Role::Delta},
       Point{SetPlayPositions::My::CornerKick::Delta_x,
             SetPlayPositions::My::CornerKick::Delta_y}},

      /*
       * regist for Opp setplay positions
       *
       */
      {{Action::SetPlay::OppKickOff, Role::Alpha},
       Point{SetPlayPositions::Opp::KickOff::Alpha_x,
             SetPlayPositions::Opp::KickOff::Alpha_y}},
      {{Action::SetPlay::OppKickOff, Role::Beta},
       Point{SetPlayPositions::Opp::KickOff::Beta_x,
             SetPlayPositions::Opp::KickOff::Beta_y}},
      {{Action::SetPlay::OppKickOff, Role::Gamma},
       Point{SetPlayPositions::Opp::KickOff::Gamma_x,
             SetPlayPositions::Opp::KickOff::Gamma_y}},
      {{Action::SetPlay::OppKickOff, Role::Delta},
       Point{SetPlayPositions::Opp::KickOff::Delta_x,
             SetPlayPositions::Opp::KickOff::Delta_y}},

      {{Action::SetPlay::OppFreeKick, Role::Alpha},
       Point{SetPlayPositions::Opp::FreeKick::Alpha_x,
             SetPlayPositions::Opp::FreeKick::Alpha_y}},
      {{Action::SetPlay::OppFreeKick, Role::Beta},
       Point{SetPlayPositions::Opp::FreeKick::Beta_x,
             SetPlayPositions::Opp::FreeKick::Beta_y}},
      {{Action::SetPlay::OppFreeKick, Role::Gamma},
       Point{SetPlayPositions::Opp::FreeKick::Gamma_x,
             SetPlayPositions::Opp::FreeKick::Gamma_y}},
      {{Action::SetPlay::OppFreeKick, Role::Delta},
       Point{SetPlayPositions::Opp::FreeKick::Delta_x,
             SetPlayPositions::Opp::FreeKick::Delta_y}},

      {{Action::SetPlay::OppThrowIn, Role::Alpha},
       Point{SetPlayPositions::Opp::ThrowIn::Alpha_x,
             SetPlayPositions::Opp::ThrowIn::Alpha_y}},
      {{Action::SetPlay::OppThrowIn, Role::Beta},
       Point{SetPlayPositions::Opp::ThrowIn::Beta_x,
             SetPlayPositions::Opp::ThrowIn::Beta_y}},
      {{Action::SetPlay::OppThrowIn, Role::Gamma},
       Point{SetPlayPositions::Opp::ThrowIn::Gamma_x,
             SetPlayPositions::Opp::ThrowIn::Gamma_y}},
      {{Action::SetPlay::OppThrowIn, Role::Delta},
       Point{SetPlayPositions::Opp::ThrowIn::Delta_x,
             SetPlayPositions::Opp::ThrowIn::Delta_y}},

      {{Action::SetPlay::OppGoalKick, Role::Alpha},
       Point{SetPlayPositions::Opp::GoalKick::Alpha_x,
             SetPlayPositions::Opp::GoalKick::Alpha_y}},
      {{Action::SetPlay::OppGoalKick, Role::Beta},
       Point{SetPlayPositions::Opp::GoalKick::Beta_x,
             SetPlayPositions::Opp::GoalKick::Beta_y}},
      {{Action::SetPlay::OppGoalKick, Role::Gamma},
       Point{SetPlayPositions::Opp::GoalKick::Gamma_x,
             SetPlayPositions::Opp::GoalKick::Gamma_y}},
      {{Action::SetPlay::OppGoalKick, Role::Delta},
       Point{SetPlayPositions::Opp::GoalKick::Delta_x,
             SetPlayPositions::Opp::GoalKick::Delta_y}},

      {{Action::SetPlay::OppCornerKick, Role::Alpha},
       Point{SetPlayPositions::Opp::CornerKick::Alpha_x,
             SetPlayPositions::Opp::CornerKick::Alpha_y}},
      {{Action::SetPlay::OppCornerKick, Role::Beta},
       Point{SetPlayPositions::Opp::CornerKick::Beta_x,
             SetPlayPositions::Opp::CornerKick::Beta_y}},
      {{Action::SetPlay::OppCornerKick, Role::Gamma},
       Point{SetPlayPositions::Opp::CornerKick::Gamma_x,
             SetPlayPositions::Opp::CornerKick::Gamma_y}},
      {{Action::SetPlay::OppCornerKick, Role::Delta},
       Point{SetPlayPositions::Opp::CornerKick::Delta_x,
             SetPlayPositions::Opp::CornerKick::Delta_y}},
  };
}

Positioning::~Positioning() {}

int Positioning::_Transition(Data *data) { return SetPlayState::Positioning; }

int Positioning::_Enter(Data *data) {
  t_start = data->T;
  return 0;
}

int Positioning::_Process(Data *data) {
  double elapsed = data->T - t_start;

  if (elapsed > (double)SETPLAY_TIME_LIMIT) {
  }

  Point target_point;
  target_point.x = data->X.pose.x;
  target_point.x = data->X.pose.y;

  Key_SetPlayAndRole key;
  key.first = data->action;
  key.second = data->role;
  target_point = map2position.at(key);

  // by force
  target_point.x = 2.0;
  target_point.y = 0.0;

  //まずはいち
  //そして方向??

  // move to position (x,y)  by DWA algorithms
  DWA::Velo _velo;
  _velo.liner = data->X.velo.x;
  _velo.angular = data->X.velo.omega;

  DWA::Config _cfg;
  _cfg.max_v = 3.5;
  _cfg.min_v = 0.0;
  _cfg.max_w = 2.0 * M_PI;
  _cfg.max_acc = 30.0;
  _cfg.max_dw = 4.0 * M_PI;
  _cfg.v_reso = 0.01;
  _cfg.w_reso = 0.1 * M_PI / 180.0;
  _cfg.v_sample_num = 20;
  _cfg.w_sample_num = 20;
  _cfg.dt = 0.01;
  _cfg.T = 0.05;
  _cfg.weight_v = 0.8;
  _cfg.weight_h = 1.0;
  _cfg.weight_c = 0.3;

  DWA::Pose _pose =
      DWA::Pose{data->X.pose.x, data->X.pose.y, data->X.pose.theta};

  DWA::Point _point = DWA::Point{target_point.x, target_point.y};

  DWA::PointSet obs;
  obs.points.clear();

  data->Uarray.clear();
  DWA::DynamicWindow dw;
  createDynamicWindow(_velo, _cfg, &dw);
  for (int vi = 0; vi < dw.possibleV.size(); vi++) {
    for (int wi = 0; wi < dw.possibleW.size(); wi++) {
      Velo_t tmp;
      tmp.x = dw.possibleV[vi];
      tmp.y = 0;
      tmp.omega = dw.possibleW[wi];
      data->Uarray.push_back(tmp);
    }
  }

  DWA::Velo velo = DWA::planning(_pose, _velo, _point, obs, _cfg);

  std::cout << "X :" << data->X.pose.x << "," << data->X.pose.y << ","
            << data->X.pose.theta << std::endl;
  std::cout << "Target :" << target_point.x << "," << target_point.y
            << std::endl;
  std::cout << "Select :" << velo.liner << "," << velo.angular << std::endl;

  data->Uin.x = velo.liner;
  data->Uin.y = 0.0;
  data->Uin.omega = velo.angular;

  return 0;
}

int Positioning::_Exit(Data *data) { return 0; }