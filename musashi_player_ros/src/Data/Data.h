#ifndef DATA_H
#define DATA_H

#include <chrono> //for time measurement
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>

#include <boost/functional/hash.hpp> //for generate hash key for unordered_map

struct Pose_t {
  float x, y, theta;

  Pose_t() : x(0.0), y(0.0), theta(0.0) {}

  friend std::ostream &operator<<(std::ostream &out, const Pose_t &pose) {
    out << "[x,y,th]=" << pose.x << "," << pose.y << "," << pose.theta;
    return out;
  }
};

struct Velo_t {
  float x, y, omega;

  Velo_t() : x(0.0), y(0.0), omega(0.0) {}

  friend std::ostream &operator<<(std::ostream &out, const Velo_t &velo) {
    out << "[vx, vy, omega]=" << velo.x << "," << velo.y << "," << velo.omega;
    return out;
  }
};

struct State_t {
  Pose_t pose;
  Velo_t velo;
};

class Data {
public:
  Data();
  ~Data();

  void initTime();
  void updateTime();

private:
  std::chrono::system_clock::time_point t_start;
  std::chrono::system_clock::time_point t_now;

public:
  double dt;
  double T;
  int state, last_state;
  int action;
  int color;
  int role;

  State_t X;    // my state vector (x, y, theta, vx, vy, omega)
  State_t Ball; // ball state vector

  Pose_t Xin; // target pose (x, y, theta)
  Velo_t Uin; // control input (vx, vy, omega)

  //for Dynamic Window logging
  std::vector<Velo_t> Uarray;
};

namespace Color {
const int Magenta = 1;
const int Cyan = 2;
} // namespace Color

namespace Role {
const int Alpha = 1;
const int Beta = 2;
const int Gamma = 3;
const int Delta = 4;
} // namespace Role

namespace State {
const int Init = 0;
const int SetPlay = 1;
const int Stop = 255;

namespace InPlay {
const int NotReady = 1;
const int Ready = 2;
const int Run = 3;
} // namespace InPlay

const std::map<int, std::string> str = {{Init, "Init"},
                                        {Stop, "Stop"},
                                        {SetPlay, "SetPlay"},
                                        {InPlay::NotReady, "InPlay::NotReady"},
                                        {InPlay::Ready, "InPlay::NotReady"},
                                        {InPlay::Ready, "InPlay::Run"}};
} // namespace State

namespace Action {
const int Stop = 0;
const int Start = 1;
const int InPlay = 2;

namespace SetPlay {
const int MyKickOff = 10;
const int MyFreeKick = 11;
const int MyThrowIn = 12;
const int MyGoalKick = 13;
const int MyCornerKick = 14;
const int MyPenaltyKick = 15;
const int OppKickOff = 21;
const int OppFreeKick = 22;
const int OppThrowIn = 23;
const int OppGoalKick = 24;
const int OppCornerKick = 25;
const int OppPenaltyKick = 26;
} // namespace SetPlay

const int DropBall = 30;
const int MyGoal = 31;
const int OppGoal = 32;

const std::map<int, std::string> str = {
    {Stop, "Stop"},
    {Start, "Start"},
    {SetPlay::MyKickOff, "My Kick Off"},
    {SetPlay::MyFreeKick, "My Free Kick"},
    {SetPlay::MyThrowIn, "My Throw In"},
    {SetPlay::MyGoalKick, "My Gaol Kick"},
    {SetPlay::MyCornerKick, "My Corner Kick"},
    {SetPlay::MyPenaltyKick, "My Penalty Kick"},
    {SetPlay::OppKickOff, " Opp Kick Off"},
    {SetPlay::OppFreeKick, " Opp Free Kick"},
    {SetPlay::OppThrowIn, " Opp Throw In"},
    {SetPlay::OppGoalKick, " Opp Gaol Kick"},
    {SetPlay::OppCornerKick, " Opp Corner Kick"},
    {SetPlay::OppPenaltyKick, " Opp Penalty Kick"},
};
} // namespace Action

typedef std::pair<std::string, int> KeyCoachBoxCmdAndColor;
typedef std::unordered_map<KeyCoachBoxCmdAndColor, int,
                           boost::hash<KeyCoachBoxCmdAndColor>>
    Cmd2ActMap;

namespace CoachboxCommand {
const std::string Stop = "STOP";
const std::string Start = "START";

const std::string KickOffMagenta = "KM";
const std::string KickOffCyan = "KC";
const std::string FreeKickMagenta = "FM";
const std::string FreeKickCyan = "FC";
const std::string GoalKickMagenta = "GM";
const std::string GoalKickCyan = "GC";
const std::string CornerKickMagenta = "CM";
const std::string CornerKickCyan = "CC";
const std::string ThrowInMagenta = "TM";
const std::string ThrowInCyan = "TC";
const std::string PenaltyKickMagenta = "PM";
const std::string PenaltyKickCyan = "PC";

const Cmd2ActMap mapToAct = {
    {{Stop, Color::Magenta}, Action::Stop},
    {{Stop, Color::Cyan}, Action::Stop},
    {{Start, Color::Magenta}, Action::Start},
    {{Start, Color::Cyan}, Action::Start},
    {{KickOffMagenta, Color::Magenta}, Action::SetPlay::MyKickOff},
    {{KickOffMagenta, Color::Cyan}, Action::SetPlay::OppKickOff},
    {{KickOffCyan, Color::Magenta}, Action::SetPlay::OppKickOff},
    {{KickOffCyan, Color::Cyan}, Action::SetPlay::MyKickOff},
    {{FreeKickMagenta, Color::Magenta}, Action::SetPlay::MyFreeKick},
    {{FreeKickMagenta, Color::Cyan}, Action::SetPlay::OppFreeKick},
    {{FreeKickCyan, Color::Magenta}, Action::SetPlay::OppFreeKick},
    {{FreeKickCyan, Color::Cyan}, Action::SetPlay::MyFreeKick},
    {{GoalKickMagenta, Color::Magenta}, Action::SetPlay::MyGoalKick},
    {{GoalKickMagenta, Color::Cyan}, Action::SetPlay::OppGoalKick},
    {{GoalKickCyan, Color::Magenta}, Action::SetPlay::OppGoalKick},
    {{GoalKickCyan, Color::Cyan}, Action::SetPlay::MyGoalKick},
    {{CornerKickMagenta, Color::Magenta}, Action::SetPlay::MyCornerKick},
    {{CornerKickMagenta, Color::Cyan}, Action::SetPlay::OppCornerKick},
    {{CornerKickCyan, Color::Magenta}, Action::SetPlay::OppCornerKick},
    {{CornerKickCyan, Color::Cyan}, Action::SetPlay::MyCornerKick},
    {{ThrowInMagenta, Color::Magenta}, Action::SetPlay::MyThrowIn},
    {{ThrowInMagenta, Color::Cyan}, Action::SetPlay::OppThrowIn},
    {{ThrowInCyan, Color::Magenta}, Action::SetPlay::OppThrowIn},
    {{ThrowInCyan, Color::Cyan}, Action::SetPlay::MyThrowIn},
    {{PenaltyKickMagenta, Color::Magenta}, Action::SetPlay::MyPenaltyKick},
    {{PenaltyKickMagenta, Color::Cyan}, Action::SetPlay::OppPenaltyKick},
    {{PenaltyKickCyan, Color::Magenta}, Action::SetPlay::OppPenaltyKick},
    {{PenaltyKickCyan, Color::Cyan}, Action::SetPlay::MyPenaltyKick},

};
}

#endif
