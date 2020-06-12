#ifndef SETPLAY_INFO_H
#define SETPLAY_INFO_H

#include <map>
#include <string>

#define SETPLAY_TIME_LIMIT 7000

namespace SetPlayState {
const int Init = 0;
const int Positioning = 1;
const int Wait = 2;
const int Run = 3;

const std::map<int, std::string> str = {
    {Init, "Init"}, {Positioning, "Positioning"}, {Wait, "Wait"}, {Run, "Run"}};
} // namespace SetPlayState


namespace HomePosition{
const float Alpha_x = 1.0;
const float Alpha_y = 0.0;
const float Beta_x = -3.0;
const float Beta_y = 0.0;
const float Gamma_x = -6.0;
const float Gamma_y = 1.0;
const float Delta_x = -6.0;
const float Delta_y = -1.0;
}

/* Definition set play positions on global coordinate */
namespace SetPlayPositions {

namespace My {
namespace KickOff {
const float Alpha_x = -0.1;
const float Alpha_y = 0.5;
const float Beta_x = 0.1;
const float Beta_y = -2.3;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace MyKickOffo

namespace FreeKick {
const float Alpha_x = -0.5;
const float Alpha_y = 0.0;
const float Beta_x = 2.0;
const float Beta_y = 0.0;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace MyFreeKick

namespace ThrowIn {
const float Alpha_x = -0.5;
const float Alpha_y = 0.0;
const float Beta_x = 1.5;
const float Beta_y = 0.0;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace MyThrowIn

namespace GoalKick {
const float Alpha_x = -0.5;
const float Alpha_y = 0.0;
const float Beta_x = 1.5;
const float Beta_y = 0.0;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace MyGoalKick

namespace CornerKick {
const float Alpha_x = -0.5;
const float Alpha_y = 0.0;
const float Beta_x = 1.5;
const float Beta_y = 0.0;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace MyCornerKick

namespace PenaltyKick {
const float Alpha_x = -0.5;
const float Alpha_y = 0.0;
const float Beta_x = HomePosition::Beta_x;
const float Beta_y = HomePosition::Beta_y;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace MyPenaltyKick
} // namespace My

namespace Opp {
namespace KickOff {
const float Alpha_x = -3.3;
const float Alpha_y = 0.5;
const float Beta_x = -4.0;
const float Beta_y = -0.5;
const float Gamma_x = -4.5;
const float Gamma_y = 1.5;
const float Delta_x = -5.0;
const float Delta_y = -2.0;
} // namespace OppKickOff

namespace FreeKick {
const float Alpha_x = -3.5;
const float Alpha_y = 0.0;
const float Beta_x = -3.5;
const float Beta_y = -2.0;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace OppFreeKick

namespace ThrowIn {
const float Alpha_x = -3.3;
const float Alpha_y = 0.0;
const float Beta_x = -3.3;
const float Beta_y = -2.0;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace OppThrowIn

namespace GoalKick {
const float Alpha_x = -3.3;
const float Alpha_y = 0.0;
const float Beta_x = -3.3;
const float Beta_y = -2.0;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace OppGoalKick

namespace CornerKick {
const float Alpha_x = -3.3;
const float Alpha_y = 0.0;
const float Beta_x = -3.3;
const float Beta_y = -2.0;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace OppCornerKick
} // namespace Opp

namespace DropBall {
const float Alpha_x = -1.3;
const float Alpha_y = 0.0;
const float Beta_x = -1.3;
const float Beta_y = -1.3;
const float Gamma_x = HomePosition::Gamma_x;
const float Gamma_y = HomePosition::Gamma_y;
const float Delta_x = HomePosition::Delta_x;
const float Delta_y = HomePosition::Delta_y;
} // namespace DropBall


} // namespace SetPlayPositions

#endif
