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

/* Definition set play positions on global coordinate */
namespace SetPlayPositions {

namespace My {
namespace KickOff {
const float Alpha_x = 0.0;
const float Alpha_y = -1.0;
const float Beta_x = 0.0;
const float Beta_y = -1.3;
const float Gamma_x = 0.0;
const float Gamma_y = 0.0;
const float Delta_x = 0.0;
const float Delta_y = 0.0;
} // namespace MyKickOff

namespace FreeKick {
const float Alpha_x = 0.0;
const float Alpha_y = -0.5;
const float Beta_x = 0.0;
const float Beta_y = -1.3;
const float Gamma_x = 0.0;
const float Gamma_y = 0.0;
const float Delta_x = 0.0;
const float Delta_y = 0.0;
} // namespace MyFreeKick

namespace ThrowIn {
const float Alpha_x = 0.0;
const float Alpha_y = -0.5;
const float Beta_x = 0.0;
const float Beta_y = -1.3;
const float Gamma_x = 0.0;
const float Gamma_y = 0.0;
const float Delta_x = 0.0;
const float Delta_y = 0.0;
} // namespace MyThrowIn

namespace GoalKick {
const float Alpha_x = 0.0;
const float Alpha_y = -0.5;
const float Beta_x = 0.0;
const float Beta_y = -1.3;
const float Gamma_x = 0.0;
const float Gamma_y = 0.0;
const float Delta_x = 0.0;
const float Delta_y = 0.0;
} // namespace MyGoalKick

namespace CornerKick {
const float Alpha_x = 0.0;
const float Alpha_y = -0.5;
const float Beta_x = 0.0;
const float Beta_y = -1.3;
const float Gamma_x = 0.0;
const float Gamma_y = 0.0;
const float Delta_x = 0.0;
const float Delta_y = 0.0;
} // namespace MyCornerKick
} // namespace My

namespace Opp {
namespace KickOff {
const float Alpha_x = 0.0;
const float Alpha_y = -0.5;
const float Beta_x = 0.0;
const float Beta_y = -1.3;
const float Gamma_x = 0.0;
const float Gamma_y = 0.0;
const float Delta_x = 0.0;
const float Delta_y = 0.0;
} // namespace MyKickOff

namespace FreeKick {
const float Alpha_x = 0.0;
const float Alpha_y = -0.5;
const float Beta_x = 0.0;
const float Beta_y = -1.3;
const float Gamma_x = 0.0;
const float Gamma_y = 0.0;
const float Delta_x = 0.0;
const float Delta_y = 0.0;
} // namespace MyFreeKick

namespace ThrowIn {
const float Alpha_x = 0.0;
const float Alpha_y = -0.5;
const float Beta_x = 0.0;
const float Beta_y = -1.3;
const float Gamma_x = 0.0;
const float Gamma_y = 0.0;
const float Delta_x = 0.0;
const float Delta_y = 0.0;
} // namespace MyThrowIn

namespace GoalKick {
const float Alpha_x = 0.0;
const float Alpha_y = -0.5;
const float Beta_x = 0.0;
const float Beta_y = -1.3;
const float Gamma_x = 0.0;
const float Gamma_y = 0.0;
const float Delta_x = 0.0;
const float Delta_y = 0.0;
} // namespace MyGoalKick

namespace CornerKick {
const float Alpha_x = 0.0;
const float Alpha_y = -0.5;
const float Beta_x = 0.0;
const float Beta_y = -1.3;
const float Gamma_x = 0.0;
const float Gamma_y = 0.0;
const float Delta_x = 0.0;
const float Delta_y = 0.0;
} // namespace MyCornerKick
} // namespace Opp

} // namespace SetPlayPositions

#endif