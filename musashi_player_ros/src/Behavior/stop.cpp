#include "stop.h"

Stop::Stop()
{
}

int Stop::_Transition(Data *data)
{
  switch(data->action){
  case Action::SetPlay::MyKickOff:
  case Action::SetPlay::MyFreeKick:
  case Action::SetPlay::MyThrowIn:
  case Action::SetPlay::MyGoalKick:
  case Action::SetPlay::MyCornerKick:
  case Action::SetPlay::MyPenaltyKick:
  case Action::SetPlay::OppKickOff:
  case Action::SetPlay::OppFreeKick:
  case Action::SetPlay::OppThrowIn:
  case Action::SetPlay::OppGoalKick:
  case Action::SetPlay::OppCornerKick:
  case Action::SetPlay::OppPenaltyKick:
    return State::SetPlay; //Transition to SetPlayState
  }

  return State::Stop;
}

int Stop::_Enter(Data *data)
{
  return 0;
}

int Stop::_Process(Data *data)
{
  data->Uin.x = 0.0;
  data->Uin.y = 0.0;
  data->Uin.omega = 0.0;
  return 0;
}

int Stop::_Exit(Data *data)
{
  return 0;
}
