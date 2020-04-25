#include "SetPlay.h"

int MyGoalKick::_positioning(Data *data)
{

	//difinition roll

	//global target positiion
	float target_X, target_Y;
	target_X = SetPlay::Positions::MyKickOff::Alpha_x;
	target_X = SetPlay::Positions::MyKickOff::Alpha_y;

	//to local coordinate

	//moveto?

	//if(reached) -> return SetPlay::State::Wait;

	return SetPlay::State::Positioning;
}

int MyGoalKick::_wait(Data *data)
{
	return SetPlay::State::Wait;
}

int MyGoalKick::_run(Data *data)
{
	return SetPlay::State::Run;
}