#include "SetPlay.h"

int MyKickOff::_positioning(Data *data)
{
	return SetPlay::State::Positioning;
}

int MyKickOff::_wait(Data *data)
{
	return SetPlay::State::Wait;
}

int MyKickOff::_run(Data *data)
{
	return SetPlay::State::Run;
}