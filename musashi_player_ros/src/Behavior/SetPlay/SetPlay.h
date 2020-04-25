#pragma once

#include "SetPlayBase/SetPlayBase.h"

class MyKickOff : public SetPlayBase
{
public:
	MyKickOff(std::string name, int setplay_id) : SetPlayBase(name, setplay_id){}
	~MyKickOff() {}
	int _positioning(Data *data);
	int _wait(Data *data);
	int _run(Data *data);
};

class OppKickOff : public SetPlayBase
{
public:
	OppKickOff(std::string name, int setplay_id) : SetPlayBase(name, setplay_id){}
	~OppKickOff() {}
	int _positioning(Data *data);
	int _wait(Data *data);
	int _run(Data *data);
};

class MyGoalKick : public SetPlayBase
{
public:
	MyGoalKick(std::string name, int setplay_id) : SetPlayBase(name, setplay_id){}
	~MyGoalKick() {}
	int _positioning(Data *data);
	int _wait(Data *data);
	int _run(Data *data);
};