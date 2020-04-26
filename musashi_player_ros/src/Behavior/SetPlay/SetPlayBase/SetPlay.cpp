#include "SetPlayBase.h"

SetPlayBase::SetPlayBase(std::string name, int id)
{
	this->set_play_name = name;
	this->set_play_id = id;
	set_play_state = SetPlay::State::Init;
}

SetPlayBase::~SetPlayBase()
{
}

int SetPlayBase::positioning(Data *data)
{
	//どのセットプレイに関しても位置決め、位置取りは、ここで全部完結させるか
	//

	//set target position X,Y
	if(set_play_id == State::SetPlay::MyKickOff){
		moveto(SetPlay::Positions::MyKickOff::Alpha_x, 
					SetPlay::Positions::MyKickOff::Alpha_y,
					0.0);
	}
	return SetPlay::State::Positioning;
}

int SetPlayBase::wait(Data *data)
{
	return _wait(data);
}

int SetPlayBase::run(Data *data)
{
	return _run(data);
}

int SetPlayBase::_Transition(Data *data)
{
	//transition to InPlay??

	//Not transition, keep this state
	return 0;
}

int SetPlayBase::_Enter(Data *data)
{
	//Initialize this set play state
	// when entering this state
	set_play_state = SetPlay::State::Init;
	return 0;
}

int SetPlayBase::_Process(Data *data)
{
	//Seaquence : positioning -> wait -> run
	//Finite State Machine (implementation by switch type)

	switch (set_play_state) {
		case SetPlay::State::Init:
			t_start = data->T;	//start measurement for positioning state
			set_play_state = SetPlay::State::Positioning;
			return 0;

		case SetPlay::State::Positioning:
			//calculate for control input x,y,omega
			set_play_state = positioning(data);
			break;

		case SetPlay::State::Wait:
			set_play_state = wait(data);
			break;

		case SetPlay::State::Run:
			set_play_state = run(data);
			break;

		default:
			break;
	}
	return 0;
}

int SetPlayBase::_Exit(Data *data)
{
	return 0;
}

//pure virtual functions
int SetPlayBase::_positioning(Data *data){return 0;}
int SetPlayBase::_wait(Data *data){return 0;}
int SetPlayBase::_run(Data *data){return 0;}