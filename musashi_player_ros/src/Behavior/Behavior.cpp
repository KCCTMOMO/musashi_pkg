#include "Behavior.h"

Behavior::Behavior()
{
    myKickOff = new MyKickOff("MyKickOff", State::SetPlay::MyKickOff);
    myGoalKick = new MyGoalKick("MyGoalKick", State::SetPlay::MyGoalKick);

    states[State::SetPlay::MyKickOff] = myKickOff;
    states[State::SetPlay::MyGoalKick] = myGoalKick;
}

Behavior::~Behavior()
{
}

void Behavior::main(Data *data)
{
    if(data->state == State::Init){
        data->state = State::Stop;
        return;
    }

    //Finite State Machine
    int _state = states[data->state]->Transition(data);

    if(_state != data->state){
        states[data->state]->Exit(data);

        states[_state]->Enter(data);
    }
    else if(_state == data->state){
        states[data->state]->Process(data);
    }

    data->last_state = data->state;
    data->state = _state;
    return;
}