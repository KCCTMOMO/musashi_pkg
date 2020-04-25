#pragma once

#include <string>
#include <iostream>
#include <map>

#include "../Data/Data.h"
#include "SetPlay/SetPlay.h"

class Behavior
{
public:
    Behavior();
    ~Behavior();

    void main(Data *data);

private:
    std::map<int, StateBase*> states;

    MyKickOff *myKickOff;
    MyGoalKick *myGoalKick;
};