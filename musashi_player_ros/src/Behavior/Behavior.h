#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <string>
#include <iostream>
#include <map>

#include "../Data/Data.h"
#include "stop.h"
#include "SetPlay/SetPlay.h"

class Behavior
{
public:
  Behavior();
  ~Behavior();

  void main(Data *data);

private:
  std::map<int, StateBase*> states;

  //State objects
  Stop *sStop;
  SetPlay *sSetPlay;
};

#endif
