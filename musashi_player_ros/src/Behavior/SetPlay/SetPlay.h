#ifndef SETPLAYSTATE_H
#define SETPLAYSTATE_H

#include <map>
#include <string>

#include "../StateBase/StateBase.h"
#include "Positioning.h"

class SetPlay : public StateBase {
public:
  SetPlay();
  virtual ~SetPlay();

protected:
  int _Transition(Data *data);
  int _Enter(Data *data);
  int _Process(Data *data);
  int _Exit(Data *data);

private:
  double t_start;

  int setplay_state; // state variable from SetPlay::State
  std::map<int, StateBase *> states;

  Positioning *sPositioning;
};


#endif
