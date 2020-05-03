#ifndef STOP_H
#define STOP_H

#include "./StateBase/StateBase.h"

class Stop : public StateBase
{
public:
  Stop();

protected:
  int _Transition(Data *data);
  int _Enter(Data *data);
  int _Process(Data *data);
  int _Exit(Data *data);
};

#endif // STOP_H
