#ifndef POSITIONING_H
#define POSITIONING_H

#include <unordered_map>

#include "../StateBase/StateBase.h"
#include "SetPlayInfo.h"

// Motion control functions
#include "../MotionControl/DynamicWindowApproach.h"
#include "../MotionControl/PurePursuit.h"

struct Point
{
  float x, y;
};

typedef std::pair<int, int> Key_SetPlayAndRole;
typedef std::unordered_map<Key_SetPlayAndRole, Point, boost::hash<Key_SetPlayAndRole>> Map;

class Positioning : public StateBase {
public:
  Positioning();
  ~Positioning();

  int _Transition(Data *data);
  int _Enter(Data *data);
  int _Process(Data *data);
  int _Exit(Data *data);

private:
  double t_start;
  Map map2position;
};

#endif