#include "Behavior.h"

#define IS_TRACE 0
#if IS_TRACE == 1
#define TRACE(msg) std::cout<<msg<<std::endl;
#else
#define TRACE(msg)
#endif

Behavior::Behavior() {
  // Instance of each state obejcts
  sStop = new Stop();
  sSetPlay = new SetPlay();

  // Append state set
  states[State::Stop] = sStop;
  states[State::SetPlay] = sSetPlay;
}

Behavior::~Behavior() {}

void Behavior::main(Data *data) {

  TRACE("State: "+State::str.at(data->state));

  if (data->state == State::Init) {
    data->state = State::Stop;
    return;
  }

  // Finite State Machine
  int _state = states[data->state]->Transition(data);

  if (_state != data->state) {
    states[data->state]->Exit(data);
    states[_state]->Enter(data);
  } else if (_state == data->state) {
    states[data->state]->Process(data);
  } else {
  }

  data->last_state = data->state;
  data->state = _state;
  return;
}
