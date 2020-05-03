#include "SetPlay.h"

SetPlay::SetPlay() {

  setplay_state = SetPlayState::Init;
  sPositioning = new Positioning();

  states[SetPlayState::Positioning] = sPositioning;
}

SetPlay::~SetPlay() {}

int SetPlay::_Transition(Data *data) {
  if (data->action == Action::Stop) {
    return State::Stop; //Cancel set play
  }
  return State::SetPlay; //Continue set play
}

int SetPlay::_Enter(Data *data) { return 0; }

int SetPlay::_Process(Data *data) {

  std::cout << SetPlayState::str.at(setplay_state) << std::endl;

  if (setplay_state == SetPlayState::Init) {
    setplay_state = SetPlayState::Positioning;
    return 0;
  }

  // Finite State Machine
  int _state = states[setplay_state]->Transition(data);

  if (_state != setplay_state) {
    states[setplay_state]->Exit(data);
    states[_state]->Enter(data);
  } else if (_state == setplay_state) {
    states[setplay_state]->Process(data);
  } else {
  }

  setplay_state = _state;
  return 0;
}

int SetPlay::_Exit(Data *data) { return 0; }
