#pragma once

#include <string>

#include "../../../Data/Data.h"
#include "../../StateBase/StateBase.h"
#include "../../MotionControl.h"

namespace SetPlay
{
    namespace State{
        const int Init = 0;
        const int Positioning = 1;
        const int Wait = 2;
        const int Run = 3;
    }

    namespace Positions
    {
        namespace MyKickOff
        {
            const double Alpha_x = 0.0;
            const double Alpha_y = 0.5;
            const double Beta_x = 0.0;
            const double Beta_y = -1.3;
        }
        namespace OppKickOff
        {
            const double Alpha_x = -3.5;
            const double Alpha_y = 0.0;
            const double Beta_x = -1.0;
            const double Beta_y = -3.5;
            const double Gamma_x = -1.0;
            const double Gamma_y = -3.5;
            const double Delta_x = -6.0;
            const double Delta_y = 0.0;
        }
    }
}

class SetPlayBase : public StateBase
{
public:
    SetPlayBase(std::string name, int id);
    ~SetPlayBase();

private:
    int positioning(Data *data);
    int wait(Data *data);
    int run(Data *data);

protected:
    int _Transition(Data *data);
	int _Enter(Data *data);
	int _Process(Data *data);
	int _Exit(Data *data);

    virtual int _positioning(Data *data) = 0;
    virtual int _wait(Data *data) = 0;
    virtual int _run(Data *data) = 0;

private:
    std::string set_play_name;
    int set_play_id;
    double t_start;
    int set_play_state; //state variable from SetPlay::State
};



