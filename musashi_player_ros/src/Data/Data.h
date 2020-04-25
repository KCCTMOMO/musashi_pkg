#pragma once

#include <string>
#include <iostream>
#include <chrono>   //for time measurement

struct Pose_t
{
    float x, y, theta;

    Pose_t(): x(0.0), y(0.0), theta(0.0) {}

    friend std::ostream& operator<<(std::ostream& out, const Pose_t& pose)
    {
        out << "[x,y,th]=" << pose.x << "," << pose.y << "," << pose.theta; 
        return out;
    }
};

struct Velo_t
{
    float x, y, omega;

    Velo_t(): x(0.0), y(0.0), omega(0.0) {}
};

struct  State_t
{
    Pose_t pose;
    Velo_t velo;
};

class Data
{
public:
    Data();
    ~Data();

    void initTime();
    void updateTime();

private:
    std::chrono::system_clock::time_point t_start;
    std::chrono::system_clock::time_point t_now;

public:
    double dt;
    double T;
    int state, last_state;

    State_t X;      //my state vector (x, y, theta, vx, vy, omega)
    Velo_t U;       //command input vector (vx, vy, omega)
    State_t Ball;   //ball state vector

    Pose_t Xin; //target pose (x, y, theta)
    Velo_t Uin; //control input (vx, vy, omega)
};

namespace Role
{
    const int Alpha = 1;
    const int Beta = 2;
    const int Gamma = 3;
    const int Delta = 4;
}

namespace State
{
    const int Init = 0;
    const int Stop = 1;

    namespace InPlay
    {
    }
    
    namespace SetPlay
    {
        const int MyKickOff = 10;
        const int MyGoalKick = 11;
        const int MyCornerKick = 12;
        const int MyFreeKick = 13;
        const int MyThrowIn = 14;

        const int OppKickOff = 20;
        const int OppGoalKick = 21;
        const int OppCornerKick = 22;
        const int OppFreeKick = 23;
        const int OppThrowIn = 24;
    }
}



