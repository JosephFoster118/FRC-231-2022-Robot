
#include "RobotPeriodic.h"

RobotPeriodic::RobotPeriodic(std::chrono::microseconds thread_period, std::string subsystem_name)
{
    name = subsystem_name;
    period = thread_period;
    //startState();
}

void RobotPeriodic::start()
{
    startState(this);
}

void RobotPeriodic::stop()
{
    if(thread != nullptr)
    {
        thread->stop();
    }
}

void RobotPeriodic::startState(RobotPeriodic* child)
{
    std::function<void()> lambda;
    stop();
    switch(robot_state)
    {
        case DISABLED:
        {
            disabledInit();
            thread = std::make_unique<PeriodicThread>(period, name,
            [child]()
            {
                child->disabledPeriodic();
            });
        }break;
        case AUTON:
        {
            autonPeiodic();
            thread = std::make_unique<PeriodicThread>(period, name,
            [child]()
            {
                child->autonPeiodic();
            });
        }break;
        case TELEOP:
        {
            teleopInit();
            thread = std::make_unique<PeriodicThread>(period, name,
            [child]()
            {
                child->teleopPeriodic();
            });
        }break;
    }
    thread->start();
}

void RobotPeriodic::changeState(RobotState new_state)
{
    robot_state = new_state;
    startState(this);
}
 