
#include "PeriodicSubsystem.h"

PeriodicSubsystem::PeriodicSubsystem(std::chrono::microseconds thread_period, std::string subsystem_name)
{
    name = subsystem_name;
    period = thread_period;
    //startState();
}

void PeriodicSubsystem::start()
{
    startState(this);
}

void PeriodicSubsystem::stop()
{
    if(thread != nullptr)
    {
        thread->stop();
    }
}

void PeriodicSubsystem::startState(PeriodicSubsystem* child)
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
            autonInit();
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

void PeriodicSubsystem::changeState(RobotState new_state)
{
    robot_state = new_state;
    startState(this);
}
