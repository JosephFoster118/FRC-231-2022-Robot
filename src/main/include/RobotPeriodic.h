#pragma once
#include <memory>
#include <atomic>

#include "PeriodicThread.h"


class RobotPeriodic
{   
public:
    RobotPeriodic() = delete;//Removes the default constructor
    RobotPeriodic(const RobotPeriodic&) = delete;//Removes the copy constructor
    RobotPeriodic(std::chrono::microseconds thread_period, std::string subsystem_name);
    virtual ~RobotPeriodic(){stop();};
    
    void start();
    void stop();
    
    enum RobotState
    {
        DISABLED = 0,
        AUTON,
        TELEOP
    };

    void changeState(RobotState new_state);

protected:

    virtual void disabledInit() = 0;
    virtual void teleopInit() = 0;
    virtual void autonInit() = 0;

    virtual void disabledPeriodic() = 0;
    virtual void teleopPeriodic() = 0;
    virtual void autonPeiodic() = 0;

    void runStatePeriodic();

    std::unique_ptr<PeriodicThread> thread;
    std::atomic<RobotState> robot_state{DISABLED};

    std::chrono::microseconds period;
    std::string name;

    void startState(RobotPeriodic* child);

};

