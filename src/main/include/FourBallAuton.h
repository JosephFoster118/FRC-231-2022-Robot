#pragma once
#include <iostream>
#include <memory>
#include "RobotContext.h"
#include "AutonScript.h"
#include "DriveSubsystem.h"
#include "IntakeSubsystem.h"
#include "ShooterSubsystem.h"
class FourBallAuton: public AutonScript // (Inherit everything from AutonScript) We are a child of AutonScript
{
public:
    FourBallAuton() = delete; //Remove the defult constructor
    FourBallAuton(std::shared_ptr<RobotContext> context); 

    virtual ~FourBallAuton(){};

protected: 
    void script();

private:
    std::shared_ptr<RobotContext> robot_context;
};