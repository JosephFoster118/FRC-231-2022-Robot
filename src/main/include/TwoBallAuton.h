#pragma once
#include <iostream>
#include <memory>
#include "RobotContext.h"
#include "AutonScript.h"
#include "DriveSubsystem.h"
#include "IntakeSubsystem.h"
#include "ShooterSubsystem.h"

class TwoBallAuton: public AutonScript // (Inherit everything from AutonScript) We are a child of AutonScript
{
public:
    TwoBallAuton() = delete; //Remove the defult constructor
    TwoBallAuton(std::shared_ptr<RobotContext> context); 

    virtual ~TwoBallAuton(){};

protected: 
    void script();

private:
    std::shared_ptr<RobotContext> robot_context;
};