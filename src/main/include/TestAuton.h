#pragma once
#include <iostream>
#include <memory>
#include "RobotContext.h"
#include "AutonScript.h"
#include "DriveSubsystem.h"

class TestAuton: public AutonScript // (Inherit everything from AutonScript) We are a child of AutonScript
{
public:
    TestAuton() = delete; //Remove the defult constructor
    TestAuton(std::shared_ptr<RobotContext> context); 

    virtual ~TestAuton(){};

protected: 
    void script();

private:
    std::shared_ptr<RobotContext> robot_context;
};