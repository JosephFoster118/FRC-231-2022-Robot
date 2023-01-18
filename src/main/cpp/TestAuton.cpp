#include "TestAuton.h"
using namespace std::chrono_literals;
using namespace std;

TestAuton::TestAuton(std::shared_ptr<RobotContext> context):AutonScript("Test") //Call the constructor of AutonScript
{
    robot_context = context;
}

void TestAuton::script()
{
    DriveSubsystem* drive = dynamic_cast<DriveSubsystem*>(robot_context->sub_systems["Drive"].get());
    SCRIPT_RUN_CHECK;
    drive->setAutoDriveSpeed(1.0);
    drive->rotateTo(170.0);
    waitUntilTrue([drive]()->bool
    {
        return drive->isActionFinished();
    });
    SCRIPT_RUN_CHECK;
    // cout << "finished going foward\n";
    // drive->driveDistance(-5.0);
    // waitUntilTrue([drive]()->bool
    // {
    //     return drive->isActionFinished();
    // });
    // SCRIPT_RUN_CHECK;
}