#include "TwoBallAuton.h"
#include "Debounce.h"
using namespace std::chrono_literals;
using namespace std;

TwoBallAuton::TwoBallAuton(std::shared_ptr<RobotContext> context):AutonScript("Test") //Call the constructor of AutonScript
{
    robot_context = context;
}

void TwoBallAuton::script()
{
    DriveSubsystem* drive = dynamic_cast<DriveSubsystem*>(robot_context->sub_systems["Drive"].get());
	IntakeSubsystem* intake = dynamic_cast<IntakeSubsystem*>(robot_context->sub_systems["Intake"].get());
	ShooterSubsystem* shooter = dynamic_cast<ShooterSubsystem*>(robot_context->sub_systems["Shooter"].get());

    shooter->setTurretPosition(-5.0);
    intake->setIntakeDeployed(true);
    sleep(std::chrono::milliseconds(250));
    SCRIPT_RUN_CHECK;
    intake->setIntake(1.0);
    drive->setAutoDriveSpeed(0.5);
    drive->driveDistance(4.0);
    waitUntilTrue([drive]()->bool
    {
        return drive->isActionFinished();
    });
    SCRIPT_RUN_CHECK;
    intake->setIntake(0.0);
    intake->setIntakeDeployed(false);
    sleep(std::chrono::milliseconds{250});
    SCRIPT_RUN_CHECK;
    drive->rotateTo(180.0);
    waitUntilTrue([drive]()->bool
    {
        return drive->isActionFinished();
    });
    SCRIPT_RUN_CHECK;
    sleep(250ms);
    waitUntilTrue([drive]()->bool
    {
        return drive->isActionFinished();
    });
    SCRIPT_RUN_CHECK;
    shooter->setTurretMode(ShooterSubsystem::AUTO);
    shooter->setBackspinSpeed(0.32);
    sleep(1s);
    SCRIPT_RUN_CHECK;
    shooter->setShootSpeed(9250);
    
    sleep(std::chrono::milliseconds{1000});
    SCRIPT_RUN_CHECK;
    Debounce shoot_debounce(250ms);
    waitUntilTrue([shooter, &shoot_debounce]()->bool
    {
        shoot_debounce.setValue(shooter->shooterAtSpeed());
        return shoot_debounce.getValue();
    });
    SCRIPT_RUN_CHECK;
    sleep(1s);
    intake->setFeeder(-0.5);

    

    // SCRIPT_RUN_CHECK;
    // drive->driveDistance(5.0);
    // waitUntilTrue([drive]()->bool
    // {
    //     return drive->isActionFinished();
    // });
    // SCRIPT_RUN_CHECK;
    // cout << "finished going foward\n";
    // drive->driveDistance(-5.0);
    // waitUntilTrue([drive]()->bool
    // {
    //     return drive->isActionFinished();
    // });
    // SCRIPT_RUN_CHECK;
}