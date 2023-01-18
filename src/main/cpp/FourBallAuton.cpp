#include "FourBallAuton.h"
using namespace std::chrono_literals;
using namespace std;

FourBallAuton::FourBallAuton(std::shared_ptr<RobotContext> context):AutonScript("Four") //Call the constructor of AutonScript
{
    robot_context = context;
}

void FourBallAuton::script()
{
    //robot_context->robot_start_angle = 90.0;
    DriveSubsystem* drive = dynamic_cast<DriveSubsystem*>(robot_context->sub_systems["Drive"].get());
    IntakeSubsystem* intake = dynamic_cast<IntakeSubsystem*>(robot_context->sub_systems["Intake"].get());
	ShooterSubsystem* shooter = dynamic_cast<ShooterSubsystem*>(robot_context->sub_systems["Shooter"].get());
    SCRIPT_RUN_CHECK;
    shooter->setTurretPosition(68.0);
    shooter->setTurretMode(ShooterSubsystem::SET);
    intake->setIntakeDeployed(true);
    shooter->setShootSpeed(11000);
    shooter->setBackspinSpeed(0.22);
    sleep(250ms);
    SCRIPT_RUN_CHECK;
    intake->setIntake(1.0);
    drive->setAutoDriveSpeed(0.85);
    drive->driveDistance(3.5); // First Drive Foward 
    Debounce drive_debounce(150ms);
    waitUntilTrue([drive, &drive_debounce]()-> bool 
    {
        drive_debounce.setValue(drive->getRemainingDistance() < 0.166);
        return drive_debounce.getValue();
    });
    SCRIPT_RUN_CHECK;
    //shooter->setTurretMode(ShooterSubsystem::AUTO);
    //shooter->setBackspinSpeed(0.32);
    drive->setAutoDriveSpeed(1.0);
    drive->rotateTo(119.0); // first turn
    Debounce drive_debounce2(150ms);

    waitUntilTrue([drive, &drive_debounce2]()-> bool 
    {
        drive_debounce2.setValue(drive->getRemainingAngle() < 5.0);
        return drive_debounce2.getValue();
    });
    SCRIPT_RUN_CHECK;
    Debounce shoot_debounce(150ms);
    waitUntilTrue([shooter, &shoot_debounce]()->bool
    {
        shoot_debounce.setValue(shooter->shooterAtSpeed());
        return shoot_debounce.getValue();
    });

    intake->setFeeder(-0.54); //-.65 = original first infeed
    sleep(1000ms);
    SCRIPT_RUN_CHECK;
    drive->rotateTo(113.0); // second turn
    shooter->setTurretMode(ShooterSubsystem::SET);
    shooter->setTurretPosition(0.0);
    intake->setFeeder(0.0);
    shooter->setShootSpeed(0.0);
    shooter->setBackspinSpeed(0.0);
    Debounce drive_debounce3(100ms);
    waitUntilTrue([drive, &drive_debounce3]()-> bool 
    {
        drive_debounce3.setValue(drive->getRemainingAngle() < 5.0);
        return drive_debounce3.getValue();
    });
    SCRIPT_RUN_CHECK;
    drive->setAutoDriveSpeed(1.0);
    drive->driveDistance(17.0);
    Debounce drive_debounce4(150ms);
    waitUntilTrue([drive, &drive_debounce4]()-> bool 
    {
        drive_debounce4.setValue(drive->getRemainingDistance() < 0.166);
        return drive_debounce4.getValue();
    });
    SCRIPT_RUN_CHECK;
    intake->setFeeder(-1.0); // second infeed
    sleep(100ms);
    intake->setFeeder(0.0);
    drive->setAutoDriveSpeed(1.0);
    drive->rotateTo(53.0);
    Debounce drive_debounce5(150ms);
    waitUntilTrue([drive, &drive_debounce5]()-> bool 
    {
        drive_debounce5.setValue(drive->getRemainingAngle() < 5.0);
        return drive_debounce5.getValue();
    });
    SCRIPT_RUN_CHECK;
    drive->setAutoDriveSpeed(0.75);
    drive->driveDistance(6.4333); // Drive to fourth ball  - 1/12
    Debounce drive_debounce6(150ms);
    waitUntilTrue([drive, &drive_debounce6]()-> bool 
    {
        drive_debounce6.setValue(drive->getRemainingDistance() < 0.166);
        return drive_debounce6.getValue();
    });
    SCRIPT_RUN_CHECK;
    drive->driveDistance(-13.0);
    shooter->setShootSpeed(11300);
    shooter->setBackspinSpeed(0.21);
    Debounce drive_debounce7(150ms);
    waitUntilTrue([drive, &drive_debounce7]()-> bool 
    {
        drive_debounce7.setValue(drive->getRemainingDistance() < 0.166);
        return drive_debounce7.getValue();
    });
    SCRIPT_RUN_CHECK;
    drive->setAutoDriveSpeed(1.0);
    drive->rotateTo(-90.0);
    Debounce drive_debounce8(100ms);
    waitUntilTrue([drive, &drive_debounce8]()-> bool 
    {
        drive_debounce8.setValue(drive->getRemainingAngle() < 5.0);
        return drive_debounce8.getValue();
    });
    SCRIPT_RUN_CHECK;
    shooter->setTurretMode(ShooterSubsystem::AUTO);
    sleep(400ms);
    intake->setFeeder(-0.73); // -.75 was original speed
    // cout << "finished going foward\n";
    // drive->driveDistance(-5.0);
    // waitUntilTrue([drive]()->bool
    // {
    //     return drive->isActionFinished();
    // });
    // SCRIPT_RUN_CHECK;
}