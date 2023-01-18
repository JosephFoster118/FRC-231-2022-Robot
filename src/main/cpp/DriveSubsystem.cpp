#include <chrono>
#include "DriveSubsystem.h"

using namespace std::chrono_literals;

DriveSubsystem::DriveSubsystem(std::shared_ptr<RobotContext> context): PeriodicSubsystem(20ms,"Drive Subsystem")
{
    robot_context = context;

} 

void DriveSubsystem::disabledInit()
{

    left_motor_1.Set(ControlMode::PercentOutput,0);
    left_motor_2.Set(ControlMode::PercentOutput,0);
    right_motor_1.Set(ControlMode::PercentOutput,0);
    right_motor_2.Set(ControlMode::PercentOutput,0);
    setStop();


}

void DriveSubsystem::teleopInit()
{
    left_ramp.reset();
    right_ramp.reset();
}

void DriveSubsystem::autonInit()
{
    angle_offset = robot_context->navx.GetYaw();
    target_angle = 0.0;
    action_finished = false;
    drive_to_finished.forceValue(false);
    
}

void DriveSubsystem::disabledPeriodic()
{
    doDrive();
}

void DriveSubsystem::teleopPeriodic()
{
    doDrive();

}

void DriveSubsystem::autonPeiodic()
{
    doDrive();
}

void DriveSubsystem::doDrive()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    switch(mode)
    {
        case STOP:
        {
            stopMode();   
        }break;
        case DRIVE:
        {
            driveMode();
        }break;
        case DRIVE_DISTANCE:
        {
            driveDistanceMode();
        }break;
        case ROTATE_TO:
        {
            rotateToMode();
        }break;
    }
    frc::SmartDashboard::PutNumber("Drive Distance", left_motor_1.GetSelectedSensorPosition());
}
void DriveSubsystem::stopMode()
{
    left_motor_1.Set(ControlMode::PercentOutput,0);
    left_motor_2.Set(ControlMode::PercentOutput,0);
    right_motor_1.Set(ControlMode::PercentOutput,0);
    right_motor_2.Set(ControlMode::PercentOutput,0);
}

void DriveSubsystem::driveMode()
{
    double left_drive = -teleop_forward + teleop_turn * 1.0; // Add 2 axis together
    double right_drive = teleop_forward + teleop_turn * 1.0; // Add 2 axis together

    left_drive = left_ramp.getSpeed (minMax(left_drive,-1.0, 1.0)); // Set 1.2 -> 1.0
    right_drive = right_ramp.getSpeed (minMax(right_drive,-1.0, 1.0)); // Set 1.2 -> 1.0
    
    left_motor_1.Set(ControlMode::PercentOutput,left_drive);
    right_motor_1.Set(ControlMode::PercentOutput,right_drive);
    left_motor_2.Set(ControlMode::PercentOutput,left_drive);
    right_motor_2.Set(ControlMode::PercentOutput,right_drive);
    frc::SmartDashboard::PutNumber("Drive", right_drive);
}

void DriveSubsystem::setStop()
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    mode = STOP;
}

void DriveSubsystem::setDrive(double forward,double turn)
{
    std::lock_guard<std::mutex> lg (lock); // lg:lock guard
    teleop_forward = forward;
    teleop_turn = turn;
    mode = DRIVE;
}

void DriveSubsystem::driveDistance(double distance)
{
    std::lock_guard<std::mutex> lg (lock);
    foward_pid.resetError();
    drive_target = distance;
    action_finished = false;
    left_motor_offset = left_motor_1.GetSelectedSensorPosition();
    right_motor_offset = right_motor_1.GetSelectedSensorPosition();
    drive_to_finished.forceValue(false);
    mode = DRIVE_DISTANCE;
}

void DriveSubsystem::driveDistanceMode()
{
    double left_position = left_motor_1.GetSelectedSensorPosition() - left_motor_offset;
    double right_position = right_motor_1.GetSelectedSensorPosition() - right_motor_offset;
    double average_position = (left_position - right_position)/2.0;
    average_position /= TICKS_PER_FOOT;
    double drive_power = foward_pid.calculatePID(average_position, drive_target);
    std::cout << drive_power << " "  << average_position - drive_target << std::endl;

    drive_power = minMax(drive_power, -0.85*auto_drive_speed, 0.85*auto_drive_speed);
    double current_angle = robot_context->navx.GetYaw() - robot_context->robot_angle_offset + robot_context->robot_start_angle;
    double remaining_angle = getRotationOffset(current_angle, target_angle);
    double drive_rotate = rotate_correction_PID.calculatePID(-remaining_angle, 0);
    drive_rotate = minMax(drive_rotate, -0.25*auto_drive_speed, 0.25*auto_drive_speed);

    left_motor_1.Set(ControlMode::PercentOutput,drive_power + drive_rotate);
    right_motor_1.Set(ControlMode::PercentOutput,-drive_power + drive_rotate);
    left_motor_2.Set(ControlMode::PercentOutput,drive_power + drive_rotate);
    right_motor_2.Set(ControlMode::PercentOutput,-drive_power + drive_rotate);
    double remaining = std::abs(average_position - drive_target);
    
        drive_to_finished.setValue(remaining < DRIVE_TO_FINISHED_DISTANCE);
        action_finished = drive_to_finished.getValue();
    frc::SmartDashboard::PutNumber("Drive Remaining", remaining);
    frc::SmartDashboard::PutNumber("Drive Remaining Angle", remaining_angle);
}
void DriveSubsystem::rotateToMode()
{
    double current_angle = robot_context->navx.GetYaw() - robot_context->robot_angle_offset + robot_context->robot_start_angle;
    double remaining = getRotationOffset(current_angle, target_angle);
    std::cout << remaining << std::endl;
    double drive_power = rotate_pid.calculatePID(-remaining, 0.0);
    drive_power = minMax(drive_power, -auto_drive_speed, auto_drive_speed);
    left_motor_1.Set(ControlMode::PercentOutput,drive_power);
    right_motor_1.Set(ControlMode::PercentOutput,drive_power);
    left_motor_2.Set(ControlMode::PercentOutput,drive_power);
    right_motor_2.Set(ControlMode::PercentOutput,drive_power);
    if(!action_finished)
    {
        rotate_to_finished.setValue(std::abs(remaining) < 5.0);
        action_finished = rotate_to_finished.getValue();
    }
    frc::SmartDashboard::PutNumber("Drive Remaining Angle", remaining);
}

void DriveSubsystem::rotateTo(double angle)
{
    std::lock_guard<std::mutex> lg (lock);
    mode = ROTATE_TO;
    target_angle = angle;
    action_finished = false;
    rotate_to_finished.forceValue(false);
    rotate_pid.resetError();
}

bool DriveSubsystem::isActionFinished()
{
    std::lock_guard<std::mutex> lg (lock);
    return action_finished;
}

void DriveSubsystem::setAutoDriveSpeed(double s)
{
    std::lock_guard<std::mutex> lg (lock);
    auto_drive_speed = s;
}

double DriveSubsystem::getRemainingDistance()
{
    std::lock_guard<std::mutex> lg (lock);
    double left_position = left_motor_1.GetSelectedSensorPosition() - left_motor_offset;
    double right_position = right_motor_1.GetSelectedSensorPosition() - right_motor_offset;
    double average_position = (left_position - right_position)/2.0;
    average_position /= TICKS_PER_FOOT;
    double remaining = std::abs(average_position - drive_target);
    return remaining;
}

double DriveSubsystem::getRemainingAngle()
{
    std::lock_guard<std::mutex> lg (lock);
    double current_angle = robot_context->navx.GetYaw() - robot_context->robot_angle_offset + robot_context->robot_start_angle;
    double remaining = getRotationOffset(current_angle, target_angle);
    return std::abs(remaining);
}
