#pragma once


#include <cmath> //Math library for C language
#include "PeriodicSubsystem.h"
#include "PID.h"
#include "ctre/Phoenix.h"
#include "RobotContext.h"
#include <algorithm>
#include "Ramp.h"
#include <mutex> 
#include "frc/Solenoid.h"
#include "Utility.h"
#include "Debounce.h"
#include "frc/smartdashboard/SmartDashboard.h"


class DriveSubsystem: public PeriodicSubsystem
{
public:
    DriveSubsystem() = delete;
    DriveSubsystem(const DriveSubsystem&) = delete;//Removes the copy constructor
    DriveSubsystem(std::shared_ptr<RobotContext> context);
    virtual ~DriveSubsystem(){};

    enum DriveMode{STOP,DRIVE, DRIVE_DISTANCE, ROTATE_TO};


    void setDrive(double forward,double turn);
    void setStop();
    void driveDistance(double distance);
    void rotateTo(double angle);
    void setAutoDriveSpeed(double s);
    double getRemainingDistance();
    double getRemainingAngle();
    

    bool isActionFinished();

    static constexpr double TICKS_PER_FOOT{18888.3};

private:
    void disabledInit();
    void teleopInit();
    void autonInit();

    void disabledPeriodic();
    void teleopPeriodic();
    void autonPeiodic(); 
    
    std::shared_ptr<RobotContext> robot_context;
    TalonFX left_motor_1{CANMap::DRIVE_MOTOR_LEFT_1};
    TalonFX left_motor_2{CANMap::DRIVE_MOTOR_LEFT_2};
    TalonFX right_motor_1{CANMap::DRIVE_MOTOR_RIGHT_1};
    TalonFX right_motor_2{CANMap::DRIVE_MOTOR_RIGHT_2};

    //Ramps
    Ramp left_ramp{8};
    Ramp right_ramp{8};
    //PIDs
    PID foward_pid{0.4, 0.0005, 0, 0.75};
    PID rotate_correction_PID {0.040,0.000015,0.0, 5.0, 0.05};
    PID rotate_pid {0.0090,0.0000100,0.0, 25.0};
    //Auton Helpers
    bool action_finished{false};
    Debounce drive_to_finished{std::chrono::milliseconds{250}};
    Debounce rotate_to_finished{std::chrono::milliseconds{250}};
    static constexpr double DRIVE_TO_FINISHED_DISTANCE{0.166};
    std::mutex lock{};
    double auto_drive_speed{1.0};

    double teleop_forward{0};
    double teleop_turn{0};
    double drive_target{0};
    double left_motor_offset{0.0};
    double right_motor_offset{0.0};
    double angle_offset{0.0};
    double target_angle{0.0};
    
    DriveMode mode{STOP};  
    void doDrive();
    void driveMode();
    void stopMode();
    void driveDistanceMode();
    void rotateToMode();

};