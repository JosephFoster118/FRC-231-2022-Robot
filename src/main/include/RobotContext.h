#pragma once
#include <frc/XboxController.h>
#include <map>
#include <memory>
#include "PeriodicSubsystem.h"
#include <AHRS.h>
#include <frc/SPI.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"


class RobotContext
{
public:
    RobotContext();
    frc::XboxController pilot_controller{0};
    frc::XboxController copilot_controller{1};
    std::map<std::string, std::shared_ptr<PeriodicSubsystem>> sub_systems;
    AHRS navx{frc::SPI::Port::kMXP};
    std::shared_ptr<nt::NetworkTable> limelight;
    double robot_start_angle{0.0};
    double robot_angle_offset{0.0};
    
};

struct CANMap
{
    static const int DRIVE_MOTOR_LEFT_1{1};
    static const int DRIVE_MOTOR_LEFT_2{2};
    static const int DRIVE_MOTOR_RIGHT_1{3};
    static const int DRIVE_MOTOR_RIGHT_2{4};
    static const int SHOOTER_MOTOR_LEFT{5};
    static const int SHOOTER_MOTOR_RIGHT{6};
    static const int BACKSPIN_MOTOR{15};
    static const int INTAKE_MOTOR{8};
    static const int FEEDER_MOTOR{7};
    static const int PNEUMATIC_CONTROLLER{9};
    static const int TURRET_MOTOR{10};
    static const int LEFT_CLIMB_MOTOR{11};
    static const int RIGHT_CLIMB_MOTOR{12};
    static const int LEFT_TRAVERSAL_MOTOR{13};
    static const int RIGHT_TRAVERSAL_MOTOR{14};
};

struct SolenoidMap
{
    static const int INTAKE{0};
    static const int TRAVERSAL{1};
};

struct DigitalIOMap
{
    static const int FEEDER_BALL_LINEBREAK{4};
    static const int TURRET_LIMIT_SWITCH{3};
};
