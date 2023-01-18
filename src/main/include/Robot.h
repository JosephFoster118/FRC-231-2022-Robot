#pragma once
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/AnalogInput.h>
#include <frc/filter/MedianFilter.h>
#include <frc/Compressor.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
//for the Camera 
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <cameraserver/CameraServer.h>

//Our files
#include "AutonScript.h"
#include "PeriodicSubsystem.h"
#include "PeriodicThread.h"
#include "DriveSubsystem.h"
#include "Utility.h"
#include "IntakeSubsystem.h"
#include "Debounce.h"
#include "TestAuton.h"
#include "ShooterSubsystem.h"
#include "TwoBallAuton.h"
#include "FourBallAuton.h"
#include "ClimberSubsystem.h"

//C++ Standard
#include <iostream>
#include <map>
#include <utility>
#include <RobotContext.h>
#include <chrono>

class Robot : public frc::TimedRobot 
{
public:
    Robot();
    virtual ~Robot();

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestPeriodic() override;

    // Deadzones
    static constexpr double DEFAULT_DEADZONE{0.1};

    static constexpr double SLOW_MOTION_FACTOR{0.6};
    static constexpr double LOW_SHOOTER_SPEED{3000.0};
    static constexpr double LOW_BACKSPIN_SPEED{0.25};
    static constexpr double HIGH_SHOOTER_SPEED{9000.0};
    static constexpr double HIGH_BACKSPIN_SPEED{0.32};
    static constexpr double LUDACRIS_SHOOTER_SPEED{10200.0};

    static constexpr double LUDRACRIS_BACKSPIN_SPEED{0.36};
    static constexpr double SHOOTER_INCREMENT_AMOUNT{2000.0};
    static constexpr double TURNING_FACTOR{1.0};

private:

    void setupSmartDashboard();
    void doTurretControls();
    void doShooterControls();

    enum CopilotMode{SHOOTER, CLIMBER};
    std::string copilotModeToString(Robot::CopilotMode mode);
    CopilotMode copilot_mode{SHOOTER};
    std::map<std::string, std::shared_ptr<PeriodicSubsystem>> sub_systems;
    void changeStates(PeriodicSubsystem::RobotState state);
    std::shared_ptr<RobotContext> robot_context;
    std::unique_ptr<AutonScript> selected_auton;
    enum Autons{TWO_BALL, FOUR_BALL};
    frc::SendableChooser<Autons> auto_chooser;
    static const inline std::vector<std::pair<std::string,Autons>> autons = {{"Two Ball",TWO_BALL}, {"Four Ball",FOUR_BALL}};

    //  Input Modifiers
    Debounce intake_debounce{std::chrono::milliseconds{100}};
    Debounce slow_motion_debounce{std::chrono::milliseconds{100}};
    Debounce outtake_debounce{std::chrono::milliseconds{100}};
    Debounce feeder_up_debounce{std::chrono::milliseconds{100}};
    Debounce feeder_down_debounce{std::chrono::milliseconds{100}};
    Debounce toggle_intake_debounce{std::chrono::milliseconds{100}};
    Debounce low_shooter_speed_debounce{std::chrono::milliseconds{100}};
    Debounce high_shooter_speed_debounce{std::chrono::milliseconds{100}};
    Debounce toggle_keep_shooter_running_debounce{std::chrono::milliseconds{100}};
    Debounce climb_up_debounce{std::chrono::milliseconds{100}};
    Debounce climb_down_debounce{std::chrono::milliseconds{100}};
    Debounce auto_aim_debounce{std::chrono::milliseconds{100}};
    Debounce set_aim_debounce{std::chrono::milliseconds{100}};
    Debounce manual_aim_debounce{std::chrono::milliseconds{100}};
    Debounce ludacris_shooter_speed_debounce{std::chrono::milliseconds{100}};
    Debounce turret_left_debounce{std::chrono::milliseconds{100}};
    Debounce turret_right_debounce{std::chrono::milliseconds{100}};
    Debounce auto_center_debounce{std::chrono::milliseconds{100}};
    Debounce set_mode_debounce{std::chrono::milliseconds{100}};
    Debounce face_turret_foward_debounce{std::chrono::milliseconds{100}};
    Debounce change_copilot_mode_debounce{std::chrono::milliseconds{100}};
    Debounce deploy_traversal_debounce{std::chrono::milliseconds{100}};
    Debounce retract_traversal_debounce{std::chrono::milliseconds{100}};
    bool last_change_copilot_mode_value{false};
    bool last_auto_aim_value{false};
    bool last_toggle_keep_shooter_running_value{false};
    bool last_low_shooter_speed_value{false};
    bool last_high_shooter_speed_value{false};
    bool last_toggle_intake_value{false};
    std::vector<double> shooter_speed{};
    int current_speed{0};
    bool last_increase_speed_value{false};
    Debounce increase_shooter_speed_button{std::chrono::milliseconds{100}};
    bool last_decrease_speed_value{false};
    Debounce decrease_shooter_speed_button{std::chrono::milliseconds{100}};
    ShooterSubsystem::TurretMode current_turret_mode{ShooterSubsystem::SET};
    frc::Compressor compressor{CANMap::PNEUMATIC_CONTROLLER, frc::PneumaticsModuleType::REVPH};
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif



