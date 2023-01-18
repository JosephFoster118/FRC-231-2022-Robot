#pragma once

#include <cmath> //Math library for C language
#include "PeriodicSubsystem.h"
#include "PID.h"
#include "ctre/Phoenix.h"
#include "RobotContext.h"
#include <algorithm>
#include "Ramp.h"
#include <mutex> 
#include <frc/Solenoid.h>
#include "Utility.h"
#include <frc/Smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include "Debounce.h"


class ShooterSubsystem: public PeriodicSubsystem
{
public:
    ShooterSubsystem() = delete;
    ShooterSubsystem(const ShooterSubsystem &) = delete; //Removes the copy constructor
    ShooterSubsystem(std::shared_ptr<RobotContext> context);
    virtual ~ShooterSubsystem(){};

    enum ShooterMode{STOP, SHOOT};
    enum TurretMode{AUTO, MANUAL, SET, FIXED};

    void setStop();
    void changeSpeedBy(double s);
    void setShootSpeed(double s);
    void toggleKeepShooterRunning();
    void setTurretSpeed(double s);
    void setTurretMode(TurretMode m);
    TurretMode getTurretMode();
    std::string turretModeToString(TurretMode m);
    void setTurretPosition(double p);
    void setTurretOffset();
    void setTurretFixedAngle(double a);
    double getTurretAbsoluteAngle();
    double getTurretRelativeAngle();
    void setBackspinSpeed(double s);

    double getSpeed();
    bool shooterAtSpeed();
    bool aimingAtTarget();
    bool isTurretCalibrated();

    static constexpr double speed_tolerance{50};

    static constexpr double AIM_TOLERANCE{4.0};

    static constexpr double TURRET_SPEED_FACTOR{1.0};
    static constexpr double TURRET_RANGE{480478};
    static constexpr double TURRET_TICKS_PER_DEGREE{2894.44578313}; //todo get this value
    static constexpr double TURRET_DEGREE_RANGE{TURRET_RANGE / TURRET_TICKS_PER_DEGREE};
    

private:
    void disabledInit();
    void teleopInit();
    void autonInit();

    void disabledPeriodic();
    void teleopPeriodic();
    void autonPeiodic(); 
    
    void calibrateTurret();
    void populateDashboard();
    
    //void moveTurret(double power);
    static constexpr double turret_limit{28000.0};
    static constexpr double turret_limit_soft{50000.0};
    static constexpr double turret_limit_factor{0.05};
    
    std::shared_ptr<RobotContext> robot_context;
    TalonFX shooter_motor_left{CANMap::SHOOTER_MOTOR_LEFT};
    TalonFX shooter_motor_right{CANMap::SHOOTER_MOTOR_RIGHT};
    TalonFX turret_motor{CANMap::TURRET_MOTOR};
    TalonSRX backspin_motor{CANMap::BACKSPIN_MOTOR};
    frc::DigitalInput turret_limit_switch{DigitalIOMap::TURRET_LIMIT_SWITCH};
    Debounce turret_limit_switch_debounce{std::chrono::milliseconds{2000}};
    //PID shooter_pid_high{0.00018, 0.000000180, 0.0}; // Tuning PID value
    //PID shooter_pid_low{0.00016, 0.000000028, 0.0, 750.0}; // Tuning PID value
    PID shooter_pid_low{0.00005, 0.000000400, 0.0}; // Tuning PID value
    PID shooter_pid_high{0.00005, 0.000000400, 0.0}; // Tuning PID value
    PID turret_pid{0.15, 0.000005, 0.0};
    PID turret_fixed_pid{0.28, 0.0, 0.0};
    PID turret_set_pid{0.28, 0.0, 0.0};
    //Ramps
    Ramp shooter_ramp{8};

    std::mutex lock{};

    void doShooter();
    void doTurret();
    ShooterMode mode{STOP}; 
    TurretMode turret_mode{MANUAL}; 
    void stopMode();
    void shootMode();

    double speed{0};
    double backspin_speed{0.0};
    bool keep_shooter_running{false};
    double turret_speed{0.0};
    double turret_offset{0.0};
    double turret_target{0.0};
    double turret_fixed_target{0.0};
    double turret_start_position{0.0};
    double turret_end_position{0.0};
};