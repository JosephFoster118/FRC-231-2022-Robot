#include "Robot.h"

using std::string;
using std::unique_ptr;
using std::cout;
using std::endl;




void Robot::setupSmartDashboard()
{
	frc::SmartDashboard::init();
	frc::SmartDashboard::SetDefaultNumber("target speed", 0);
	frc::SmartDashboard::SetDefaultNumber("actual speed",0);
	frc::SmartDashboard::SetDefaultBoolean("Intake", false);
	frc::SmartDashboard::SetDefaultBoolean("Feeder", false);
	frc::SmartDashboard::SetDefaultNumber("Drive", 0.0);
	frc::SmartDashboard::SetDefaultNumber("Pressure", 0.0);
	frc::SmartDashboard::SetDefaultBoolean("Keep Intake Running", false);
	frc::SmartDashboard::SetDefaultNumber("LSV", 0.0);
	frc::SmartDashboard::SetDefaultNumber("RSV", 0.0);
	frc::SmartDashboard::SetDefaultBoolean("Shooter At Speed", false);
	frc::SmartDashboard::SetDefaultBoolean("Auto Aim", false);
	frc::SmartDashboard::SetDefaultNumber("turret position", 0.0);
	frc::SmartDashboard::SetDefaultNumber("turret start", 0.0);
	frc::SmartDashboard::SetDefaultNumber("turret end", 0.0);
	frc::SmartDashboard::SetDefaultBoolean("Turret Switch", false);
	frc::SmartDashboard::SetDefaultNumber("Robot Angle", 0.0);
	frc::SmartDashboard::SetDefaultNumber("Turret Absolute Angle", 0.0);
	frc::SmartDashboard::SetDefaultNumber("Turret Relative Angle", 0.0);
	frc::SmartDashboard::SetDefaultString("Turret Mode", "unset");
	frc::SmartDashboard::SetDefaultNumber("Joystick Angle", 0.0);
	frc::SmartDashboard::SetDefaultNumber("Drive Distance", 0.0);
	frc::SmartDashboard::SetDefaultNumber("Drive Remaining", 0.0);
	frc::SmartDashboard::SetDefaultNumber("Drive Remaining Angle", 0.0);
	frc::SmartDashboard::SetDefaultBoolean("Turret Calibrated", false);
	frc::SmartDashboard::SetDefaultString("Copilot Mode", copilotModeToString(copilot_mode));
	frc::SmartDashboard::SetDefaultNumber("BackSpeed", 0.0);
	frc::SmartDashboard::SetDefaultNumber("Left Climber", 0.0);
	frc::SmartDashboard::SetDefaultNumber( "Right Climber", 0.0);
	frc::SmartDashboard::SetDefaultNumber("Ludacris Speed Tuner", 0.0);

	bool first_option = true;
	for(auto p: autons)
	{
		if(first_option)
		{
			auto_chooser.SetDefaultOption(p.first, p.second);
			first_option = false;
		}
		else
		{
			auto_chooser.AddOption(p.first, p.second);
		}
	}
	auto_chooser.SetName("Auto Chooser");
	frc::SmartDashboard::PutData(&auto_chooser);
	
}
Robot::Robot()
{
	robot_context = std::make_shared<RobotContext>();
	DriveSubsystem* drive = new DriveSubsystem{robot_context};
	IntakeSubsystem* intake = new IntakeSubsystem{robot_context};
	ShooterSubsystem* shooter = new ShooterSubsystem{robot_context};
	ClimberSubsystem* climber = new ClimberSubsystem{robot_context};
	
    robot_context->sub_systems.insert(std::pair<std::string,PeriodicSubsystem*>("Drive",static_cast<PeriodicSubsystem*>(drive)));
	robot_context->sub_systems.insert(std::pair<std::string,PeriodicSubsystem*>("Intake",static_cast<PeriodicSubsystem*>(intake)));
	robot_context->sub_systems.insert(std::pair<std::string,PeriodicSubsystem*>("Shooter",static_cast<PeriodicSubsystem*>(shooter)));
	robot_context->sub_systems.insert(std::pair<std::string,PeriodicSubsystem*>("Climber",static_cast<PeriodicSubsystem*>(climber)));
	for(double i = 0; i <= 20000; i+= 2000)
	{
		shooter_speed.emplace_back(i);
	}
	setupSmartDashboard();
	frc::CameraServer::GetInstance()->StartAutomaticCapture();
	compressor.EnableDigital();
}

Robot::~Robot()
{

}

void Robot::AutonomousInit()
{	
	robot_context->robot_angle_offset = robot_context->navx.GetYaw();
	changeStates(PeriodicSubsystem::AUTON);
	if(selected_auton)
	{
		selected_auton->stop();
	}
	Autons selection = auto_chooser.GetSelected();
	switch (selection)
	{
		case TWO_BALL:
		{
			selected_auton = std::make_unique<TwoBallAuton>(robot_context);	
		}break;
		case FOUR_BALL:
		{
			selected_auton = std::make_unique<FourBallAuton>(robot_context);
		}break;
	}
	
	selected_auton->start();
}

void Robot::AutonomousPeriodic()
{
	frc::SmartDashboard::PutNumber("Robot Angle", robot_context->navx.GetYaw() - robot_context->robot_angle_offset + robot_context->robot_start_angle);
}

void Robot::TeleopInit()
{
	if(selected_auton != nullptr)
	{
		selected_auton->stop();
		selected_auton.release();
	}
	copilot_mode = SHOOTER;
	changeStates(PeriodicSubsystem::TELEOP);
}

void Robot::TeleopPeriodic()
{
	DriveSubsystem* drive = dynamic_cast<DriveSubsystem*>(robot_context->sub_systems["Drive"].get());
	IntakeSubsystem* intake = dynamic_cast<IntakeSubsystem*>(robot_context->sub_systems["Intake"].get());
	ShooterSubsystem* shooter = dynamic_cast<ShooterSubsystem*>(robot_context->sub_systems["Shooter"].get());
	ClimberSubsystem* climber = dynamic_cast<ClimberSubsystem*>(robot_context->sub_systems["Climber"].get());
	frc::SmartDashboard::PutNumber("Robot Angle", robot_context->navx.GetYaw() - robot_context->robot_angle_offset);
	//Get input 
	double forward_axis = deadZone(robot_context->pilot_controller.GetLeftY(), DEFAULT_DEADZONE);
	double turn_axis = deadZone(robot_context->pilot_controller.GetRightX(), DEFAULT_DEADZONE);
	slow_motion_debounce.setValue(robot_context->pilot_controller.GetLeftBumper() >= 0.5);
	turn_axis *= TURNING_FACTOR;
	// if(robot_context->copilot_controller.GetStartButtonPressed())
	// {
	// 	robot_context->robot_angle_offset = robot_context->navx.GetYaw();
	// }
	//Drive Subsytem
	if(slow_motion_debounce.getValue())
	{
		drive->setDrive(forward_axis*SLOW_MOTION_FACTOR,turn_axis*SLOW_MOTION_FACTOR);
	}
	else 
	{
		drive->setDrive(forward_axis,turn_axis);
	}
	
	//Intake Subsystem
	intake_debounce.setValue(robot_context->pilot_controller.GetRightBumper());
	outtake_debounce.setValue(robot_context->pilot_controller.GetRightTriggerAxis() >= 0.5);
	bool pilot_controlling_feeder{false}; 
	double pilot_intake_power = 0.0;
	if(intake_debounce.getValue())
	{
		frc::SmartDashboard::PutBoolean("Intake", true);
		pilot_intake_power = 1.0;
		if(intake->hasBall())
		{
			intake->setFeeder(0.25);
			pilot_controlling_feeder = true;
		}
	}
	else if(outtake_debounce.getValue())
	{
		frc::SmartDashboard::PutBoolean("Intake", true);
		pilot_intake_power = -1.0;
		if(intake->hasBall())
		{
			intake->setFeeder(-0.285); //.25
			pilot_controlling_feeder = true;
		}
	}
	else 
	{
		frc::SmartDashboard::PutBoolean("Intake", false);
	}
	toggle_intake_debounce.setValue(robot_context->pilot_controller.GetLeftTriggerAxis() > 0.5);
	bool toggle_intake_value = toggle_intake_debounce.getValue();
	intake->setIntakeDeployed(toggle_intake_value);
	// if(toggle_intake_value && !last_toggle_intake_value)
	// {
	// 	intake->toggleIntakeDeployed();

	// }

	//last_toggle_intake_value = toggle_intake_value;

	change_copilot_mode_debounce.setValue(robot_context->copilot_controller.GetBackButton() && robot_context->copilot_controller.GetStartButton());
	bool change_copilot_mode_value = change_copilot_mode_debounce.getValue();
	if(change_copilot_mode_value && !last_change_copilot_mode_value)
	{
		if(copilot_mode == SHOOTER)
		{
			copilot_mode = CLIMBER;
		}
		else if(copilot_mode == CLIMBER)
		{
			copilot_mode = SHOOTER;
		}
	}
	last_change_copilot_mode_value = change_copilot_mode_value;
	frc::SmartDashboard::PutString("Copilot Mode", copilotModeToString(copilot_mode));
	climb_up_debounce.setValue(robot_context->pilot_controller.GetYButton());
	climb_down_debounce.setValue(robot_context->pilot_controller.GetAButton());
	double copilot_intake_power = 0.0;
	if(climb_up_debounce.getValue())
	{
		climber->setClimberPower(1.0);
	}
	else if(climb_down_debounce.getValue())
	{
		climber->setClimberPower(-1.0);
	}
	else
	{
		climber->setClimberPower(0.0);
	}
	if(copilot_mode == SHOOTER)
	{
		//Shooter Subsystem
		doShooterControls();
		//Turret
		doTurretControls();
		climber->setClimberPower(0.0);

		//feeder
		if(!pilot_controlling_feeder)
		{
			feeder_up_debounce.setValue(robot_context->copilot_controller.GetRightTriggerAxis() >= 0.5);
			feeder_down_debounce.setValue(robot_context->copilot_controller.GetRightBumper());
			if(feeder_up_debounce.getValue())
			{
				frc::SmartDashboard::PutBoolean("Feeder", true);
				intake->setFeeder(0.25);
				copilot_intake_power = 1.0;
			}
			else if(feeder_down_debounce.getValue())
			{
				frc::SmartDashboard::PutBoolean("Feeder", true);
				intake->setFeeder(0.-30); //.25
				copilot_intake_power = -1.0;
			}
			else 
			{
				frc::SmartDashboard::PutBoolean("Feeder", false);
				intake->setFeeder(0.0);
			}
		}
	}
	else if(copilot_mode == CLIMBER)
	{
		if(!pilot_controlling_feeder)
		{
			intake->setFeeder(0.0);
		}
		shooter->setTurretMode(ShooterSubsystem::SET);
		shooter->setTurretPosition(-90.0);
		shooter->setShootSpeed(0.0);
		if(robot_context->copilot_controller.GetLeftBumper())
		{
			climber->setOverride(true);
			climber->setLeftPower(robot_context->copilot_controller.GetLeftY());
			climber->setRightPower(robot_context->copilot_controller.GetRightY());

		}
		else
		{
			climber->setOverride(true);
			deploy_traversal_debounce.setValue(robot_context->copilot_controller.GetRightBumper());
			retract_traversal_debounce.setValue(robot_context->copilot_controller.GetRightTriggerAxis() >0.5);

			if(deploy_traversal_debounce.getValue())
			{
				climber->setTraversalDeployed(true);
			}
			else if(retract_traversal_debounce.getValue())
			{
				climber->setTraversalDeployed(false);
			}
		}
	
	}
	if(pilot_intake_power != 0.0)
	{
		intake->setIntake(pilot_intake_power);
	}
	else if(copilot_intake_power != 0.0)
	{
		intake->setIntake(copilot_intake_power);
	}
	else
	{
		intake->setIntake(0.0);
	}
	
	
}

void Robot::DisabledInit()
{
	if(selected_auton != nullptr)
	{
		selected_auton->stop();
		selected_auton.release();
	} //Stop whatever script we are running, if other states are started
	current_speed = 0;
	changeStates(PeriodicSubsystem::DISABLED);
}

void Robot::DisabledPeriodic()
{
	ShooterSubsystem* shooter = dynamic_cast<ShooterSubsystem*>(robot_context->sub_systems["Shooter"].get());
	if(robot_context->copilot_controller.GetLeftStickButton() && robot_context->copilot_controller.GetRightStickButton())
	{
		shooter->setTurretOffset();
	}
	frc::SmartDashboard::PutNumber("Robot Angle", robot_context->navx.GetYaw() - robot_context->robot_angle_offset + robot_context->robot_angle_offset);
	// if(robot_context->copilot_controller.GetStartButtonPressed())
	// {
	// 	robot_context->robot_angle_offset = robot_context->navx.GetYaw();
	// }
}

void Robot::TestPeriodic()
{

}

void Robot::changeStates(PeriodicSubsystem::RobotState state)
{
	for (const auto &entry: robot_context->sub_systems)
	{
		auto system = entry.second;
		system->changeState(state);
	}
}

void Robot::doTurretControls()
{
	ShooterSubsystem* shooter = dynamic_cast<ShooterSubsystem*>(robot_context->sub_systems["Shooter"].get());
	auto_aim_debounce.setValue(robot_context->copilot_controller.GetAButton());
	face_turret_foward_debounce.setValue(robot_context->copilot_controller.GetPOV() == 0);

	bool auto_aim_value = auto_aim_debounce.getValue();
	bool face_turret_foward_value = face_turret_foward_debounce.getValue();
	double set_stick_x = robot_context->copilot_controller.GetLeftX();
	double set_stick_y = robot_context->copilot_controller.GetLeftY();
	double set_magnitude = std::sqrt(set_stick_x*set_stick_x + set_stick_y*set_stick_y);
	double manual_stick_x = robot_context->copilot_controller.GetRightX();

	if(auto_aim_value)
	{
		shooter->setTurretMode(ShooterSubsystem::AUTO);
	}
	else if(set_magnitude > 0.5)
	{
		shooter->setTurretMode(ShooterSubsystem::SET);
	 	double unit_x = set_stick_x/set_magnitude;
		double unit_y = set_stick_y/set_magnitude;
		double stick_angle = std::atan2(-unit_y, -unit_x);
		double stick_angle_degrees = stick_angle*180/M_PI;
		stick_angle_degrees -= 90.0;
		if(stick_angle_degrees < -180.0)
		{
			stick_angle_degrees += 360.0;
		}
		shooter->setTurretPosition(stick_angle_degrees);
	}
	else if(face_turret_foward_value)
	{
		shooter->setTurretMode(ShooterSubsystem::SET);
		shooter->setTurretPosition(0.0);
	}
	else
	{
		shooter->setTurretMode(ShooterSubsystem::MANUAL);
		shooter->setTurretSpeed(deadZone(manual_stick_x, 0.05));
		
	}

}

void Robot::doShooterControls()
{
	ShooterSubsystem* shooter = dynamic_cast<ShooterSubsystem*>(robot_context->sub_systems["Shooter"].get());
	toggle_keep_shooter_running_debounce.setValue(robot_context->copilot_controller.GetAButton());
	bool toggle_keep_shooter_running_value = toggle_keep_shooter_running_debounce.getValue();
	if(toggle_keep_shooter_running_value && last_toggle_keep_shooter_running_value)
	{
		shooter->toggleKeepShooterRunning();
	}

	low_shooter_speed_debounce.setValue(robot_context->copilot_controller.GetLeftTriggerAxis() >= 0.5);
	high_shooter_speed_debounce.setValue(robot_context->copilot_controller.GetLeftBumper());
	ludacris_shooter_speed_debounce.setValue(robot_context->copilot_controller.GetXButton());
	bool low_shooter_speed_value = low_shooter_speed_debounce.getValue();
	bool high_shooter_speed_value = high_shooter_speed_debounce.getValue();
	bool ludacris_shooter_speed_value = ludacris_shooter_speed_debounce.getValue();
	if(low_shooter_speed_value)
	{
		shooter->setShootSpeed(LOW_SHOOTER_SPEED);
		shooter->setBackspinSpeed(LOW_BACKSPIN_SPEED);
	}
	else if(high_shooter_speed_value)
	{
		shooter->setShootSpeed(HIGH_SHOOTER_SPEED);
		shooter->setBackspinSpeed(HIGH_BACKSPIN_SPEED);
	}
	else if(ludacris_shooter_speed_value)
	{
		shooter->setShootSpeed(LUDACRIS_SHOOTER_SPEED + frc::SmartDashboard::GetNumber("Ludacris Speed Tuner", 0.0));
		shooter->setBackspinSpeed(LUDRACRIS_BACKSPIN_SPEED);
	}
	else
	{
		shooter->setShootSpeed(0.0);
		shooter->setBackspinSpeed(0.0);
	}
	frc::SmartDashboard::PutBoolean("Shooter At Speed", shooter->shooterAtSpeed());
	frc::SmartDashboard::PutBoolean("Auto Aim", shooter->aimingAtTarget());
}

std::string Robot::copilotModeToString(Robot::CopilotMode mode)
{
	switch(mode)
	{
		case SHOOTER:
		{
			return "Shooter";
		}
		case CLIMBER:
		{
			return "Climber";
		}
		default:
		{
			return "Unknown";
		}
	}
}

/*
Dynamic cast -> Set a periodic system to a drive system (reserve of what we usually do)
Is this a drive subsystem?, if not it wil crash

Try and catch: Use to catch error during runtime
*/