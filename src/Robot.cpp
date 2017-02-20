#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include <Timer.h>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/Sendable.h>
#include <SmartDashboard/SmartDashboard.h>
#include <CameraServer.h>
#include <tables/ITable.h>

#include "RobotDefs.h"

#include <PowerDistributionPanel.h>
#include <ADXRS450_Gyro.h>

#include "CANTalonDrivetrain.h"
#include "Winch.h"
#include "GearLift.h"


// forward declarations
//
void ProcessDriveTrain(void);
void ProcessWinch(void);
void ProcessGearLift(void);
void UpdateDashboard(void);

double round(double value, int numDecimals);


const double maxSpeed = 2000;

class Robot: public frc::IterativeRobot
{

private:
	frc::PowerDistributionPanel	m_PDP {0};
	frc::XboxController	m_controller{0};
	frc::ADXRS450_Gyro 	m_gyro{frc::SPI::kOnboardCS0};
	CANTalonDriveTrain 	m_driveTrain {&m_controller, &m_gyro};
	Winch 				m_winchMotor {&m_PDP};
	GearLift			m_gearLift   {};

	double m_distance[AUTO_MOVE_MAX_SEGMENTS];
	double m_angle[AUTO_MOVE_MAX_SEGMENTS];
	double m_speed[AUTO_MOVE_MAX_SEGMENTS];

	double m_leftJoystickY;
	double m_rightJoystickY;
	double m_gyroAngle;

	frc::LiveWindow* lw = LiveWindow::GetInstance();

	frc::SendableChooser<std::string> startPositionSelector;
	const std::string StartPositionLeft	  = "Left Start";
	const std::string StartPositionCenter = "Middle Start";
	const std::string StartPositionRight  = "Right Start";
	std::string startPosition;
	std::shared_ptr<NetworkTable> axisCameraTable;

	typedef enum
	{
		autoStart,
		autoTraverse,
		autoDropGear,
		autoDone
	} eAutonomousState;
	eAutonomousState m_autoState = autoStart;

public:

	void RobotInit()
	{
		startPositionSelector.AddObject(StartPositionLeft,   StartPositionLeft);
		startPositionSelector.AddObject(StartPositionCenter, StartPositionCenter);
		startPositionSelector.AddObject(StartPositionRight,  StartPositionRight);
		frc::SmartDashboard::PutData("Auto Modes", &startPositionSelector);

		frc::CameraServer::GetInstance()->StartAutomaticCapture("Driving Camera", 0);

//.		axisCameraTable = NetworkTable::GetTable("GRIP/contoursReport");
//.		double gripNumbers[1];
//.		axisCameraTable->GetNumberArray("contoursReport", gripNumbers);
//.		frc::SmartDashboard::PutNumber("Center X: ", gripNumbers[0]);
	}


	//=================================================================================
	// Autonomous Initialize
	//
	void AutonomousInit() override
	{
		m_autoState = autoStart;

		startPosition = startPositionSelector.GetSelected();
		// std::string startPosition = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << startPosition << std::endl;

		for (int i=0; i<AUTO_MOVE_MAX_SEGMENTS; i++)
		{
			m_distance[i] = 0.0;
			m_speed[i] = 0.0;
			m_angle[i] = 0.0;
		}

		if (startPosition == StartPositionLeft)
		{
			m_angle[0] 		= kLeftAngle1;
			m_distance[0] 	= kLeftLeg1;
			m_speed[0]		= kLeftSpeed1;
			m_angle[1] 		= kLeftAngle1;
			m_distance[1] 	= kLeftLeg2;
			m_speed[1]		= kLeftSpeed2;
		}
		else if (startPosition == StartPositionCenter)
		{
			m_angle[0] 		= kMidAngle1;
			m_distance[0] 	= kMidLeg1;
			m_speed[0]		= kMidSpeed1;
			m_angle[1] 		= kMidAngle1;
			m_distance[1] 	= kMidLeg2;
			m_speed[1]		= kMidSpeed2;
			m_angle[2] 		= kMidAngle3;
			m_distance[2] 	= kMidLeg3;
			m_speed[2]		= kMidSpeed3;
		}
		else if (startPosition == StartPositionRight)
		{
			m_angle[0] 		= kRightAngle1;
			m_distance[0] 	= kRightLeg1;
			m_speed[0]		= kRightSpeed1;
			m_angle[1] 		= kRightAngle1;
			m_distance[1] 	= kRightLeg2;
			m_speed[2]		= kRightSpeed2;
		}
	}


	void AutonomousPeriodic()
	{
		switch (m_autoState)
		{
		case autoStart:
			break;
		case autoTraverse:
			break;
		case autoDropGear:
			break;
		case autoDone:
			break;
		}
	}

	void TeleopInit()
	{
		m_driveTrain.Stop();
	}

	void TeleopPeriodic()
	{
		m_leftJoystickY  = m_controller.GetY(frc::GenericHID::kLeftHand);
		m_rightJoystickY = m_controller.GetY(frc::GenericHID::kRightHand);
		m_gyroAngle = m_gyro.GetAngle();


		ProcessDriveTrain();
//		ProcessWinch();
//		ProcessGearLift();

		UpdateDashboard();

	}

	void TestPeriodic()
	{
		//lw->Run();

		if (m_winchMotor.IsStalled())
			m_winchMotor.Stop();
		else if (m_controller.GetBumper(frc::GenericHID::kLeftHand))
			m_winchMotor.Raise();
		else if (m_controller.GetBumper(frc::GenericHID::kRightHand))
			m_winchMotor.Lower();
		else
			m_winchMotor.Stop();

	}

	void ProcessDriveTrain(void)
	{

		m_driveTrain.Update(m_leftJoystickY, m_rightJoystickY);
	}

	void ProcessWinch(void)
	{
		if (m_winchMotor.IsStalled())
			m_winchMotor.Stop();
		else if (m_controller.GetBumper(frc::GenericHID::kLeftHand))
			m_winchMotor.Raise();
		else if (m_controller.GetBumper(frc::GenericHID::kRightHand))
			m_winchMotor.Lower();
		else
			m_winchMotor.Stop();
	}

	void ProcessGearLift(void)
	{
		double leftTrigger = m_controller.GetTriggerAxis(frc::GenericHID::kLeftHand);
		double rightTrigger = m_controller.GetTriggerAxis(frc::GenericHID::kRightHand);

		if (m_gearLift.IsStalled())
			m_gearLift.Stop();
		else if (leftTrigger > .2)
			m_gearLift.Raise();
		else
			m_gearLift.Lower();

		if (rightTrigger > .2)
			m_gearLift.Clamp();
		else
			m_gearLift.Release();
	}

	void UpdateDashboard(void)
	{
/*
		double centerX = 0;
		axisCameraTable->GetNumber("x", centerX);
		frc::SmartDashboard::PutNumber("Center X: ", centerX);
		axisCameraTable = NetworkTable::GetTable("GRIP/myBlobsReport");
 */

		frc::SmartDashboard::PutNumber("Left Joystick : ", round(m_leftJoystickY, 2));
		frc::SmartDashboard::PutNumber("Left Command  : ", round(m_driveTrain.GetLeftTarget(), 2));
		frc::SmartDashboard::PutNumber("Left Speed    : ", round(m_driveTrain.GetLeftSpeed(), 2));

		frc::SmartDashboard::PutNumber("Right Joystick: ", round(m_rightJoystickY, 2));
		frc::SmartDashboard::PutNumber("Right Command : ", round(m_driveTrain.GetRightTarget(), 2));
		frc::SmartDashboard::PutNumber("Right Speed   : ", round(m_driveTrain.GetRightSpeed(), 2));

		frc::SmartDashboard::PutNumber("Gyro Angle: ", round(m_gyroAngle, 2));
	}

	double round(double value, int numDecimals)
	{
		return trunc(value * pow(10, numDecimals)) / pow(10, numDecimals);
	}



};


START_ROBOT_CLASS(Robot)
