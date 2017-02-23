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
#include <DigitalInput.h>

#include "CANTalonDrivetrain.h"
#include "Winch.h"
#include "GearLift.h"


const double maxSpeed = 2000;

class Robot: public frc::IterativeRobot
{

private:
	frc::PowerDistributionPanel	m_PDP {0};
	frc::XboxController	m_controller{0};
	frc::ADXRS450_Gyro 	m_gyro{frc::SPI::kOnboardCS0};
	DigitalInput m_diGearLiftDown {DIO_SWITCH_GEARLIFT_DOWN};
	DigitalInput m_diGearLiftUp   {DIO_SWITCH_GEARLIFT_UP};

	CANTalonDriveTrain 	m_driveTrain {&m_controller, &m_gyro};
	Winch 				m_winchMotor {&m_PDP};
	GearLift			m_gearLift   {};

	double m_leftJoystickY;
	double m_rightJoystickY;
	double m_gyroAngle;
	double m_leftTrigger;
	double m_rightTrigger;
	bool m_gearLiftIsDown;
	bool m_gearLiftIsUp;
	bool m_gearLiftIsClamped;

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

	typedef enum
	{
		traverseNext,
		traverseTurn,
		traverseMove,
		traverseDone
	} eTraverseState;

	eTraverseState m_traverseState = traverseDone;
	int m_traverseIndex = 0;



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
		m_traverseIndex = 0;

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
			m_angle[1] 		= kLeftAngle2;
			m_distance[1] 	= kLeftLeg2;
			m_speed[1]		= kLeftSpeed2;
		}
		else if (startPosition == StartPositionCenter)
		{
			m_angle[0] 		= kMidAngle1;
			m_distance[0] 	= kMidLeg1;
			m_speed[0]		= kMidSpeed1;
			m_angle[1] 		= kMidAngle2;
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
			m_angle[1] 		= kRightAngle2;
			m_distance[1] 	= kRightLeg2;
			m_speed[2]		= kRightSpeed2;
		}
	}


	void AutonomousPeriodic()
	{
		switch (m_autoState)
		{
		case autoStart:
			m_autoState = autoTraverse;
			break;
		case autoTraverse:
			if (AutoTraverse())
				m_autoState = autoDropGear;
			break;
		case autoDropGear:
			m_autoState = autoDone;
			break;
		case autoDone:
			break;
		}

		UpdateDashboard();
	}



	double m_distance[AUTO_MOVE_MAX_SEGMENTS];
	double m_angle[AUTO_MOVE_MAX_SEGMENTS];
	double m_speed[AUTO_MOVE_MAX_SEGMENTS];



	bool AutoTraverse(void)
	{
		if (m_traverseState == traverseNext)
		{
			if (m_traverseIndex >= AUTO_MOVE_MAX_SEGMENTS ||
				m_speed[m_traverseIndex] == 0)
			{
				m_traverseState = traverseDone;
				return true;
			}
			else
			{
				m_traverseState = traverseTurn;
				m_driveTrain.AutoTurnStart(m_angle[m_traverseIndex], m_gyroAngle);
			}
		}

		if (m_traverseState == traverseTurn)
		{
			if (m_driveTrain.AutoTurnUpdate(m_gyroAngle))
			{
				m_traverseState = traverseMove;
				m_driveTrain.AutoMoveStart(m_distance[m_traverseIndex], m_speed[m_traverseIndex]);
			}
		}

		if (m_traverseState == traverseMove)
		{
			if (m_driveTrain.AutoMoveUpdate())
			{
				m_traverseState = traverseNext;
				m_traverseIndex++;
			}
		}


		return false;
	}

	void TeleopInit()
	{
		m_driveTrain.Stop();
	}

	void InitTraverse(void)
	{
		m_angle[0] 		= kRightAngle1;
		m_distance[0] 	= kRightLeg1;
		m_speed[0]		= kRightSpeed1;
		m_angle[1] 		= kRightAngle2;
		m_distance[1] 	= kRightLeg2;
		m_speed[1]		= kRightSpeed2;
		m_traverseState = traverseNext;
		m_traverseIndex = 0;
	}


	void TeleopPeriodic()
	{
		m_leftJoystickY  = m_controller.GetY(frc::GenericHID::kLeftHand);
		m_rightJoystickY = m_controller.GetY(frc::GenericHID::kRightHand);
		m_gyroAngle = m_gyro.GetAngle();

		m_leftTrigger = m_controller.GetTriggerAxis(frc::GenericHID::kLeftHand);
		m_rightTrigger = m_controller.GetTriggerAxis(frc::GenericHID::kRightHand);


		if (m_controller.GetAButton() )
		{
			if (m_autoState == autoTraverse)
			{
				if (AutoTraverse())
					m_autoState = autoDone;
			}
		}
		else
		{
			m_autoState = autoTraverse;
			InitTraverse();
			m_driveTrain.Stop();
		}

		UpdateDashboard();

		return;


		// The gear lift switched are N/O switches that pull the input to ground when
		//	they are closed. The state of the actual switches are inverted, since otherwise
		//	the "pull-to-ground" wiring would result in negative logic.
		//
		//	For testing purposes without the actual gear lift mechanism, the Down switch is inverted from what is should be.
#warning "GearLift Down switch is inverted for testing"
		m_gearLiftIsDown = m_diGearLiftDown.Get(); //**********************************************************inverted for testing
		m_gearLiftIsUp   = !m_diGearLiftUp.Get();

		UpdateDriveTrain();
		UpdateWinch();
		UpdateGearLift();

		UpdateDashboard();
	}

	void TestPeriodic()
	{
		//lw->Run();


	}

	void UpdateDriveTrain(void)
	{
		m_driveTrain.Update(m_leftJoystickY, m_rightJoystickY);
	}

	void UpdateWinch(void)
	{
		if (m_winchMotor.IsStalled())
		{
			m_winchMotor.Stop();
			return;
		}

		if (m_controller.GetBumper(frc::GenericHID::kLeftHand))
			m_winchMotor.Raise(m_controller.GetBumper(frc::GenericHID::kRightHand));
		else
			m_winchMotor.Stop();
	}

	void UpdateGearLift(void)
	{

		// gear lifting logic
		//
		if (m_gearLift.IsStalled())
			m_gearLift.Stop();
		else if (m_leftTrigger > GEARLIFT_COMMAND_DEADBAND && !m_gearLiftIsUp)
			m_gearLift.Raise();
		else if (m_leftTrigger <= GEARLIFT_COMMAND_DEADBAND && !m_gearLiftIsDown)
			m_gearLift.Lower();
		else
			m_gearLift.Stop();

		// gear clamping logic
		//
		if (m_rightTrigger > GEARLIFT_COMMAND_DEADBAND)
		{
			m_gearLift.Clamp();
			m_gearLiftIsClamped = true;
		}
		else
		{
			m_gearLift.Release();
			m_gearLiftIsClamped = false;
		}
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

		frc::SmartDashboard::PutNumber("GearLift Up   : ", m_gearLiftIsUp);
		frc::SmartDashboard::PutNumber("GearLift Down : ", m_gearLiftIsDown);
		frc::SmartDashboard::PutNumber("GearLift Clamp: ", m_gearLiftIsClamped);

		frc::SmartDashboard::PutNumber("Auto State    : ", m_autoState);
		frc::SmartDashboard::PutNumber("TraverseIndex : ", m_traverseIndex);
		frc::SmartDashboard::PutNumber("TraverseState : ", m_traverseState);

		frc::SmartDashboard::PutNumber("Gyro Angle: ", round(m_gyroAngle, 2));

		frc::SmartDashboard::PutNumber("Start Position   : ", round(m_driveTrain.GetStartPosition(), 2));
		frc::SmartDashboard::PutNumber("End Position     : ", round(m_driveTrain.GetEndPosition(), 2));
		frc::SmartDashboard::PutNumber("Current Position : ", round(m_driveTrain.GetCurrentPosition(), 2));
		frc::SmartDashboard::PutNumber("Delta Position   : ", round(m_driveTrain.GetDeltaPosition(), 2));

	}

	double round(double value, int numDecimals)
	{
		return trunc(value * pow(10, numDecimals)) / pow(10, numDecimals);
	}



};


START_ROBOT_CLASS(Robot)
