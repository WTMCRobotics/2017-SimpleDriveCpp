#include <iostream>
#include <memory>
#include <string>
#include <cmath>

#include <Timer.h>

#include <IterativeRobot.h>
#include <DriverStation.h>
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


bool g_bPracticeRobot = false;
const double maxSpeed = 2000;

class Robot: public frc::IterativeRobot
{

private:
	frc::PowerDistributionPanel	m_PDP {0};
	frc::XboxController	m_controller{0};
	frc::ADXRS450_Gyro 	m_gyro{frc::SPI::kOnboardCS0};

	DriverStation::Alliance m_allianceColor;
	int m_allianceLocation;


	CANTalonDriveTrain 	m_driveTrain {&m_controller, &m_gyro};
	Winch 				m_winchMotor {&m_PDP};
	GearLift			m_gearLift   {};
	DigitalInput 		m_diPracticeRobot {DIO_PRACTICE_ROBOT};

	double 	m_gyroAngle;
	double 	m_leftJoystickY;
	double 	m_rightJoystickY;
	double 	m_leftTrigger;
	double 	m_rightTrigger;
	bool   	m_bLeftBumper;
	bool   	m_bRightBumper;
	bool	m_bButtonA;
	bool	m_bButtonB;
	bool	m_bButtonX;
	bool	m_bButtonY;

	frc::LiveWindow* lw = LiveWindow::GetInstance();

	std::shared_ptr<NetworkTable> axisCameraTable;


	// autonomous states
	//
	typedef enum
	{
		autoStart,
		autoTraverse,
		autoDropGear,
		autoDone
	} eAutonomousState;


	// traverse states
	//
	typedef enum
	{
		traverseNext,
		traverseTurn,
		traverseMove,
		traverseDone
	} eTraverseState;


	eAutonomousState m_autoState = autoStart;
	eTraverseState m_traverseState = traverseDone;
	int m_traverseIndex = 0;



public:

	void RobotInit()
	{
		m_allianceColor    = DriverStation::GetInstance().GetAlliance();
		m_allianceLocation = DriverStation::GetInstance().GetLocation();
		std::cout << "Alliance Color   : " << m_allianceColor << std::endl;
		std::cout << "Alliance Location: " << m_allianceLocation << std::endl;

		// Jumper is installed on Practice Robot, which pulls DI low
		g_bPracticeRobot = !m_diPracticeRobot.Get();

		frc::CameraServer::GetInstance()->StartAutomaticCapture("Driving Camera", 0);

//.		axisCameraTable = NetworkTable::GetTable("GRIP/contoursReport");
//.		double gripNumbers[1];
//.		axisCameraTable->GetNumberArray("contoursReport", gripNumbers);
//.		frc::SmartDashboard::PutNumber("Center X: ", gripNumbers[0]);

	}


	//=================================================================================
	// Autonomous Initialize
	//
	//	Starting Location key:
	//			+-----------------+
	//		 	| 1				1 |
	//	Blue 	| x 			x | Red
	//	Alliance| 2				2 | Alliance
	//		 	| 3				3 |
	//			+-----------------+
	//
	void AutonomousInit() override
	{
		m_traverseIndex = 0;

		// clear all traverse segments
		//
		for (int i=0; i<AUTO_MOVE_MAX_SEGMENTS; i++)
		{
			m_distance[i] = 0.0;
			m_speed[i] = 0.0;
			m_angle[i] = 0.0;
		}

		// The autonomous traverse path depends on the starting location
		//
		switch (m_allianceLocation)
		{
		case 1:
			m_angle[0] 		= kStart1Angle0;
			m_distance[0] 	= kStart1Leg0;
			m_speed[0]		= kStart1Speed0;
			m_angle[1] 		= kStart1Angle1;
			m_distance[1] 	= kStart1Leg1;
			m_speed[1]		= kStart1Speed1;
			break;
		case 2:
			m_angle[0] 		= kStart2Angle0;
			m_distance[0] 	= kStart2Leg0;
			m_speed[0]		= kStart2Speed0;
			m_angle[1] 		= kStart2Angle1;
			m_distance[1] 	= kStart2Leg1;
			m_speed[1]		= kStart2Speed1;
			m_angle[2] 		= kStart2Angle2;
			m_distance[2] 	= kStart2Leg2;
			m_speed[2]		= kStart2Speed2;
			break;
		case 3:
			m_angle[0] 		= kStart3Angle0;
			m_distance[0] 	= kStart3Leg0;
			m_speed[0]		= kStart3Speed0;
			m_angle[1] 		= kStart3Angle1;
			m_distance[1] 	= kStart3Leg1;
			m_speed[2]		= kStart3Speed1;
			break;
		default:
			break;
		}

		// Autonomous traverse path was calculated from Red Alliance starting location.
		//	If the actual alliance is Red, the angles need to be flipped.
		//
		if (m_allianceColor == DriverStation::Alliance::kBlue)
		{
			for (int i=0; i<AUTO_MOVE_MAX_SEGMENTS; i++)
				m_angle[i] 	  = -m_angle[i];

		}
	}


	//=================================================================================
	// Autonomous Periodic
	//
	//	Step through Autonomous State Machine
	//
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

		frc::SmartDashboard::PutString("Alliance Color    : ", (m_allianceColor == DriverStation::Alliance::kRed) ? "Red" : "Blue");
		frc::SmartDashboard::PutNumber("Alliance Location : ", m_allianceLocation);
	}

	void InitTraverse(void)
	{
		m_angle[0] 		= 0.0;
		m_distance[0] 	= 2.00;
		m_speed[0]		= 150.00;
		m_angle[1] 		= 60.0;
		m_distance[1] 	= 1.00;
		m_speed[1]		= 150.00;
		m_speed[2] 		= 0;
		m_traverseState = traverseNext;
		m_traverseIndex = 0;
	}

	void UpdateControlData()
	{
		m_gyroAngle = m_gyro.GetAngle();
		m_leftJoystickY  = m_controller.GetY(frc::GenericHID::kLeftHand);
		m_rightJoystickY = m_controller.GetY(frc::GenericHID::kRightHand);

		m_leftTrigger = m_controller.GetTriggerAxis(frc::GenericHID::kLeftHand);
		m_rightTrigger = m_controller.GetTriggerAxis(frc::GenericHID::kRightHand);

		m_bLeftBumper = m_controller.GetBumper(frc::GenericHID::kLeftHand);
		m_bRightBumper = m_controller.GetBumper(frc::GenericHID::kRightHand);

		m_bButtonA = m_controller.GetAButton();
		m_bButtonB = m_controller.GetBButton();
		m_bButtonX = m_controller.GetXButton();
		m_bButtonY = m_controller.GetYButton();
	}

	void TeleopPeriodic()
	{

		if (m_controller.GetAButton() )
		{
			UpdateControlData();
			if (m_autoState == autoTraverse)
			{
				if (AutoTraverse())
					m_autoState = autoDone;
			}
			UpdateDashboard();
			return;
		}
		else
		{
			m_autoState = autoTraverse;
			InitTraverse();
//			m_driveTrain.Stop();
		}

		UpdateControlData();

		m_driveTrain.Update(m_leftJoystickY, m_rightJoystickY, m_bLeftBumper);
		m_gearLift.Update(m_bButtonX, m_bButtonY, m_bRightBumper);
		m_winchMotor.Update(m_leftTrigger);

		UpdateDashboard();
	}

	void TestPeriodic()
	{
		//lw->Run();


	}

	void UpdateDashboard(void)
	{
/*
		double centerX = 0;
		axisCameraTable->GetNumber("x", centerX);
		frc::SmartDashboard::PutNumber("Center X: ", centerX);
		axisCameraTable = NetworkTable::GetTable("GRIP/myBlobsReport");
 */

		frc::SmartDashboard::PutString("Robot : ", (g_bPracticeRobot) ? "Practice Robot" : "Competition Robot");

		frc::SmartDashboard::PutNumber("Left Joystick : ", round(m_leftJoystickY, 2));
		frc::SmartDashboard::PutNumber("Left Command  : ", round(m_driveTrain.GetLeftTarget(), 2));
		frc::SmartDashboard::PutNumber("Left Speed    : ", round(m_driveTrain.GetLeftSpeed(), 2));

		frc::SmartDashboard::PutNumber("Right Joystick: ", round(m_rightJoystickY, 2));
		frc::SmartDashboard::PutNumber("Right Command : ", round(m_driveTrain.GetRightTarget(), 2));
		frc::SmartDashboard::PutNumber("Right Speed   : ", round(m_driveTrain.GetRightSpeed(), 2));

		frc::SmartDashboard::PutNumber("GearLift Up   : ", m_gearLift.IsUp());
		frc::SmartDashboard::PutNumber("GearLift Down : ", m_gearLift.IsDown());
		frc::SmartDashboard::PutNumber("GearLift Clamp: ", m_gearLift.IsClamped());
		frc::SmartDashboard::PutNumber("Winch Trigger : ", round(m_leftTrigger, 2));

		frc::SmartDashboard::PutNumber("Auto State    : ", m_autoState);
		frc::SmartDashboard::PutNumber("TraverseIndex : ", m_traverseIndex);
		frc::SmartDashboard::PutNumber("TraverseState : ", m_traverseState);

		frc::SmartDashboard::PutNumber("Gyro Angle: ", round(m_gyroAngle, 2));

		frc::SmartDashboard::PutNumber("Start Position   : ", round(m_driveTrain.GetStartPosition(), 2));
		frc::SmartDashboard::PutNumber("End Position     : ", round(m_driveTrain.GetEndPosition(), 2));
		frc::SmartDashboard::PutNumber("Left Position : ", round(m_driveTrain.GetLeftPosition(), 2));
		frc::SmartDashboard::PutNumber("Right Position : ", round(m_driveTrain.GetRightPosition(), 2));
		frc::SmartDashboard::PutNumber("Delta Position   : ", round(m_driveTrain.GetDeltaPosition(), 2));

	}

	double round(double value, int numDecimals)
	{
		return trunc(value * pow(10, numDecimals)) / pow(10, numDecimals);
	}



};


START_ROBOT_CLASS(Robot)
