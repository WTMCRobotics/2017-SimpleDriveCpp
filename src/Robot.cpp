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

#include "Drivetrain.h"
#include "CANTalonDrivetrain.h"

// Uncomment to define the motor controller being used
//
#define CANTalon
//#define VictorSP


double round(double value, int numDecimals);
bool speedSet = false;

class Robot: public frc::IterativeRobot
{

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameLeft = "Left Start";
	const std::string autoNameMiddle = "Middle Start";
	const std::string autoNameRight = "Right Start";
	std::string autoSelected;
	std::shared_ptr<NetworkTable> axisCameraTable;

#if defined(CANTalon)
	CANTalonDriveTrain drivetrain {};
#elif defined(VictorSP)
	Drivetrain drivetrain {};
#else
#error "Need to define drivetrain";
#endif


public:

	void RobotInit()
	{
		chooser.AddObject(autoNameLeft, autoNameLeft);
		chooser.AddObject(autoNameMiddle, autoNameMiddle);
		chooser.AddObject(autoNameRight, autoNameRight);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		frc::CameraServer::GetInstance()->StartAutomaticCapture("Driving Camera", 0);

		axisCameraTable = NetworkTable::GetTable("GRIP/contoursReport");
		double gripNumbers[1];
		axisCameraTable->GetNumberArray("contoursReport", gripNumbers);
		frc::SmartDashboard::PutNumber("Center X: ", gripNumbers[0]);
	}


	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override
	{
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
	}

	void AutonomousPeriodic()
	{
		Timer timer;
		double testVal;
		timer.Reset();
		timer.Start();
		if (autoSelected == autoNameLeft)
		{
			while (timer.Get() < 5)
			{
				testVal = 500;
				drivetrain.Update(testVal);
				frc::SmartDashboard::PutNumber("Timer: ", timer.Get());
			}
			timer.Reset();
			timer.Start();
			while (timer.Get() < 3)
			{
				testVal = 0;
				drivetrain.Update(testVal);
				frc::SmartDashboard::PutNumber("Timer: ", timer.Get());
			}
		}
		else if (autoSelected == autoNameMiddle)
		{
			while (timer.Get() < 7)
			{
				testVal = 500;
				drivetrain.Update(testVal);
				frc::SmartDashboard::PutNumber("Timer: ", timer.Get());
			}
			timer.Reset();
			timer.Start();
			while (timer.Get() < 2)
			{
				testVal = 0;
				drivetrain.Update(testVal);
				frc::SmartDashboard::PutNumber("Timer: ", timer.Get());
			}
		}
		else if (autoSelected == autoNameRight)
		{
			// Right Auto goes here
		}
		else
		{
			// Default Auto goes here
		}
	}

	void TeleopInit()
	{
		drivetrain.Stop();
	}

	void TeleopPeriodic()
	{
		double maxSpeed = 2000;

		drivetrain.Update(maxSpeed);

		axisCameraTable = NetworkTable::GetTable("GRIP/myBlobsReport");
		double centerX = 0;
		axisCameraTable->GetNumber("x", centerX);
		frc::SmartDashboard::PutNumber("Center X: ", centerX);

		frc::SmartDashboard::PutNumber("Joystick Left: ", round(drivetrain.GetControllerValue(frc::GenericHID::kLeftHand), 2));
		frc::SmartDashboard::PutNumber("Right Command: ", round(drivetrain.GetRightCommand(), 2));
		frc::SmartDashboard::PutNumber("Right Speed: ", round(drivetrain.GetRightSpeed(), 2));

		frc::SmartDashboard::PutNumber("Joystick Right: ", round(drivetrain.GetControllerValue(frc::GenericHID::kRightHand), 2));
		frc::SmartDashboard::PutNumber("Left Command: ", round(drivetrain.GetLeftCommand(), 2));
		frc::SmartDashboard::PutNumber("Left Speed: ", round(drivetrain.GetLeftSpeed(), 2));

		frc::SmartDashboard::PutNumber("Gyro Angle: ", round(drivetrain.GetGyroAngle(), 2));
	}

	void TestPeriodic()
	{
		lw->Run();
	}


};

double round(double value, int numDecimals)
{
	return trunc(value * pow(10, numDecimals)) / pow(10, numDecimals);
}

START_ROBOT_CLASS(Robot)
