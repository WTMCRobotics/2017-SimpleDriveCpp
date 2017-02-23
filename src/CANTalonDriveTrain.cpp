/*
 * CANTalonDriveTrain.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: tomh
 */

#include <CANTalonDriveTrain.h>

CANTalonDriveTrain::CANTalonDriveTrain(frc::XboxController* pController, frc::ADXRS450_Gyro*  pGyro)
{
	m_pController = pController;
	m_pGyro		  = pGyro;
	m_pGyro->Calibrate();

	m_leftSpeed 	= 0.0;
	m_rightSpeed 	= 0.0;
	m_speedFactor	= 1.0;

#if defined(MODE_Voltage)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPercentVbus);
	m_rightMasterDrive.Set(0);

#elif defined(MODE_Speed)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_rightMasterDrive.Set(0.0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(2048);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_rightMasterDrive.SetVoltageRampRate(DRIVE_VOLTAGE_RAMP_SEC);
	m_rightMasterDrive.SetSensorDirection(true);

	m_leftMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_leftMasterDrive.Set(0.0);
	m_leftMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_leftMasterDrive.ConfigEncoderCodesPerRev(2048);
	m_leftMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_leftMasterDrive.SetVoltageRampRate(DRIVE_VOLTAGE_RAMP_SEC);
	m_leftMasterDrive.SetSensorDirection(true);

#elif defined(MODE_Position)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPosition);
	m_rightMasterDrive.Set(0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(2048);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
#else
#error No mode selected for TallonSRX
#endif

//	m_rightSlaveDrive.SetControlMode(CANSpeedController::kFollower);
//	m_rightSlaveDrive.Set(CAN_ID_RIGHTMASTER);

//	m_leftSlaveDrive.SetControlMode(CANSpeedController::kFollower);
//	m_leftSlaveDrive.Set(CAN_ID_LEFTMASTER);

}

CANTalonDriveTrain::~CANTalonDriveTrain()
{

}

void CANTalonDriveTrain::Stop(void)
{
	m_leftSpeed = 0.0;
	m_leftMasterDrive.Set(-m_leftSpeed);
	m_rightSpeed = 0.0;
	m_rightMasterDrive.Set(m_rightSpeed);
}

void CANTalonDriveTrain::Update(double leftCommand, double rightCommand)
{
	m_leftTarget  = Deadband(leftCommand)  * MaxSpeed * m_speedFactor;
	m_rightTarget = Deadband(rightCommand) * MaxSpeed * m_speedFactor;

	m_leftMasterDrive.Set(-m_leftTarget);
	m_rightMasterDrive.Set(m_rightTarget);

	m_leftSpeed = m_leftMasterDrive.GetSpeed();
	m_rightSpeed = m_rightMasterDrive.GetSpeed();
}

void CANTalonDriveTrain::AutoTurnStart(double currentAngle, double deltaAngle)
{
	m_startPosition = currentAngle;
	m_endPosition = fmod((m_startPosition + deltaAngle), 360.0);

	double velocity = (deltaAngle < 0) ? -100 : 100;
	m_leftMasterDrive.Set(velocity);
	m_rightMasterDrive.Set(velocity);

	m_leftSpeed = m_leftMasterDrive.GetSpeed();
	m_rightSpeed = m_rightMasterDrive.GetSpeed();
}

bool CANTalonDriveTrain::AutoTurnUpdate(double currentAngle)
{
	m_currentPosition = currentAngle;
	m_deltaPosition = m_endPosition - m_currentPosition;

	m_leftSpeed = m_leftMasterDrive.GetSpeed();
	m_rightSpeed = m_rightMasterDrive.GetSpeed();

	if (m_deltaPosition > 0)
	{
		Stop();
		return true;
	}

	return false;
}

void CANTalonDriveTrain::AutoMoveStart(double legLength, double velocity)
{
	m_startPosition = m_leftMasterDrive.GetPosition();
	m_endPosition = m_startPosition - legLength;

	m_leftMasterDrive.Set(velocity);
	m_rightMasterDrive.Set(-velocity);

	m_leftSpeed = m_leftMasterDrive.GetSpeed();
	m_rightSpeed = m_rightMasterDrive.GetSpeed();
}

bool CANTalonDriveTrain::AutoMoveUpdate(void)
{
	m_currentPosition = m_leftMasterDrive.GetPosition();
	m_deltaPosition = m_endPosition - m_currentPosition;

	m_leftSpeed = m_leftMasterDrive.GetSpeed();
	m_rightSpeed = m_rightMasterDrive.GetSpeed();

	if (m_deltaPosition > 0)
	{
		Stop();
		return true;
	}

	return false;
}



double CANTalonDriveTrain::Deadband(double commandValue)
{
	//return commandValue;
	return (fabs(commandValue) >= DRIVE_COMMAND_DEADBAND) ? commandValue : 0.0;
}

