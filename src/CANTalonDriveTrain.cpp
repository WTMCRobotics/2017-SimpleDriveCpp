/*
 * CANTalonDriveTrain.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: tomh
 */

#include <CANTalonDriveTrain.h>

CANTalonDriveTrain::CANTalonDriveTrain()
{
	m_gyro.Calibrate();

#if defined(MODE_Voltage)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPercentVbus);
	m_rightMasterDrive.Set(0);
#elif defined(MODE_Speed)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_rightMasterDrive.Set(0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(2048);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_rightMasterDrive.SetVoltageRampRate(5.5);
	m_rightMasterDrive.SetSensorDirection(true);
#elif defined(MODE_Position)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPosition);
	m_rightMasterDrive.Set(0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(2048);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
#else
#error No mode selected for TallonSRX
#endif

	m_rightSlaveDrive.SetControlMode(CANSpeedController::kFollower); //Speed);
	m_rightSlaveDrive.Set(1);

//	m_rightMasterDrive.SetPID(1, 0, 0);

}

CANTalonDriveTrain::~CANTalonDriveTrain()
{

}

void CANTalonDriveTrain::Stop(void)
{
	m_rightMasterDrive.Set(0);
}

void CANTalonDriveTrain::Update(double rightSpeed, double leftSpeed)
{
	m_rightMasterDrive.Set(rightSpeed);
	//m_rightSlaveDrive.Set(leftSpeed);

}

void CANTalonDriveTrain::Update(const double multiplier)
{
	double leftValue = m_controller.GetY(frc::GenericHID::kLeftHand);
	if (leftValue < 0.1 && leftValue > -0.1)
		m_rightMasterDrive.Set(0);
	else
		m_rightMasterDrive.Set(leftValue * multiplier);
	//double rightValue = m_controller.GetY(frc::GenericHID::kRightHand);
	//m_leftMasterDrive.Set(-leftValue * speedLimit);

}

double CANTalonDriveTrain::GetControllerValue(frc::GenericHID::JoystickHand hand)
{
	return m_controller.GetY(hand);
}

double CANTalonDriveTrain::GetGyroAngle()
{
	return m_gyro.GetAngle();
}
