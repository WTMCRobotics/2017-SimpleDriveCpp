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

	m_leftSpeed = 0.0;
	m_rightSpeed = 0.0;

#if defined(MODE_Voltage)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kPercentVbus);
	m_rightMasterDrive.Set(0);
#elif defined(MODE_Speed)
	m_rightMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_rightMasterDrive.Set(0.0);
	m_rightMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rightMasterDrive.ConfigEncoderCodesPerRev(2048);
	m_rightMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_rightMasterDrive.SetVoltageRampRate(7);
	m_rightMasterDrive.SetSensorDirection(true);

	m_leftMasterDrive.SetControlMode(CANSpeedController::kSpeed);
	m_leftMasterDrive.Set(0.0);
	m_leftMasterDrive.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_leftMasterDrive.ConfigEncoderCodesPerRev(2048);
	m_leftMasterDrive.ConfigPeakOutputVoltage(+12.0f, -12.0f);
	m_leftMasterDrive.SetVoltageRampRate(7);
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

	m_rightSlaveDrive.SetControlMode(CANSpeedController::kFollower); //Speed);
	m_rightSlaveDrive.Set(1);

	m_leftSlaveDrive.SetControlMode(CANSpeedController::kFollower);
	m_leftSlaveDrive.Set(2);
//	m_rightMasterDrive.SetPID(1, 0, 0);

}

CANTalonDriveTrain::~CANTalonDriveTrain()
{

}

void CANTalonDriveTrain::Stop(void)
{
	m_leftSpeed = 0.0;
	m_leftMasterDrive.Set(m_leftSpeed);
	m_rightSpeed = 0.0;
	m_rightMasterDrive.Set(m_rightSpeed);
}

void CANTalonDriveTrain::Update(double rightSpeed, double leftSpeed)
{
//	m_rightSpeed = rightSpeed;
//	m_leftSpeed = leftSpeed;

//	m_rightMasterDrive.Set(m_rightSpeed);
//	m_leftMasterDrive.Set(m_rightSpeed);
}

void CANTalonDriveTrain::Update(const double maxSpeed)
{
	m_leftCommand = m_controller.GetY(frc::GenericHID::kLeftHand);
	m_rightCommand = m_controller.GetY(frc::GenericHID::kRightHand);

	if (m_leftCommand <= 0.2 && m_leftCommand >= -0.2)
		m_leftSpeed = 0.0;
	else
		m_leftSpeed = m_leftCommand * maxSpeed;

	if (m_rightCommand <= 0.2 && m_rightCommand >= -0.2)
		m_rightSpeed = 0.0;
	else
		m_rightSpeed = -(m_rightCommand * maxSpeed);

	m_leftMasterDrive.Set(m_leftSpeed);
	m_rightMasterDrive.Set(m_rightSpeed);
}

double CANTalonDriveTrain::GetControllerValue(frc::GenericHID::JoystickHand hand)
{
	return m_controller.GetY(hand);
}

double CANTalonDriveTrain::GetGyroAngle()
{
	return m_gyro.GetAngle();
}
