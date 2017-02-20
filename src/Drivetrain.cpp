/*
 * Drivetrain.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: Developer
 */

#include <Drivetrain.h>

Drivetrain::Drivetrain()
{
	gyro.Calibrate();
}

Drivetrain::~Drivetrain()
{
}

void Drivetrain::Stop()
{
	m_rightMotor1.Set(0.0);
	m_leftMotor1.Set(0.0);
	m_rightMotor2.Set(0.0);
	m_leftMotor2.Set(0.0);
}

void Drivetrain::Update(double leftValue, double rightValue)
{
	m_rightMotor1.Set(rightValue * m_speedFactor);
	m_leftMotor1.Set(-leftValue * m_speedFactor);

	m_rightMotor2.Set(rightValue * m_speedFactor);
	m_leftMotor2.Set(-leftValue * m_speedFactor);
}

double Drivetrain::GetControllerValue(frc::GenericHID::JoystickHand hand)
{
	return controller.GetY(hand);
}

double Drivetrain::GetGyroAngle()
{
	return gyro.GetAngle();
}
