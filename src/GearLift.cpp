/*
 * GearLift.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: Developer
 */

#include <GearLift.h>

GearLift::GearLift()
{
}

GearLift::~GearLift()
{
}

void GearLift::Stop()
{
	m_liftMotor.Set(0.0);
}

void GearLift::Raise()
{
	m_liftMotor.Set(RAISE_SPEED);
}

void GearLift::Lower()
{
	m_liftMotor.Set(-LOWER_SPEED);
}

void GearLift::Clamp()
{
	m_clampSolinoid.Set(DoubleSolenoid::Value::kForward);
}

void GearLift::Release()
{
	m_clampSolinoid.Set(DoubleSolenoid::Value::kReverse);
}

bool GearLift::IsStalled()
{

	return false;
}
