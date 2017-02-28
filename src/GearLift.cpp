/*
 * GearLift.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: Developer
 */

#include <GearLift.h>

GearLift::GearLift()
{
	m_bGearLiftDown		= false;
	m_bGearLiftUp		= false;
	m_bGearLiftClamped	= false;
	m_bGearLiftStalled 	= false;
}

GearLift::~GearLift()
{
}

void GearLift::Stop()
{
	m_liftMotor.Set(0.0);
}

void GearLift::Update(bool bLiftRaise, bool bLiftLower, bool bClampControl)
{
	// The gear lift switched are N/O switches that pull the input to ground when
	//	they are closed. The state of the actual switches are inverted, since otherwise
	//	the "pull-to-ground" wiring would result in negative logic.
	//
	//	For testing purposes without the actual gear lift mechanism, the Down switch is inverted from what is should be.

	// Switches are negative logic
	m_bGearLiftDown =  !m_diGearLiftDown.Get();
	m_bGearLiftUp   = !m_diGearLiftUp.Get();

	// gear lifting logic
	//
	if (IsStalled())
		Stop();
	else if (bLiftRaise && !m_bGearLiftUp)
		Raise();
	else if (bLiftLower && !m_bGearLiftDown)
		Lower();
	else
		Stop();

	// gear clamping logic
	//
	if (bClampControl)
	{
		Clamp();
		m_bGearLiftClamped = true;
	}
	else
	{
		Release();
		m_bGearLiftClamped = false;
	}
}

void GearLift::Raise()
{
	m_liftMotor.Set(-RAISE_SPEED);
}

void GearLift::Lower()
{
	m_liftMotor.Set(LOWER_SPEED);
}

void GearLift::Clamp()
{
	m_clampSolinoid.Set(DoubleSolenoid::Value::kForward);
}

void GearLift::Release()
{
	m_clampSolinoid.Set(DoubleSolenoid::Value::kReverse);
}

