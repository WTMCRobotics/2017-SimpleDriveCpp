/*
 * GearLift.h
 *
 *  Created on: Feb 18, 2017
 *      Author: Developer
 */

#ifndef GEAR_LIFT_H_
#define GEAR_LIFT_H_

#include <VictorSP.h>
#include <DoubleSolenoid.h>

#include "RobotDefs.h"

#define RAISE_SPEED  0.40
#define LOWER_SPEED	 0.40


class GearLift
{
private:
	frc::VictorSP 		m_liftMotor {1};

	frc::DoubleSolenoid	m_clampSolinoid {PCM_ID, PCM_CHANEL_GEAR_CLAMP, PCM_CHANEL_GEAR_RELEASE};

public:
	GearLift();
	virtual ~GearLift();

	void Stop(void);
	void Raise(void);
	void Lower(void);
	void Clamp(void);
	void Release(void);
	bool IsStalled(void);
};

#endif /* GEAR_LIFT_H_ */
