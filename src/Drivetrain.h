/*
 * Drivetrain.h
 *
 *  Created on: Jan 23, 2017
 *      Author: Developer
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

#include <VictorSP.h>
#include <XBoxController.h>
#include <GenericHID.h>

class Drivetrain
{
private:
	frc::VictorSP m_rightMotor1 {1};
	frc::VictorSP m_leftMotor1  {0};
	frc::VictorSP m_rightMotor2 {3};
	frc::VictorSP m_leftMotor2  {2};
	frc::XboxController controller{0};

public:
	Drivetrain();
	virtual ~Drivetrain();

	void Stop(void);
	void Update();
	double GetControllerValue(frc::GenericHID::JoystickHand);
};

#endif /* SRC_DRIVETRAIN_H_ */
