/*
 * CANTalonDriveTrain.h
 *
 *  Created on: Feb 6, 2017
 *      Author: tomh
 asdfasdf
 */

#ifndef SRC_CANTALONDRIVETRAIN_H_
#define SRC_CANTALONDRIVETRAIN_H_

#include <CANTalon.h>
#include <XBoxController.h>
#include <GenericHID.h>
#include <ADXRS450_Gyro.h>
#include <SPI.h>

// uncomment only one of these mode options
//
//#define MODE_Voltage
#define MODE_Speed
//#define MODE_Position

const double PERCENT_SPEED = .2;

class CANTalonDriveTrain
{
private:
	CANTalon m_rightMasterDrive {1};
//	CANTalon m_rightSlaveDrive {3};
	CANTalon m_leftMasterDrive {2};
//	CANTalon m_leftSlaveDrive {4}
	frc::XboxController m_controller{0};
	frc::ADXRS450_Gyro m_gyro{frc::SPI::kOnboardCS0};

	double m_leftSpeed = 0.0;
	double m_rightSpeed = 0.0;
	double m_leftCommand = 0.0;
	double m_rightCommand = 0.0;


public:
	CANTalonDriveTrain();
	virtual ~CANTalonDriveTrain();

	void Stop();
	void Update(double rightSpeed, double leftSpeed);
	void Update(const double scaleFactor);

	double GetControllerValue(frc::GenericHID::JoystickHand hand);
	double GetGyroAngle(void);

	double GetLeftSpeed(void)    { return m_leftSpeed;}
	double GetRightSpeed(void)   { return m_rightSpeed;}

	double GetLeftCommand(void)  { return m_leftCommand;}
	double GetRightCommand(void) { return m_rightCommand;}

};

#endif /* SRC_CANTALONDRIVETRAIN_H_ */
