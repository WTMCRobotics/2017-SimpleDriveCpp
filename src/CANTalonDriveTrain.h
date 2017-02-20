/*
 * CANTalonDriveTrain.h
 *
 *  Created on: Feb 6, 2017
 *      Author: tomh
 asdfasdf
 */

#ifndef CAN_TALON_DRIVETRAIN_H_
#define CAN_TALON_DRIVETRAIN_H_

#include <GenericHID.h>
#include <SPI.h>

#include <CANTalon.h>
#include <XBoxController.h>
#include <ADXRS450_Gyro.h>

#include "RobotDefs.h"

// uncomment only one of these mode options
//
//#define MODE_Voltage
#define MODE_Speed
//#define MODE_Position

const double PERCENT_SPEED = .2;

class CANTalonDriveTrain
{
private:
	// motor controllers
	CANTalon m_rightMasterDrive {CAN_ID_RIGHTMASTER};
	CANTalon m_leftMasterDrive  {CAN_ID_LEFTMASTER};
	CANTalon m_rightSlaveDrive  {CAN_ID_RIGHTSLAVE};
	CANTalon m_leftSlaveDrive   {CAN_ID_LEFTSLAVE};

	double m_leftSpeed	  = 0.0;
	double m_rightSpeed   = 0.0;
	double m_leftCommand  = 0.0;
	double m_rightCommand = 0.0;

	double m_speedFactor = 1.0;

	// pointers to global objects
	frc::XboxController* m_pController;
	frc::ADXRS450_Gyro*  m_pGyro;


public:
	CANTalonDriveTrain(frc::XboxController* pController, frc::ADXRS450_Gyro* pGyro);
	virtual ~CANTalonDriveTrain();

	void Stop();
	void Update(double rightCommand, double leftCommand);

	void SetSpeedFactor(double speedFactor) { m_speedFactor = fmax(0.0, fmin(m_speedFactor, 1.0)); }
	double GetSpeedFactor(void) { return m_speedFactor; }

	double GetLeftSpeed(void)    { return m_leftSpeed;}
	double GetRightSpeed(void)   { return m_rightSpeed;}

private:
	double CANTalonDriveTrain::SetDeadband(double commandValue);

};

#endif /* CAN_TALON_DRIVETRAIN_H_ */
