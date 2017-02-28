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



class CANTalonDriveTrain
{
private:
	const double MaxSpeed = 500;	// max speed in RPM of wheels

	// motor controllers
	CANTalon m_rightMasterDrive {CAN_ID_RIGHTMASTER};
	CANTalon m_rightSlaveDrive  {CAN_ID_RIGHTSLAVE};
	CANTalon m_leftSlaveDrive   {CAN_ID_LEFTSLAVE};
	CANTalon m_leftMasterDrive  {CAN_ID_LEFTMASTER};

	double m_leftTarget  = 0.0;
	double m_rightTarget = 0.0;

	double m_leftSpeed	  = 0.0;
	double m_rightSpeed   = 0.0;

	double m_speedFactor = .25;

	double m_startPosition 	 = 0.0;
	double m_endPosition	 = 0.0;
	double m_leftPosition = 0.0;
	double m_rightPosition = 0.0;
	double m_deltaPosition 	 = 0.0;

	// pointers to global objects
	frc::XboxController* m_pController;
	frc::ADXRS450_Gyro*  m_pGyro;


public:
	CANTalonDriveTrain(frc::XboxController* pController, frc::ADXRS450_Gyro* pGyro);
	virtual ~CANTalonDriveTrain();

	void Stop();
	void Update(double rightCommand, double leftCommand, bool slowSpeed);

	void AutoTurnStart(double currentAngle, double deltaAngle);
	bool AutoTurnUpdate(double currentAngle);
	void AutoMoveStart(double legLength, double velocity);
	bool AutoMoveUpdate(void);


	void SetSpeedFactor(double speedFactor) { m_speedFactor = fmax(0.0, fmin(m_speedFactor, 1.0)); }
	double GetSpeedFactor(void) { return m_speedFactor; }

	double GetLeftTarget(void)	{ return m_leftTarget;}
	double GetLeftSpeed(void)   { return m_leftSpeed;}

	double GetRightTarget(void) { return m_rightTarget;}
	double GetRightSpeed(void)  { return m_rightSpeed;}

	double GetStartPosition(void) { return m_startPosition;}
	double GetEndPosition(void)  { return m_endPosition;}
	double GetLeftPosition(void) { return m_leftPosition;}
	double GetRightPosition(void) { return m_rightPosition;}
	double GetDeltaPosition(void)  { return m_deltaPosition;}

private:
	double Deadband(double commandValue);

};

#endif /* CAN_TALON_DRIVETRAIN_H_ */
