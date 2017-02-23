#ifndef ROBOT_DEFS_H_
#define ROBOT_DEFS_H_

// Power Distribution Panel
//
// #define PDP_CHANEL_RIGHTMASTER	0
// #define PDP_CHANEL_RIGHTSLAVE	2
// #define PDP_CHANEL_LEFTMASTER	1
// #define PDP_CHANEL_LEFTSLAVE		3
// #define PDP_CHANEL_WINCH			15
// #define PDP_CHANEL_GEARLIFT		14
#define PDP_CHANEL_RIGHTMASTER	0
#define PDP_CHANEL_RIGHTSLAVE	1
#define PDP_CHANEL_LEFTSLAVE	2
#define PDP_CHANEL_LEFTMASTER	3
#define PDP_CHANEL_WINCH		15
#define PDP_CHANEL_GEARLIFT		14

// CAN Bus IDs
//
#define CAN_ID_RIGHTMASTER	4
#define CAN_ID_RIGHTSLAVE	3
#define CAN_ID_LEFTSLAVE	2
#define CAN_ID_LEFTMASTER	1

// Motor Parameters
//
#define STALL_CURRENT_WINCH 		4.0
#define STALL_CURRENT_GEAR_LIFT		4.0

// ToDo: test deadband value
#define DRIVE_COMMAND_DEADBAND		0.2
#define DRIVE_COMMAND_DEADBAND		0.2
#define DRIVE_VOLTAGE_RAMP_SEC		12.0
#define GEARLIFT_COMMAND_DEADBAND	0.2

// Digital Inputs
//
#define	DIO_SWITCH_GEARLIFT_DOWN	0
#define	DIO_SWITCH_GEARLIFT_UP		1

// Pneumatic Control Module
//
#define PCM_ID					0
#define PCM_CHANEL_GEAR_CLAMP 	0
#define PCM_CHANEL_GEAR_RELEASE 1

// Autonomous mode constants
//
#define AUTO_MOVE_MAX_SEGMENTS	3

// left starting position
#define kLeftAngle1		+0.0
#define kLeftLeg1		  4.75
#define kLeftSpeed1		  250.0
#define kLeftAngle2		60.0
#define kLeftLeg2		  50.0
#define kLeftSpeed2		  250.0

// right starting position
#define kRightAngle1	+0.0
#define kRightLeg1		  4.75
#define kRightSpeed1	  200.0
#define kRightAngle2	-60.0
#define kRightLeg2		  2.8
#define kRightSpeed2	  200.0

// center starting position
#define kMidAngle1		+0.0
#define kMidLeg1		  1000.0
#define kMidSpeed1		  200.0
#define kMidAngle2		+45.0
#define kMidLeg2		  50.0
#define kMidSpeed2		  200.0
#define kMidAngle3		-45.0
#define kMidLeg3		  50.0
#define kMidSpeed3		  200.0


#endif	/* ROBOT_DEFS_H_ */
