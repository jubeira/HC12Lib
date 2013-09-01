#ifndef _COMMAND_DATA_H_INCLUDED_
#define _COMMAND_DATA_H_INCLUDED_

#include "common.h"
#include "arith.h"
#include "quad_control.h"

typedef struct 
{
	s8 yaw;
	s8 pitch;
	s8 roll;
	u8 elev;
	
	char input;
}commandData_T;

frac comm_ProcessElev(u8 elev);
// Applies split function to raw joystick data for a better control. Includes dead zone, and zones with high and 
// low slope.

setpoint_T comm_FjoyToSetpoint(const commandData_T* data);
// Converts joysitck data to proper setpoint.

// Note: In this case, no need of using sei and cli when copying data, because a security copy is being made in main,
// and this code is called by main, not by an interrupt.

#endif // _COMMAND_DATA_H_INCLUDED_