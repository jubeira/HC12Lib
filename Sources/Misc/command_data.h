#ifndef _COMMAND_DATA_H_INCLUDED_
#define _COMMAND_DATA_H_INCLUDED_

#include "common.h"
#include "arith.h"

typedef struct 
{
	s8 yaw;
	s8 pitch;
	s8 roll;
	u8 elev;
	
	char input;
}commandData_T;

frac comm_ProcessElev(u8 elev);

#endif // _COMMAND_DATA_H_INCLUDED_