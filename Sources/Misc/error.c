#include "error.h"
#include "common.h"
#include <stdio.h>
#include "motors.h"
#include "timers.h"

extern struct motorData motData;

void err_Throw(char* errMsg)
{
    // Safety comes first
    asm sei;
    motData.mode = MOT_MANUAL;

	motData.speed[0] = 0;
	motData.speed[1] = 0;
	motData.speed[2] = 0;
	motData.speed[3] = 0;
	
	mot_KillOtherTimers();
	rti_Kill();

	asm cli;

	puts(errMsg);

	while (1){
	    
	    motData.mode = MOT_MANUAL;
		motData.speed[0] = 0;
		motData.speed[1] = 0;
		motData.speed[2] = 0;
		motData.speed[3] = 0;
		asm cli;
	}

	return;
}
