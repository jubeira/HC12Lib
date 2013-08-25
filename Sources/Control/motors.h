#ifndef _MOTORS_H_INCLUDED_
#define _MOTORS_H_INCLUDED_

#include "arith.h"


typedef enum {MOT_MANUAL=0, MOT_AUTO}motorMode;

struct motorData{

	motorMode mode;
	frac speed[4];
};

void mot_Init(void);
// Initializes motors, linking master and slaves.

void mot_KillOtherTimers(void);
// Kills timers not used for motors. Use in case of errors.

#endif
