#ifndef _MOTORS_H_INCLUDED_
#define _MOTORS_H_INCLUDED_

#include "arith.h"

#define MOTOR_SLAVE1_OC 6
#define MOTOR_SLAVE2_OC 5
#define MOTOR_SLAVE3_OC 4
#define MOTOR_MASTER_OC 7

#define MOTORS_MASK ( (1<<MOTOR_MASTER_OC) | (1<<MOTOR_SLAVE1_OC) | (1<<MOTOR_SLAVE2_OC) | (1<<MOTOR_SLAVE3_OC))
#define MOTORS_PORT PTT
#define MOTORS_DDR DDRT

typedef enum {MOT_MANUAL=0, MOT_AUTO}motorMode;

struct motorData{

	motorMode mode;
	frac speed[4];
};

void mot_Init(void);

#endif
