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

    tim_DisconnectOutput(MOTOR_MASTER_OC);
    tim_DisconnectOutput(MOTOR_SLAVE1_OC);
    tim_DisconnectOutput(MOTOR_SLAVE2_OC);
    tim_DisconnectOutput(MOTOR_SLAVE3_OC);

    MOTORS_DDR |= MOTORS_MASK;
    MOTORS_PORT &= ~MOTORS_MASK;

	puts(errMsg);

	while (1)
		;

	return;
}
