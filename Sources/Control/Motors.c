#include "timers.h"
#include "error.h"
#include "common.h"
#include "motors.h"
#include <stdio.h>
#include "quad_control.h"

#define MOTOR_SLAVE1_OC 6
#define MOTOR_SLAVE2_OC 5
#define MOTOR_SLAVE3_OC 4
#define MOTOR_MASTER_OC 7


#define _M1_ MOTOR_SLAVE2_OC
#define _M2_ MOTOR_SLAVE1_OC
#define _M3_ MOTOR_MASTER_OC
#define _M4_ MOTOR_SLAVE3_OC


#define MOTORS_MASK ( (1<<MOTOR_MASTER_OC) | (1<<MOTOR_SLAVE1_OC) | (1<<MOTOR_SLAVE2_OC) | (1<<MOTOR_SLAVE3_OC))
#define MOTORS_PORT PTT
#define MOTORS_DDR DDRT


#define MOT_PERIOD_MS 20
#define MOT_DUTY_MIN_MS 1
#define MOT_DUTY_MAX_MS 2

#define MOT_OVF_NS ((u32)TIM_OVERFLOW_TICKS*TIM_TICK_NS)
#define MOT_CONSTANT_TERM_TICKS TIM_US_TO_TICKS(1000*MOT_DUTY_MIN_MS)
#define MOT_SLOPE_TICKS (TIM_US_TO_TICKS(1000*MOT_DUTY_MAX_MS)-MOT_CONSTANT_TERM_TICKS)

#define MOTOR_MASTER_PIN GLUE(PTT_PTT,MOTOR_MASTER_OC)

#define MOT_LINK_MASK ((1 << MOTOR_SLAVE1_OC) | (1 << MOTOR_SLAVE2_OC) | \
			(1 << MOTOR_SLAVE3_OC) | (1 << MOTOR_MASTER_OC))

#define MOT_FRAC_TO_ESC_HNS(_frac) (10000 + (DIV_CEIL(((u32)_frac)*10000,32767)))
#define MOT_FRAC_TO_TIM_TICKS(_frac) TIM_HNS_TO_TICKS(MOT_FRAC_TO_ESC_HNS(_frac))


void mot_SlaveErr(void);
void mot_MasterSrv(void);

extern struct motorData control(void);
extern controlData_T controlData;

struct motorData motData = { MOT_MANUAL, {0,0,0,0} };
bool readyToCalculate = _FALSE;


// Macros to solve links in few assembly instructions.
#define mot_Link() tim7_LinkTimer(MOT_LINK_MASK, MOT_LINK_MASK)
#define mot_Unlink() tim7_UnlinkTimer(MOT_LINK_MASK)


void mot_Init(void)
{
	tim_id timerId[4];
	tim_Init();

	timerId[3] = tim_GetTimer(TIM_OC, mot_MasterSrv, NULL, MOTOR_MASTER_OC);
	timerId[1] = tim_GetTimer(TIM_OC, mot_SlaveErr, NULL, MOTOR_SLAVE1_OC);
	timerId[0] = tim_GetTimer(TIM_OC, mot_SlaveErr, NULL, MOTOR_SLAVE2_OC);
	timerId[2] = tim_GetTimer(TIM_OC, mot_SlaveErr, NULL, MOTOR_SLAVE3_OC);

	if ((timerId[0] < 0) || (timerId[1] < 0) || (timerId[2] < 0) || (timerId[3] < 0))
		err_Throw("Timers for motors already in use.");

	tim_SetOutputLow(MOTOR_SLAVE1_OC);
	tim_SetOutputLow(MOTOR_SLAVE2_OC);
	tim_SetOutputLow(MOTOR_SLAVE3_OC);

	mot_Link();
	tim_SetOutputToggle(MOTOR_MASTER_OC);
	tim_ClearFlag (MOTOR_MASTER_OC);
	tim_EnableInterrupts(MOTOR_MASTER_OC);

	return;
}


void mot_MasterSrv(void)
{
	static u16 latchedTime;

	if (MOTOR_MASTER_PIN == PIN_HIGH)
	{
		latchedTime = tim_GetValue(MOTOR_MASTER_OC);

		if (motData.mode == MOT_AUTO)
		{
			control_mixer(controlData.thrust, controlData.torque, &motData);
		}

		tim_SetValue(_M1_, latchedTime + fmul(motData.speed[0], MOT_SLOPE_TICKS) + MOT_CONSTANT_TERM_TICKS);
		tim_SetValue(_M2_, latchedTime + fmul(motData.speed[1], MOT_SLOPE_TICKS) + MOT_CONSTANT_TERM_TICKS);
		tim_SetValue(_M3_, latchedTime + fmul(motData.speed[2], MOT_SLOPE_TICKS) + MOT_CONSTANT_TERM_TICKS);
		tim_SetValue(_M4_, latchedTime + fmul(motData.speed[3], MOT_SLOPE_TICKS) + MOT_CONSTANT_TERM_TICKS);

		mot_Unlink();

		readyToCalculate = _TRUE;
	}
	else
	{
		tim_SetValue(MOTOR_MASTER_OC, latchedTime + TIM_US_TO_TICKS(1000*MOT_PERIOD_MS));
		mot_Link();
	}

	return;
}


#define TIMER_QUANTITY 8
void mot_KillOtherTimers(void)
{
	int i;
	for (i = 0; i < TIMER_QUANTITY; i++)
	{
		if ((MOTORS_MASK & (1<<i)) == 0)
			tim_FreeTimer(i);
	}
	
	return;
}


void mot_SlaveErr(void)
{
	err_Throw("A slave motor has interrupted.");
	return;
}



