#include "mc9s12xdp512.h"
#include <stdio.h>
#include "common.h"
#include "dmu.h"
#include "dmu_macros.h"
#include "rti.h"
#include "timers.h"
#include "pll.h"
#include "quick_serial.h"
#include "usonic.h"
#include "error.h"
#include "quad_control.h"
#include "nlcf.h"
#include "arith.h"
#include "motors.h"
#include "debug.h"
#include "nRF24L01+.h"
#include "batt.h"
#include "lcd.h"
#include "quad_rf.h"


#define DMU_TIMER 1
#define C1_ID 0
#define C2_ID 1
#define SHIFT_ID 2
#define BATT_ID 3

#define THRUST_STEP 200
#define THRUST_LIMIT 10500
#define THRUST_INC_PERIOD_MS 300
#define THRUST_INIT 200

#define UP 0xFF
#define DOWN 0x00

extern struct dmu_data_T dmu_data;

void Init (void);
void PrintMeas (s32 measurement);
void GetMeasurementsMask(void *data, rti_time period, rti_id id);
void rti_MotDelay(void *data, rti_time period, rti_id id);
void GetSamplesMask(void *data, rti_time period, rti_id id);
void dataReady_Srv(void);
void dataReady_Ovf(void);
void fifoOvf_Srv(void);
void icFcn(void);
void rti_ThrustRamp(void *data, rti_time period, rti_id id);
void nrf_Callback (u8 *data, u8 length);

void Init (void);


struct tim_channelData dmu_timerData = {0,0};

u16 overflowCnt = 0;
u16 lastEdge = 0;

extern void att_process(void);
extern bool have_to_output;

extern bool readyToCalculate;
extern struct motorData motData;
extern controlData_T controlData;

bool have_to_output = 0;

struct{
	
	quat attitude;
	frac thrust;

}setpoint = {UNIT_Q, 0};



void att_process(void)
{
	static int ccount = 0;

	{
		att_estim(dmu_measurements.gyro, dmu_measurements.accel,
							&controlData.QEst, &controlData.bff_angle_rate);

		// El control se corre sólo después de inicializar los motores, para que el control integral
		// no empiece acumulando error.
		if (motData.mode == MOT_AUTO)
		{
			controlData.torque = adv_att_control(setpoint.attitude, controlData.QEst, controlData.bff_angle_rate);
			controlData.thrust = setpoint.thrust;
		}

		#ifdef MAIN_OUTPUT

		if (++ccount == 20) {
			ccount = 0;
			have_to_output = 1;
		}

		#endif

	}
}

void sample_ready(void)
{
	if (tim_GetEdge(DMU_TIMER) == EDGE_RISING)
	{
		tim_SetFallingEdge(DMU_TIMER);
		dmu_GetMeasurements(att_process);

	} else {
		tim_SetRisingEdge(DMU_TIMER);
	}
}

/* Main para control */

#define Q_COMPONENTS(q) (q).r, (q).v.x, (q).v.y, (q).v.z
extern vec3 Bias;


#define OC_PERIOD ((u8)62500)
#define TIM4_DUTY 14000
#define TIM5_DUTY 5000
#define TIM6_DUTY 10000
#define TIM7_DUTY 9375

u8 start = _FALSE;

u8 batts[2];

void batt_Callback(void)
{
	nrf_StoreAckPayload(batts,2);
	return;
}

void main (void)
{
	u8 userInput;
	u8 measurementCount = 0;
	bool motDelayDone = _FALSE;

	u16 torqueCount = 0;

	Init ();

	nrf_Receive(nrf_Callback);
	tim_GetTimer(TIM_IC, sample_ready, dataReady_Ovf, DMU_TIMER);
	tim_SetRisingEdge(DMU_TIMER);
	tim_ClearFlag(DMU_TIMER);
	tim_EnableInterrupts(DMU_TIMER);


#ifdef MAIN_BATT
	batt_AddBatt (ATD0, 0, batt_Callback, BATT_MV_TO_LEVEL(3600), BATT_MV_TO_LEVEL(4200), &(batts[0]));
	batt_AddBatt (ATD0, 1, batt_Callback, BATT_MV_TO_LEVEL(3600), BATT_MV_TO_LEVEL(4200), &(batts[1]));
#endif

#ifdef MAIN_CALIBRATE

	puts("Press 'm' to calibrate\n");

	while (measurementCount < 2)
	{
		struct cal_output calibrationOutput;
		struct qpair calibration;

		userInput = qs_getchar(0);

		if (userInput == 'c')
		{
			quat aux;
			asm sei;
			aux = controlData.QEst;
			asm cli;
			printf("Current quaternion: %d %d %d %d\n", Q_COMPONENTS(aux));
			continue;
		}

		if (userInput != 'm')
			continue;

		if (measurementCount == 0)
		{	asm sei;
			calibration.p0 = controlData.QEst;
			asm cli;
			puts("First measurement done\n");
		}
		else if (measurementCount == 1)
		{
			asm sei;
			calibration.p1 = controlData.QEst;
			asm cli;
			puts("Second measurement done\n");
		}
		measurementCount++;

		if (measurementCount == 2)
		{
			calibrationOutput = att_calibrate(calibration.p0, calibration.p1);
			printf("Cal output: %d\n", calibrationOutput.quality);
			printf("Correction: %d %d %d %d\n", Q_COMPONENTS(calibrationOutput.correction));

			if (calibrationOutput.quality == CAL_BAD)
			{
				measurementCount = 1;	// Stay looping second measurement.
				puts("Calibrate again\n");
			}j

			measurementCount = 1;
			//att_apply_correction(calibrationOutput);
		}
	}

#elif (defined MAIN_CONTROL)

	mot_Init();

	rti_Register (rti_MotDelay, &motDelayDone, RTI_ONCE, RTI_MS_TO_TICKS(3000));

	while(!motDelayDone)
		;

//	motData.speed[0] = S16_MAX;
	motData.speed[1] = S16_MAX;
//	motData.speed[2] = S16_MAX;
	motData.speed[3] = S16_MAX;

	motDelayDone = _FALSE;
	rti_Register (rti_MotDelay, &motDelayDone, RTI_ONCE, RTI_MS_TO_TICKS(2000));

	while(!motDelayDone)
		;

	motData.speed[0] = 0;
	motData.speed[1] = 0;
	motData.speed[2] = 0;
	motData.speed[3] = 0;


	motDelayDone = _FALSE;
	rti_Register (rti_MotDelay, &motDelayDone, RTI_ONCE, RTI_MS_TO_TICKS(3000));

	while(!motDelayDone)
		;

	 rti_Register(rti_ThrustRamp, NULL, RTI_MS_TO_TICKS(THRUST_INC_PERIOD_MS), RTI_NOW);

//	while (start == _FALSE)
//		;


	motData.mode = MOT_AUTO;

#endif

#ifdef MAIN_SETPOINT

	while (1) {
		char input;
		input = qs_getchar(0);

		if (input == 'a')
		{
			//quat aux = {32488, 3024, -3024, 0};
			quat aux = {32488, -4277, 0, 0};

			setpoint = aux;
		}
		else if (input == 's')
		{
			quat aux = UNIT_Q;
			setpoint = aux;
		}
		else if (input == 'd')
		{
			//quat aux = {32488, -3024, 3024, 0};
			quat aux = {32488, 4277, 0, 0};
			setpoint = aux;
		}
		else if (input == 'q')
		{
			motData.mode = MOT_MANUAL;

			motData.speed[0] = 0;
			motData.speed[1] = 0;
			motData.speed[2] = 0;
			motData.speed[3] = 0;

			while (1)
				;
		}
	}

#elif (defined MAIN_OUTPUT)

	while(1) {
		if (have_to_output) {
			quat QEstAux;

			asm sei;
			have_to_output = 0;
			QEstAux = controlData.QEst;
			asm cli;

			//printf("%d %d %d %d,", Q_COMPONENTS(QEstAux));
			printf("%d %d %d %d,", Q_COMPONENTS(setpoint.attitude));
			//printf("thrust: %d", controlData.thrust);

		}
	}

#else 
	while(1)
		;

#endif

}


void rti_ThrustRamp(void *data, rti_time period, rti_id id)
{
	if (setpoint.thrust == 0)
		setpoint.thrust = THRUST_INIT;

	if ((setpoint.thrust + THRUST_STEP) < THRUST_LIMIT)
		setpoint.thrust += THRUST_STEP;
	else
	{
		setpoint.thrust = THRUST_LIMIT;
		rti_Cancel(id);
	}

	return;
}

void measure (s32 measurement);

/*
// MAIN de testeo para DMU.
 void main (void)
 {
	int a;
	char vel;

	PLL_SPEED(BUS_CLOCK_MHZ);

	Init ();

//	DDRA = 0x01;



	tim_GetTimer(TIM_IC, dataReady_Srv, NULL, DMU_TIMER);
	tim_EnableInterrupts(DMU_TIMER);
	tim_SetRisingEdge(DMU_TIMER);


//	mot_Init();


//	tim_GetTimer(TIM_IC, fifoOvf_Srv, NULL, DMU_TIMER);

//	tim_EnableInterrupts(DMU_TIMER);
//	tim_SetRisingEdge(DMU_TIMER);

//	rti_Register(GetSamplesMask, NULL, RTI_MS_TO_TICKS(500), RTI_MS_TO_TICKS(

//	rti_Register(GetMeasurementsMask, NULL, RTI_MS_TO_TICKS(500), RTI_MS_TO_TICKS(500));
//	usonic_Measure(measure);
 	while (1)
 	{
 		vel = qs_getchar(0);
	 	if(vel == 'u')
	 	{
	 		motData.speed[0] += 300;
	 		motData.speed[1] += 300;
	 		motData.speed[2] += 300;
	 		motData.speed[3] += 300;
	 	}
	 	else if(vel == 'd')
	 	{
	 		motData.speed[0] -= 300;
	 		motData.speed[1] -= 300;
	 		motData.speed[2] -= 300;
	 		motData.speed[3] -= 300;
	 	}

 	}

}
*/
/*
void measure (s32 measurement)
{
	if (measurement != USONIC_INVALID_MEAS)
		printf("Distance: %ld cm.\n",measurement);
	else
		printf("Invalid measurement.\n");

	 usonic_Measure(measure);
}
*/
void nrf_Callback (u8 *data, u8 length)
{
	vec3 stick;
	frac throttle;
	efrac norm2;
	quat newSetpoint;

	switch (length)
	{
	case 4:
		stick.x = (s8)data[2];	// roll

		// Fixme
		stick.y = 0;//(s8)data[1];	// pitch
		stick.z = 0;//(s8)data[0];	// yaw

		stick.x <<= 8;
//		stick.y <<= 8;
//		stick.z <<= 8;


		throttle = data[3]; // elev
		throttle *= 60;
			
		norm2 = f_to_extended(fmul(stick.x, stick.x)) + fmul(stick.y, stick.y) + fmul(stick.z, stick.z);
		
		if (norm2 > FRAC_1)
		{
			stick = evclip(vefdiv(stick, fsqrt(norm2)));	// Se está dividiendo por un número mayor a 1, stick tiene que dar menor a lo que era.
			newSetpoint.r = 0;
		}	
		else
			newSetpoint.r = fsqrt(FRAC_1 - norm2);
		
		newSetpoint.v = stick;
		
		setpoint.attitude = newSetpoint;
		setpoint.thrust = throttle;

		setpoint.attitude.v = stick;
			
		break;
		
	default:
		break;
	}
}

void Init (void)
{
	PLL_SPEED(BUS_CLOCK_MHZ);

 	// Modules that don't require interrupts to be enabled
	qs_init(0, MON12X_BR);
	tim_Init();
	rti_Init();

 	asm cli;

 	// Modules that do require interrupts to be enabled
	dmu_Init();
	nrf_Init(PRX);
	// Modules that do require interrupts to be enabled

#ifdef MAIN_BATT
	batt_Init();
#endif

	puts("Init done");

	return;
}

void GetMeasurementsMask(void *data, rti_time period, rti_id id)
{
	dmu_GetMeasurements(dmu_PrintFormattedMeasurements_WO);
	return;
}

void GetSamplesMask(void *data, rti_time period, rti_id id)
{
	dmu_FifoAverage(NULL);
	return;
}

void rti_MotDelay(void *data, rti_time period, rti_id id)
{
	*(bool*)data = _TRUE;
	return;
}


void PrintMeas (s32 measurement)
{
	//printf("%ld\n", measurement);
}


void dataReady_Srv(void)
{
	static u16 count=0;

	if (tim_GetEdge(DMU_TIMER) == EDGE_RISING)
	{
		tim_SetFallingEdge(DMU_TIMER);
		if (++count == 100)
		{
			dmu_GetMeasurements(dmu_PrintFormattedMeasurements);
			count = 0;
		}
	}
	else
		tim_SetRisingEdge(DMU_TIMER);
}

void dataReady_Ovf(void)
{
//	dmu_timerData.overflowCnt++;
	overflowCnt++;

	return;
}

void fifoOvf_Srv(void)
{
//	printf("fifo ovf!!!\n");


	if (dmu_data.fifo.enable == _FALSE)
		return;

	dmu_data.fifo.enable = _FALSE;
	dmu_ReadFifo(NULL);

//	dmu_FifoReset(NULL);

	return;
}



void icFcn()
{
	static u16 count = 0;
	u32 time;

	time = tim_GetTimeElapsed(overflowCnt, 2, lastEdge);

	lastEdge = tim_GetValue(2);
	overflowCnt = 0;
 	count++;

 	if (count == 10)
 	{
 		printf("t: %lu\n", time*TIM_TICK_NS);
 		count = 0;
 	}
 	return;
 }
}
