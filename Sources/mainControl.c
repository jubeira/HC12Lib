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
#include "command_data.h"

// Hardware settings
#define TRANSMITTER_INIT PTX
#define DMU_TIMER 1
#define C1_ID 0
#define C2_ID 1
#define SHIFT_ID 2
#define BATT_ID 3


// Thrust-ramp settings
#define THRUST_STEP 200
#define THRUST_LIMIT 5500
#define THRUST_INC_PERIOD_MS 300
#define THRUST_INIT 200


// Setpoint
struct{

	quat attitude;
	frac thrust;

}setpoint = {UNIT_Q, 0};

// Control start
u8 start = _FALSE;

// Extern data.
extern struct motorData motData;
extern controlData_T controlData;

// Function definitions
void Init (void);
void att_process(void);
void rti_MotDelay(void *data, rti_time period, rti_id id);
void rti_ThrustRamp(void *data, rti_time period, rti_id id);
void main_HandleOutputs(void);

// Functions

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

bool have_to_output = 0;

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


bool remote_data_arrived = _FALSE;
u8 remote_char = '\0';
commandData_T receiveData;

void nrf_CheckPayload(bool success, u8 *ackPayload, u8 length)
{
    if (length == sizeof(receiveData))
    {
        remote_data_arrived = _TRUE;
        receiveData = *(commandData_T*)ackPayload;
    }
    else if (length == sizeof(u8))
    {
		remote_data_arrived = _TRUE;
    	remote_char = *ackPayload;
    }
    	
}


/* Main para control */

#define Q_COMPONENTS(q) (q).r, (q).v.x, (q).v.y, (q).v.z
extern u8 int_Disable;

void main (void)
{
	u8 userInput;
	u8 measurementCount = 0;
	bool motDelayDone = _FALSE;

	u16 torqueCount = 0;

	Init ();

	tim_GetTimer(TIM_IC, sample_ready, NULL, DMU_TIMER);
	tim_SetRisingEdge(DMU_TIMER);
	tim_ClearFlag(DMU_TIMER);
	tim_EnableInterrupts(DMU_TIMER);


#ifdef MAIN_BATT
	batt_AddBatt (ATD0, 0, NULL, BATT_MV_TO_LEVEL(3600), BATT_MV_TO_LEVEL(4200), &(batts[0]));
	batt_AddBatt (ATD0, 1, NULL, BATT_MV_TO_LEVEL(3600), BATT_MV_TO_LEVEL(4200), &(batts[1]));

	batt_CallOnSample (batt_Callback);

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
			}

			measurementCount = 1;
			//att_apply_correction(calibrationOutput);
		}
	}

#elif (defined MAIN_CONTROL)

	mot_Init();

	rti_Register (rti_MotDelay, &motDelayDone, RTI_ONCE, RTI_MS_TO_TICKS(3000));

	while(!motDelayDone)
		;

	motData.speed[0] = S16_MAX;
	motData.speed[1] = S16_MAX;
	motData.speed[2] = S16_MAX;
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

	// vamos a auto si llega una p en setpoint comentando la línea siguiente y activando setp.
	#ifndef MAIN_SETPOINT
	rti_Register(rti_ThrustRamp, NULL, RTI_MS_TO_TICKS(THRUST_INC_PERIOD_MS), RTI_NOW);
	motData.mode = MOT_AUTO;
	#endif


#endif

#ifdef MAIN_SETPOINT

	while (1) {
		commandData_T receiveDataCopy;
		char input;
		//input = qs_getchar(0);

		#ifdef MAIN_OUTPUT

			main_HandleOutputs();

		#endif

		if (!remote_data_arrived)
			continue;

		asm sei
	#ifdef USING_FJOY
		receiveDataCopy = receiveData;
		input = receiveData.input;
	#else
		input = remote_char;
	#endif
		
		remote_data_arrived = _FALSE;
		asm cli
		
#define INCL_REAL 31651
#define INCL_IMAG 5997
		
		switch (input) {
		case 'a':
			{
			//quat aux = {32488, 3024, -3024, 0};
			//quat aux = {INCL_REAL, INCL_IMAG, -INCL_IMAG, 0};
			//setpoint.attitude = aux;
			break;
			}
		case 's':
			{
			//quat aux = UNIT_Q;
			//setpoint.attitude = aux;
			break;
			}
		case 'd':
			{
			//quat aux = {32488, -3024, 3024, 0};
			//quat aux = {INCL_REAL, -INCL_IMAG, INCL_IMAG, 0};
			//setpoint.attitude = aux;
			break;
			}
		case 'w':
			{
			//quat aux = {INCL_REAL, -INCL_IMAG, -INCL_IMAG, 0};
			//setpoint.attitude = aux;
			break;
			}
		case 'x':
			{
			//quat aux = {INCL_REAL, INCL_IMAG, INCL_IMAG, 0};
			//setpoint.attitude = aux;
			break;
			}
		case 'f':
			{
			//quat aux = {23170, 0, 0, 23170};
			//setpoint.attitude = aux;
			break;
			}
		case 'y':
			int_Disable = 1;
			break;
			
		case 'u':
			int_Disable = 0;
			break;
			
		case ' ':
		case 'q':
			motData.mode = MOT_MANUAL;

			motData.speed[0] = 0;
			motData.speed[1] = 0;
			motData.speed[2] = 0;
			motData.speed[3] = 0;
			break;

		case 'p':
			//rti_Register(rti_ThrustRamp, NULL, RTI_MS_TO_TICKS(THRUST_INC_PERIOD_MS), RTI_NOW);
			motData.mode = MOT_AUTO;
			setpoint.thrust = 3500;
			break;
			
		case '.':
			if (setpoint.thrust < 25900)
				setpoint.thrust += 100;
			else
				setpoint.thrust = 26000;
			break;
			
		case ',':
			if (setpoint.thrust > 100)
				setpoint.thrust -= 100;
			else
				setpoint.thrust = 0;
			break;
		
		case 'l':
			if (setpoint.thrust < 25750)
				setpoint.thrust += 250;
			else
				setpoint.thrust = 26000;
			break;
		
		case 'k':
			if (setpoint.thrust > 250)
				setpoint.thrust -= 250;
			else
				setpoint.thrust = 0;
			break;
			
		default:
			if (receiveDataCopy.input >= '0' && receiveDataCopy.input <= '9') {
				int new_thrust = (receiveDataCopy.input - '0')*1500 + 6000;
				setpoint.thrust = new_thrust;
			}
			break;
		}
		
	#ifdef USING_FJOY
	// Joystick code here
		{
		u8 elev = receiveDataCopy.elev;
		setpoint.thrust = comm_ProcessElev(elev);
		}
		
	#endif
	}

#elif (defined MAIN_OUTPUT) && !(defined MAIN_SETPOINT)

	while(1) {
		main_HandleOutputs();
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
	nrf_Init(TRANSMITTER_INIT);

	// Habilitar en caso de querer recibir setpoints, poner callback.
//	nrf_Receive(nrf_copychar);

	// Modules that do require interrupts to be enabled

#ifdef MAIN_BATT
	batt_Init();
#endif

	puts("Init done");

	return;
}

#ifdef TRANSMIT_VEC3
vec3 transmitData = {0,0,0};
#elif (defined TRANSMIT_EVEC3)
evec3 transmitData = {0,0,0};
#elif (defined TRANSMIT_QUAT)
quat transmitData = {0,{0,0,0}};
#elif (defined TRANSMIT_SPAM)
char transmitData = 'a';
#endif

void main_HandleOutputs(void)
{
	if (have_to_output) {
		quat QEstAux;

		asm sei;
		have_to_output = 0;
		QEstAux = controlData.QEst;
		asm cli;

		//printf("%d %d %d %d,", Q_COMPONENTS(QEstAux));
		//printf("%d %d %d %d,", Q_COMPONENTS(setpoint.attitude));
		//printf("thrust: %d", controlData.thrust);
		#ifdef TRANSMIT_QUAT
		nrf_Transmit((u8*)(&QEstAux), sizeof(QEstAux), nrf_CheckPayload);
		#elif ((defined TRANSMIT_EVEC3) || (defined TRANSMIT_VEC3) || (defined TRANSMIT_SPAM))
		nrf_Transmit((u8*)(&transmitData), sizeof(transmitData), nrf_CheckPayload);      
        #endif
	}
}


void rti_MotDelay(void *data, rti_time period, rti_id id)
{
	*(bool*)data = _TRUE;
	return;
}

u8 batts[2];

void batt_Callback(void)
{
	nrf_StoreAckPayload(batts,2);
	return;
}
