#include "mc9s12xdp512.h"
#include "common.h"
#include "pll.h"
#include "quick_serial.h"
#include "quad_rf.h"
#include "nRF24L01+.h"
#include "arith.h"
#include "fjoy.h"
#include "command_data.h"
#include "lcd.h"
#include "debug.h"

#include <stdio.h>

extern struct fjoy_status;

commandData_T transmitData;

void Init (void);

void nrf_TXCallback(bool success, u8 *ackPayload, u8 length);
void nrf_RXCallback(u8 *data, u8 length);
void fjoy_Callback(void);


void main (void)
{
	Init ();
/*
	qrf_SendJoyMeasurements();
	qrf_PrintCommInfo();
*/
//	nrf_Receive (nrf_RXCallback);

	while (1) {
	#ifndef USING_FJOY
	#warning "not using fjoy"
		char input = qs_getchar(0);

		nrf_StoreAckPayload (&input, sizeof(input));
		//nrf_Transmit (&input, 1, nrf_TXCallback);
	#endif
	}
}

void Init (void)
{
	PLL_SPEED(BUS_CLOCK_MHZ);

	// Modules that don't require interrupts to be enabled
	qs_init(0, MON12X_BR);

	asm cli;

	// Modules that do require interrupts to be enabled
	//qrf_Init();
	nrf_Init(PRX);

#ifdef USING_FJOY
#warning "Using fjoy"
	fjoy_Init();
	fjoy_CallOnUpdate(fjoy_Callback);

#endif

	lcd_Init(LCD_2004);
	printf("Init Done\n");

	return;
}

void nrf_TXCallback(bool success, u8 *ackPayload, u8 length)
{
	if (success)
		puts("ok\n");
	else
		puts(":(\n");
}

#define Q_COMPONENTS(q) (q).r, (q).v.x, (q).v.y, (q).v.z
#define VEC_COMPONENTS(v) (v).x, (v).y, (v).z

void nrf_RXCallback(u8 *data, u8 length)
{
    switch(length)
    {
        case sizeof(quat):
            printf("%d %d %d %d", Q_COMPONENTS(*((quat*)data)));
            break;
        case sizeof(evec3):
            printf("%ld %ld %ld", VEC_COMPONENTS(*((evec3*)data)));
            break;
    	case sizeof(vec3):
            printf("%d %d %d", VEC_COMPONENTS(*((evec3*)data)));
			break;
        default:
            break;
    }

    putchar(RECEPTOR_TERMINATOR);
    return;
}


void fjoy_Callback(void)
{
	bool interruptStatus;
	int input = qs_poll_rx(0);

	if (input == QS_NODATA)
		transmitData.input = '\0';
	else
		transmitData.input = (char)input;

	interruptStatus = SafeSei();

	transmitData.yaw = fjoy_status.yaw;
	transmitData.roll = fjoy_status.roll;
	transmitData.pitch = fjoy_status.pitch;
	transmitData.elev = fjoy_status.elev;

	SafeCli(interruptStatus);

	nrf_StoreAckPayload (&transmitData, sizeof(transmitData));

	memset(lcd_memory, ' ' , LCD_MEMORY/4);
	sprintf(lcd_memory, "%d", (int)comm_ProcessElev(transmitData.elev));
}
