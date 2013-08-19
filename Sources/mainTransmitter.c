#include "mc9s12xdp512.h"
#include "common.h"
#include "pll.h"
#include "quick_serial.h"
#include "quad_rf.h"
#include "nRF24L01+.h"
#include "arith.h"

#include <stdio.h>

void Init (void);

void nrf_TXCallback(bool success, u8 *ackPayload, u8 length);
void nrf_RXCallback(u8 *data, u8 length);


void main (void)
{
	Init ();
/*
	qrf_SendJoyMeasurements();
	qrf_PrintCommInfo();
*/
	nrf_Receive (nrf_RXCallback);


	while (1) {
		char input;
//		input = qs_getchar(0);

		//nrf_Transmit (&input, 1, nrf_TXCallback);

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

void nrf_RXCallback(u8 *data, u8 length)
{
	printf("%d %d %d %d,", Q_COMPONENTS(*((quat*)data)));

}