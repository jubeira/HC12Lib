#include "mc9s12xdp512.h"
#include "common.h"
#include "pll.h"
#include "quick_serial.h"
#include "lcd.h"
#include "usonic.h"

#include <stdio.h>

void Init (void);

void print_us(s32 x)
{
	memset(lcd_memory, ' ', LCD_MEMORY);
	sprintf(lcd_memory, "%ld", x);
	usonic_Measure(print_us);
}

void main (void)
{
	Init ();
	
	print_us(0);

	while (1) {
		asm nop;
	}
}

void Init (void)
{
	PLL_SPEED(BUS_CLOCK_MHZ);

	// Modules that don't require interrupts to be enabled
	qs_init(0, MON12X_BR);

	asm cli;

	// Modules that do require interrupts to be enabled
	
	lcd_Init(LCD_2004);
	usonic_Init();
	
	printf("Init Done\n");

	return;
}

