#include <mc9s12xdp512.h>
#include "rti.h"

enum {BASE_2=0, BASE_10};
#define RTI_DIVIDER_BASE BASE_10

#define RTI_PRESCALER (0b00010111 | (RTI_DIVIDER_BASE << 7))	// 16 MHz; 16.000 divider --> RTI_FREQ = 1000

#define RTI_SETPRESCALER(presc) (RTICTL = presc)
#define RTI_ENABLE_INTERRUPTS() (CRGINT_RTIE = 1)
#define RTI_CLEAR_FLAG() (CRGFLG_RTIF = 1)
#define RTI_SET_CLOCK_SOURCE(src) (CLKSEL_PLLSEL = src)

#define RTI_MAX_FCNS 20

#define RTI_IS_VALID_ID(id) (((id >= 0) && (id < RTI_MAX_FCNS)) ? _TRUE : _FALSE)

struct rti_cb {
	rti_time period;
	rti_time count;
	rti_ptr callback;
	void *data;
};

bool rti_isInit = _FALSE;

struct rti_cb rti_tbl[RTI_MAX_FCNS];


void rti_Init()
{
	rti_id i;
	if (rti_isInit == _TRUE)
		return;
	
	rti_isInit = _TRUE;	
	
	for (i = 0; i < RTI_MAX_FCNS; i++)
		rti_tbl[i].callback = NULL;
		
	RTI_SETPRESCALER (RTI_PRESCALER);
	RTI_ENABLE_INTERRUPTS();
	RTI_CLEAR_FLAG();
	
	return;
}


rti_id rti_Register (rti_ptr callback, void *data, rti_time period, rti_time delay)
{
	rti_id i;
	
	for (i = 0; i < RTI_MAX_FCNS; i++) 
	{
		if (rti_tbl[i].callback == NULL) 
		{
			rti_tbl[i].callback = callback;
			rti_tbl[i].data = data;
			rti_tbl[i].period = period;
			rti_tbl[i].count = delay;
			break;
		}
	}
		
	if (i == RTI_MAX_FCNS)
		i = RTI_INVALID_ID;
	
	return i;
}


void rti_SetPeriod(rti_id id, rti_time period)
{
	if (!RTI_IS_VALID_ID(id))
		return;
	
	rti_tbl[id].period = period;
	
	return;
}


void rti_Cancel(rti_id id)
{
	if (!RTI_IS_VALID_ID(id))
		return;
	
	rti_tbl[id].callback = NULL;
}


void rti_Kill(void)
{
	int i;
	for (i = 0; i < RTI_MAX_FCNS; i++)
		rti_Cancel(i);
	
	return;
}


void interrupt rti_Service(void)
{
	rti_id i;
	for (i = 0; i < RTI_MAX_FCNS; i++) 
	{
		if (rti_tbl[i].callback != NULL) 
		{
			if ((--rti_tbl[i].count) == 0) 
			{	
				rti_tbl[i].callback(rti_tbl[i].data, rti_tbl[i].period, i);
				
				if (rti_tbl[i].count != 0) // If the callback deleted itself and registered another function in the same place, count wont be 0
					break;
				
				if (rti_tbl[i].period == RTI_ONCE)
					rti_tbl[i].callback = NULL;
				else
					rti_tbl[i].count = rti_tbl[i].period;
			}
		}
	}

	RTI_CLEAR_FLAG();
	
	return;
}