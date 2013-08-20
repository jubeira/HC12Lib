#ifndef _DEBUG_H_INCLUDED_
#define _DEBUG_H_INCLUDED_

// Main program

//#define MAIN_CALIBRATE
//#define MAIN_CONTROL
//#define MAIN_SETPOINT
#define MAIN_OUTPUT
//#define MAIN_SENDMSG

//#define MAIN_BATT


//#define SEND_ATT_IN_PAYLOAD

// General keys

//#define IIC_DEBUG
//#define DMU_DEBUG


// Particular keys for IIC
#ifdef IIC_DEBUG

#define IIC_DEBUG_EOT

#endif	// IIC_DEBUG

#ifdef DMU_DEBUG

//#define FIFO_DEBUG_COUNT
#define DMU_DEBUG_OFFSET
//#define FIFO_DEBUG_PRINT_AVG_SAMPLES
//#define DMU_DEBUG_PRINT_ACCUMULATION

#endif	// FIFO_DEBUG

/* Debug messages */

#ifdef DEBUG
#define PUTS_WARNING(s) puts(s)
#else
#define PUTS_WARNING(s)
#endif


#endif	//_DEBUG_H_INCLUDED_