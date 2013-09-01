#ifndef _DEBUG_H_INCLUDED_
#define _DEBUG_H_INCLUDED_

// Main program

//#define MAIN_CALIBRATE
#define MAIN_CONTROL
#define MAIN_SETPOINT
#define MAIN_OUTPUT

//#define USING_FJOY
//#define FULL_FJOY
//#define THRUST_FJOY

#define TRANSMIT_VEC3
//#define TRANSMIT_EVEC3
//#define TRANSMIT_QUAT
//#define TRANSMIT_SPAM

#define RECEPTOR_TERMINATOR '\n'

//#define MAIN_BATT


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



#endif	//_DEBUG_H_INCLUDED_
