#ifndef _FJOY_DISP_H
#define _FJOY_DISP_H

// Prints the current axes readings (from fjoy) to a LCD display. This deletes all data written on the display.
// The LCD display must already be initialized, and be a LCD_2004 display (4 rows, 20 characters each).
void fjoy_PrintAxes (void);

#endif