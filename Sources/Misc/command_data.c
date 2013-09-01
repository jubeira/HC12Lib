#include "command_data.h"

frac comm_ProcessElev(u8 elev)
{
	frac thrust;
	
	elev <<= 1;
	
	if (elev < 20)
		thrust = 0;
	else if ((elev >= 20) && (elev < 80))
		thrust = 5000 + (elev-20)*166;
	else if ((elev >= 80) && (elev < 200))
		thrust = 15000 + (elev-80)*50;
	else
		thrust = 21000 + (elev-200)*80;	// No debe pasar FRAC_1 = 32000...
	
	return thrust;
}