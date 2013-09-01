#include "command_data.h"

frac comm_ProcessElev(u8 elev)
{
	frac thrust;
	
	if (elev < 20)
		thrust = 0;
	else if ((elev > 20) && (elev < 100))
		thrust = 5000 + (elev-20)*125;
	else if ((elev > 100) && (elev < 200))
		thrust = 15000 + (elev-100)*50;
	else
		thrust = 25000 + (elev-200)*100;	// No debe pasar FRAC_1 = 32000...
}