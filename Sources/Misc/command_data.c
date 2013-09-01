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


setpoint_T comm_FjoyToSetpoint(const commandData_T* data)
{
	vec3 stick;
	frac throttle;
	efrac norm2;
	setpoint_T newSetpoint;

	stick.x = ((s16)data->roll) << 8;	// roll
	stick.y = ((s16)data->pitch) << 8;	// pitch
	stick.z = ((s16)data->yaw) << 8;	// yaw

	throttle = data->elev; // elev

	norm2 = f_to_extended(fmul(stick.x, stick.x)) + fmul(stick.y, stick.y) + fmul(stick.z, stick.z);

	if (norm2 > FRAC_1)
	{
		stick = evclip(vefdiv(stick, fsqrt(norm2)));	// Se está dividiendo por un número mayor a 1, stick tiene que dar menor a lo que era.
		newSetpoint.attitude.r = 0;
	}	
	else
		newSetpoint.attitude.r = fsqrt(FRAC_1 - norm2);

	newSetpoint.attitude.v = stick;
	newSetpoint.thrust = comm_ProcessElev(throttle);

	return newSetpoint;
}