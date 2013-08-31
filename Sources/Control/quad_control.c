/*
 * quad_control.c
 *
 */


#include <limits.h>
#include "quad_control.h"
#include <stdio.h>
#include "debug.h"

controlData_T controlData = {{0,0,0,0}, {0, 0, 0}, {0,0,0}, 0};

/*
********** Control de actitud *************
 */

/* Integral */
#define int_gain_divide 8000
#define integral_error_limit 250

/* Realimentación de posicion */
#define prop_gain_frac_xy 6500
#define prop_gain_frac_z 6500

const vec3 PosGain = {prop_gain_frac_xy, prop_gain_frac_xy, prop_gain_frac_z};

/* Realimentacion de velocidad */

//#define der_gain_int_xy (1)
//#define der_gain_int_z (1)
//const ivec3 VelGain = {der_gain_int_xy, der_gain_int_xy, der_gain_int_z};

#define der_gain_int_xy 256
#define der_gain_int_z 280
const vec3_q8_8 VelGain = {der_gain_int_xy, der_gain_int_xy, der_gain_int_z};

/* * */

#ifdef TRANSMIT_VEC3
extern vec3 transmitData;
#elif (defined TRANSMIT_EVEC3)
extern evec3 transmitData;
#elif (defined TRANSMIT_QUAT)
extern quat transmitData;
#endif

vec3 adv_att_control(quat setpoint, quat att, vec3 angle_rate)
{
	static quat att_prev = UNIT_Q;
	static vec3 error_sat_prev = VEC0;
	static evec3 integral_out_prev = VEC0;

	vec3 error = qerror(setpoint, att);
	vec3 torques = VEC0;
	evec3 ctrl_signal;

	vec3 error_sat = vsat(error, integral_error_limit);
	evec3 integral_out;
	
	transmitData = error;

	integral_out = dvsum(dvsum(v_to_extended(error_sat_prev), v_to_extended(error_sat)), integral_out_prev);

	integral_out_prev = integral_out;
	error_sat_prev = error_sat;

	integral_out.z = 0;

	ctrl_signal = 	dvsum(
				dvsub(
					v_to_extended(vfmul_e2e(error, PosGain)),
				 	//vimul2_e2e(angle_rate, VelGain)
				 	vmul8_8(angle_rate, VelGain)
				),
				dvdiv(integral_out, int_gain_divide)
			);

	att_prev = att;

	torques = evclip(ctrl_signal);

	// FIXME: esto esta mal, es para poder probar sin que moleste el yaw
	//torques.z = 0;
	return torques;
}


#define h_Kp_mul 1
#define h_Kd_mul 10

frac h_control(frac setpoint, frac h)
{
	static frac err_prev = 0;
	frac h_error = setpoint - h, thrust;

	thrust = h_error*h_Kp_mul + (h_error - err_prev)*h_Kd_mul;
	err_prev = h_error;

	return thrust;
}

#define mix_thrust_shift 0
#define	mix_roll_shift_r 0
#define mix_pitch_shift_r 0
#define mix_yaw_shift 1

frac gammainv(frac T, frac t1, frac t2, frac t3)
{
	dfrac r = 0;

	r += (((dfrac)T) << mix_thrust_shift);
	r += (((dfrac)t1) >> mix_roll_shift_r);
	r += (((dfrac)t2) >> mix_pitch_shift_r);
	r += (((dfrac)t3) << mix_yaw_shift);

	return fsqrt((r > 0)? ((r < FRAC_1)? r : FRAC_1) : 0);
	//return ((r > 0)? ((r < FRAC_1)? r : FRAC_1) : 0);
}

/* 0 y 2: brazo rojo; 1, 3: brazo negro
 * El hack es porque el motor 1 empuja menos, o el 3 empuja más.
 */
void control_mixer(frac thrust, vec3 torque, struct motorData* output)
{
	frac hack;

	output->speed[0] = gammainv(thrust, 0, torque.y, -torque.z);

	hack = gammainv(thrust, -torque.x, 0, torque.z);
	output->speed[1] = dtrunc(fexpand(hack) + fmul2(hack, 5000));

	output->speed[2] = gammainv(thrust, 0, -torque.y, -torque.z);
	output->speed[3] = gammainv(thrust, torque.x, 0, torque.z);

	return;
}
