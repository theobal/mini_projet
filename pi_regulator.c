#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <selector.h>
#include <sensors/VL53L0X/VL53L0X.h>

//simple P regulator implementation
int16_t p_regulator_angle(float line_position, float goal){

	float error = 0;
	float speed_correction = 0;

	//static float sum_error = 0;

	error = line_position - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ROTATION_THRESHOLD){
		return 0;
	}

	//compute the speed correction
	speed_correction = KP1 * error;

	return (int16_t)speed_correction;
}

//simple PI regulator implementation
int16_t pi_regulator_distance(float obsticle_distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	//if the obsticle is too far, keep the cruise speed
	if(obsticle_distance > MAX_OBSTICLE_DISTANCE){
		return CRUISE_SPEED;
	}else{

		// compute the error
		error = obsticle_distance - goal;

		//disables the PI regulator if the error is to small
		//this avoids to always move as we cannot exactly be where we want and
		//the TOF sensor is a bit noisy
		if(fabs(error) < ERROR_THRESHOLD){
			return 0;
		}

		sum_error += error;

		//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
		if(sum_error > MAX_SUM_ERROR){
			sum_error = MAX_SUM_ERROR;
		}else if(sum_error < -MAX_SUM_ERROR){
			sum_error = -MAX_SUM_ERROR;
		}

		//compute the speed
		speed = KP2 * error + KI2 * sum_error;

		//saturate the speed at the cruise speed to allow a smooth transition
		if(speed > CRUISE_SPEED){
			speed = CRUISE_SPEED;
		}else if(speed < -CRUISE_SPEED){
			speed = -CRUISE_SPEED;
		}
	}
	return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	int16_t speed = 0;
	int16_t speed_correction = 0;

	while(1){
		time = chVTGetSystemTime();

		// stop the rotation correction if the obsticle is too close, as the camera will be obstructed
		if(VL53L0X_get_dist_mm()*MMTOCM < MIN_OBSTICLE_DISTANCE){
			speed_correction = 0;
		}else{
			//computes the rotation
			speed_correction = p_regulator_angle(get_line_position(), IMAGE_BUFFER_SIZE/2);
		}
		//compute the speed
		speed = pi_regulator_distance(VL53L0X_get_dist_mm()*MMTOCM, GOAL_OBSTICLE_DISTANCE);

		//applies the speed and the correction for the rotation
		right_motor_set_speed(speed - speed_correction);
		left_motor_set_speed(speed + speed_correction);

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
