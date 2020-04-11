#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

//PID regulator for translation
int16_t pi_regulator_translation(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float alt_error =0;
	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
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

	speed = KP_1 * error + KI * sum_error + KD * ((error - alt_error)/ CLOCK_PI);

    return (int16_t)speed;
}

// P regulator to let the robot rotate to be front of the line
uint16_t pi_regulator_rotation(int16_t line_position)
{
	uint16_t error = 0;
	uint16_t speed = 0;

	error = line_position - (IMAGE_BUFFER_SIZE/2);

	if(fabs(error) < ERROR_THRESHOLD)
	{
		return 0;
	}

	speed = KP_2 * error;

    return speed;

}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed_translation = 0;
    int16_t speed_rotation = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        if (get_distance_cm() != 0)
        {
        	speed_translation = pi_regulator_translation(get_distance_cm(), GOAL_DISTANCE);
        }
        else
        	speed_rotation = REGULAR_SPEED;

        //computes a correction factor to let the robot rotate to be in front of the line
        speed_rotation = pi_regulator_rotation(get_line_position());

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed_translation - ROTATION_COEFF * speed_rotation);
		left_motor_set_speed(speed_translation + ROTATION_COEFF * speed_rotation);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
