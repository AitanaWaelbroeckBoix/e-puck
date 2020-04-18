#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

//simple PID regulator implementation for the translation
int16_t pid_regulator_translation(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;
	static float alt_error = 0;

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

	speed = KP_1 * error + KI * sum_error + KD_1 * ((error - alt_error)/PI_CLOCK);
	alt_error = error;
    return (int16_t)speed;
}
// PD regulator to let the robot rotate to be front of the line
uint16_t pd_regulator_rotation(uint16_t error)
{
	uint16_t speed = 0;
	static uint16_t alt_error = 0;

	if(fabs(error) < ERROR_THRESHOLD)
	{
		return 0;
	}

	speed = KP_2 * error + KD_2 * ((alt_error - error)/PI_CLOCK);
	alt_error = error;

    return (uint16_t)speed;

}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_rotation = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        if (get_stop_or_go()){

        	speed = 200;
        }
        else{
        	speed = STOP;
        }

        // give the value of the rotation speed to be on the line
        speed_rotation = pd_regulator_rotation(get_error_line_position());

        //applies the speed from the PI regulator and the correction for the rotation
        //right_motor_set_speed(speed - speed_rotation);
		//left_motor_set_speed(speed + speed_rotation);
        right_motor_set_speed(speed);
        left_motor_set_speed(speed );

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
