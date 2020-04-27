#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>

//simple PI regulator implementation
//error in cm
int16_t pid_regulator(float distance, float goal){

	float error = 0, speed = 0;
	static float sum_error = 0, alt_error_pid = 0;

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

	speed = KP_1 * error + KI * sum_error + KD_1 * ((error - alt_error_pid)/PI_CLOCK);
	alt_error_pid = error;

	chprintf((BaseSequentialStream *)&SD3,"- SPEED %d -", speed);

    return (int16_t)speed;
}

// PD regulator to let the robot rotate to be front of the line
//error in pixels
uint16_t pd_regulator_rotation(uint16_t error)
{
	uint16_t speed = 0;
	static float alt_error_pd =0;

	if(fabs(error) < ROTATION_THRESHOLD){
		return 0;
	}

	speed = KP_2 * error + KD_2 * ((error - alt_error_pd)/PI_CLOCK);
	alt_error_pd = error;

    return (uint16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed = 0, speed_correction = 0;
    uint16_t distance = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        if (get_stop_or_go()){
            //sets the speed so that the robot always stays at 10cm of the car in front of him
        	/*distance = VL53L0X_get_dist_mm()/10;

        	chprintf((BaseSequentialStream *)&SD3,"- DISTANCE %d -", distance);
        	if(distance > GOAL_DISTANCE){<
        		speed = 200;
        	}
        	else{
        		speed = pid_regulator(distance, GOAL_DISTANCE);

        	}*/
        	speed = 200;

        }
        else{
        	speed = STOP;
        }

        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = pd_regulator_rotation(get_error_line_position());

        //applies the speed from the PI regulator and the correction for the rotation
        right_motor_set_speed(speed - speed_correction);
        left_motor_set_speed(speed + speed_correction);

        //chprintf((BaseSequentialStream *)&SD3,"- ERROR %d -",get_error_line_position());
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
