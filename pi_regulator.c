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


// PD regulator to keep the robot at a constant distance of the obstacle that's in front of him
int16_t pd_regulator_tof(float distance, float goal){

	float error = 0, speed = 0;
	static float alt_error_pd = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	speed = KP_1 * error + KD_1 * ((error - alt_error_pd)/PI_CLOCK);
	alt_error_pd = error;

	//chprintf((BaseSequentialStream *)&SD3,"- SPEED %d -", speed);

    return (int16_t)speed;
}

// PD regulator lets the robot rotate to always face the center of the line he is meant to follow
//error in pixels
int16_t pd_regulator_ligne(float error)
{
	float speed = 0;
	static float alt_error_pd =0;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ROTATION_THRESHOLD){
		return 0;
	}

	speed = KP_2*error + KD_2 * ((error - alt_error_pd)/PI_CLOCK);
	alt_error_pd = error;

    return (int16_t)speed;
}

//thread in charge of the PD regulators
static THD_WORKING_AREA(waPdRegulators, 256);
static THD_FUNCTION(PdRegulators, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed , speed_correction = 0;
    uint16_t distance = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //CHECK OF TRAFFIC LIGHT: if there is no traffic light, or if the traffic light is green or off,
        //the robot goes at a speed set by the PD of the ToF
        if (get_stop_or_go()){

            //transforms the distance between the robot and the obstacle in cm
        	distance = VL53L0X_get_dist_mm()/10;

        	//chprintf((BaseSequentialStream *)&SD3,"- DISTANCE %d -", distance);

        	//CHECK OF ToF: only uses the PD regulator of the ToF when the robot is too close of the obstacle
        	//of the obstacle in front of him
        	//PD of the ToF sets the speed so that the robot always stays at GOAL_DISTANCE (8 cm)
        	if(distance > GOAL_DISTANCE){
        		speed = MAX_SPEED;
        	}
        	else{
        		speed = pd_regulator_tof(distance, GOAL_DISTANCE);
        	}
        }
        //if the traffic light is red, the robot stops
        else{
        	speed = STOP;
        }

        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = pd_regulator_ligne(get_error_line_position());
       // speed_correction = get_error_line_position();

        //applies the speed from the Pd regulators and the correction for the rotation
        right_motor_set_speed(speed - speed_correction);
        left_motor_set_speed(speed+ speed_correction);

        //chprintf((BaseSequentialStream *)&SD3,"- ERROR %d -",get_error_line_position());
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPdRegulators, sizeof(waPdRegulators), NORMALPRIO, PdRegulators, NULL);
}
