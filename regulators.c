#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <regulators.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>

// PID regulator to keep the robot at a constant distance of the obstacle that's in front of him
int16_t pid_regulator_tof(float distance, uint16_t goal){

	float error = 0, speed = 0;
	static float sum_error = 0, alt_error_pid = 0;

	error = distance - goal;

	//disables the PID regulator if the error is to small
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

	speed = KP_1 * error + KI_1 * sum_error + KD_1 * ((error - alt_error_pid)/PI_CLOCK);

	alt_error_pid = error;

	//chprintf((BaseSequentialStream *)&SD3,"- SPEED %d -", speed);

    return (int16_t)speed;
}

// PD regulator lets the robot rotate to always face the center of the line he is meant to follow
//error in pixels
int16_t pd_regulator_ligne(int16_t error){

	float speed = 0;
	static int16_t alt_error_pd =0;

	//disables the PD regulator if the error is to small
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
static THD_WORKING_AREA(waRegulators, 256);
static THD_FUNCTION(Regulators, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed , speed_correction = 0;
    float distance = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //CHECK OF THE TRAFFIC LIGHT: if there is no traffic light, or if the traffic light is green or off,
        //the robot goes at a speed set by the PD of the ToF
        if (get_stop_or_go()){

            //transforms the distance between the robot and the obstacle in cm
        	distance = VL53L0X_get_dist_mm()/10;

        	//chprintf((BaseSequentialStream *)&SD3,"- DISTANCE %d -", distance);

        	//CHECK OF THE ToF: only uses the PD regulator of the ToF when the robot is too close of the obstacle
        	//of the obstacle in front of him
        	//PD of the ToF sets the speed so that the robot always stays at GOAL_DISTANCE (8 cm)
        	if(distance > GOAL_DISTANCE){
        		speed = MAX_SPEED;
        	}
        	else{
        		speed = pid_regulator_tof(distance, GOAL_DISTANCE);
        	}
        }
        //if the traffic light is red, the robot stops
        else{
        	speed = STOP;
        }

        //computes a correction factor to let the robot rotate to face de line
        speed_correction = pd_regulator_ligne(get_error_line_position());

        //applies the speed from the Pd regulators and the correction for the rotation
        right_motor_set_speed(speed - speed_correction);
        left_motor_set_speed(speed + speed_correction);

        //chprintf((BaseSequentialStream *)&SD3,"- ERROR %d -",get_error_line_position());
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waRegulators, sizeof(waRegulators), NORMALPRIO, Regulators, NULL);
}
