#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "leds.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <regulators.h>
#include <process_image.h>

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //starts the camera
    dcmi_start();
	po8030_start();
	//initializes the motors
	motors_init();
	//starts ToF sensor
	VL53L0X_start();

	//starts the threads for the regulators and the processing of the image
	process_image_start();
	regulators_start();

	/* Infinite loop. */
	while (1) {
	//waits 1 second
	chThdSleepMilliseconds(1000);
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
