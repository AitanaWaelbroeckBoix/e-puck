#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <stdbool.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>

#include <process_image.h>

static int16_t error_line_pos = 0;	//line is in the middle
static uint8_t stop_or_go = GO;
static bool camera_mode = false;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//Returns the error line position. The center of the line represents an error = 0. The error detected is normalized between [-320, 320].
int16_t error_line_position(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;
	int16_t error_position = 0 ;
	static int16_t last_error_position = 0 ;

	//performs an average
	for(uint16_t j = 0 ; j < IMAGE_BUFFER_SIZE ; j++){
		mean += buffer[j];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;

		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
			if (i > IMAGE_BUFFER_SIZE || !end)
			{
				line_not_found = 1;
			}
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH)
		{
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}

	}while(wrong_line);

	// if no line was found, we return the last error measured
	if(line_not_found)
	{
		begin = 0;
		end = 0;
		error_position = last_error_position;
	}
	//if a line was found, we return the new normalized measure of the error
	else
	{
		last_error_position = error_position = (begin + end)/2 - IMAGE_BUFFER_SIZE/2;
	}

	return error_position;
}

//returns if a red traffic light has been detected
uint8_t traffic_light(uint8_t *buffer){

	uint32_t max_mean = 0, local_mean = 0;
	uint16_t center_of_light = 0;
	uint8_t min_contrast_number = 0;
	uint8_t max_contrast_number = 0;

	//initialization of the local_mean. This mean value of intensity is calculated over NB_PX_LOCAL_MEAN (200) pixels
	for(uint16_t i = 0 ; i < (NB_PX_LOCAL_MEAN) ; i++){
		local_mean += buffer[i];
		max_mean= local_mean;
		center_of_light = NB_PX_LOCAL_MEAN/2;
	}

	//search for the center of max_mean, max_mean is the highest mean value computed on NB_PX_LOCAL_MEAN number of pixels
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE - NB_PX_LOCAL_MEAN ; i++){
		local_mean = buffer[NB_PX_LOCAL_MEAN+i]-buffer[i] + local_mean;

		if(local_mean > max_mean){
			max_mean = local_mean;
			center_of_light = i + NB_PX_LOCAL_MEAN/2;
		}
	}

	//computes the number of pixels with "very high" and "very low" intensity around the peak of intensity
	for(uint16_t i = center_of_light - NB_PX_LOCAL_MEAN/2 ; i < center_of_light + NB_PX_LOCAL_MEAN/2 ; i++ ){
		if(buffer[i] < MIN_INT_THRESHOLD ){
			min_contrast_number ++;
		}
		if(buffer[i] > MAX_INT_THRESHOLD){
			max_contrast_number ++;
		}
	}

	//chprintf((BaseSequentialStream *)&SD3,"- MAX_CONTRAST %d -", max_contrast_number);
	//chprintf((BaseSequentialStream *)&SD3,"- MIN_CONTRAST %d-", min_contrast_number );

	//if there are less pixels at high intensity than MIN_INT_NB_PX, the robot doesn't stop and the body led is on
	//this is the case when the traffic light is off
	if(max_contrast_number < MAX_INT_NB_PX){
		///chprintf((BaseSequentialStream *)&SD3,"- NO_LIGHT -");
		leds_go();
		return GO;
	}
	// if there are more pixels at low intensity than MIN_INT_NB_PX, then it means that the light is green
	if(min_contrast_number > MIN_INT_NB_PX){
		//chprintf((BaseSequentialStream *)&SD3,"- GREEN -");
		leds_go();
		return GO;
	}
	// else, the light is red
	else{
		//chprintf((BaseSequentialStream *)&SD3,"- RED -");
		leds_stop();
		return STOP;
	}
}

//in charge of capturing images for the following of the line and the traffic light alternately
static THD_WORKING_AREA(waCaptureImage, 1024);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//double buffering allows treating the image for the following of the line while the one for the
    //traffic light is being taken, and so on
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);

    while(1){

    	//depending on the camera mode, an image is taken for the following of the line or  for the traffic light
    	switch(camera_mode){

			case FOLLOW_LINE:
				//we take the last two lines to have the more reliable information on the line
				po8030_advanced_config(FORMAT_RGB565, 0, MODE_LINE, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
				break;

			case TRAFFIC_LIGHT:
				//we take the first two lines to react immediately after the detection of the traffic light
				po8030_advanced_config(FORMAT_RGB565, 0, MODE_TRAFFIC, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
				break;
    	}
    	dcmi_prepare();
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//switching of the camera mode before the semaphore to avoid problems due to the thread ProcessImage
		camera_mode =!camera_mode;
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

//in charge of processing the images for the following of the line and the traffic light alternately
static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	bool camera_mode_local = false;

    while(1){

    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);

        //re-inversion of the camera_mode to use the right buffer sent by the thread CaptureImage
        camera_mode_local = !camera_mode;
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//process of the images depending on camera_mode_local
		switch(camera_mode_local){

			case FOLLOW_LINE:
				//search for a line in the image and gets its position error
				error_line_pos = error_line_position(image);
				break;

			case TRAFFIC_LIGHT:
				//search for a traffic light to know if it has to stop or continue moving
				stop_or_go = traffic_light(image);
				break;
		}
    }
}

int16_t get_error_line_position(void){
	return error_line_pos;
}

uint8_t get_stop_or_go(void){
	return stop_or_go;
}

void process_image_start(void){
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
}
