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

//Returns the error line position
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
		        chprintf((BaseSequentialStream *)&SD3,"- BEGIN %d -", begin);
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
		            chprintf((BaseSequentialStream *)&SD3,"- END %d -", end);
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
			if (i > IMAGE_BUFFER_SIZE || !end)
			{
				line_not_found = 1;
				//end = IMAGE_BUFFER_SIZE;
				chprintf((BaseSequentialStream *)&SD3,"- NO_END -");
			}
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		    chprintf((BaseSequentialStream *)&SD3,"- NO_BEGIN -");
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

	if(line_not_found)
	{
		begin = 0;
		end = 0;
		chprintf((BaseSequentialStream *)&SD3,"- PRISE_LAST_ERROR %d -", last_error_position );
		error_position = last_error_position;
	}
	else
	{
		last_error_position = error_position = (begin + end)/2 - IMAGE_BUFFER_SIZE/2;
		chprintf((BaseSequentialStream *)&SD3,"- NOUVELLE_ERROR %d -", error_position);
	}
	return error_position;
}

uint8_t traffic_light(uint8_t *buffer){

	uint32_t max_mean = 0, local_mean = 0;
	uint16_t center_of_light = 0;
	uint8_t min_contrast_number = 0;
	uint8_t max_contrast_number = 0;
	//uint32_t min, max=0;

	// initialise local_mean
	for(uint16_t i = 0 ; i < (NB_PX_LOCAL_MEAN) ; i++){
		local_mean += buffer[i];
		max_mean= local_mean;
		center_of_light = NB_PX_LOCAL_MEAN/2;
	}

	// search for max_mean, max_mean is the highest mean value computed on NB_PX_LOCAL_MEAN number of pixels
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE - NB_PX_LOCAL_MEAN ; i++){
		local_mean = buffer[NB_PX_LOCAL_MEAN+i]-buffer[i] + local_mean;

		if(local_mean > max_mean){
			max_mean = local_mean;
			center_of_light = i + NB_PX_LOCAL_MEAN/2;
		}
	}

	// compute contrast of the light around the peak of intensity
	for(uint16_t i = center_of_light - NB_PX_LOCAL_MEAN/2 ; i < center_of_light + NB_PX_LOCAL_MEAN/2 ; i++ ){
		if(buffer[i] < MIN_INT_THRESHOLD ){
			min_contrast_number ++;
			//chprintf((BaseSequentialStream *)&SD3,"- Itensité min %d -", min_contrast_number);
		}
		if(buffer[i] > MAX_INT_THRESHOLD){
			max_contrast_number ++;
			//chprintf((BaseSequentialStream *)&SD3,"- Itensité max %d -", max_contrast_number);
		}
	}

	//chprintf((BaseSequentialStream *)&SD3,"- MAX_CONTRAST %d -", max_contrast_number);
	//chprintf((BaseSequentialStream *)&SD3,"- MIN_CONTRAST %d-", min_contrast_number );

	// there are pixels at high itensity
	if(max_contrast_number < MAX_INT_NB_PX){
		///chprintf((BaseSequentialStream *)&SD3,"- NO_LIGHT -");
		leds_go();
		return GO;
	}
	// then it means that the light is green
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

static THD_WORKING_AREA(waCaptureImage, 1024);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);

    while(1){

    	switch(camera_mode){
			case FOLLOW_LINE:
				po8030_advanced_config(FORMAT_RGB565, 0, MODE_LINE, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
				//chprintf((BaseSequentialStream *)&SD3,"- LINE_capture -");
				break;
			case TRAFFIC_LIGHT:
				po8030_advanced_config(FORMAT_RGB565, 0, MODE_TRAFFIC, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
				//chprintf((BaseSequentialStream *)&SD3,"- TRAFFIC_capture -");
				break;
    	}
    	dcmi_prepare();
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		camera_mode =!camera_mode;
        //chprintf((BaseSequentialStream *)&SD3,"- CAMERA MODE_capture %d -", camera_mode);
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	bool send_to_computer = true;
	bool camera_mode_local = false;

    while(1){

    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
        //chprintf((BaseSequentialStream *)&SD3,"- CAMERA MODE_process %d -", camera_mode);
        camera_mode_local = !camera_mode;
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		switch(camera_mode_local){
			case FOLLOW_LINE:
				//search for a line in the image and gets its position error
				error_line_pos = error_line_position(image);
				//chprintf((BaseSequentialStream *)&SD3,"- LINE_process -");

				if(send_to_computer){
				//sends to the computer the image
					SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
				}
				//invert the bool
				send_to_computer = !send_to_computer;

				break;

			case TRAFFIC_LIGHT:
				//search for a traffic ligh to if it has to stop or continue moving
				stop_or_go = traffic_light(image);
				//chprintf((BaseSequentialStream *)&SD3,"- TRAFFIC_process -");
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
