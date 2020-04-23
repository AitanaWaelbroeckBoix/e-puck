#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

static uint16_t error_line_pos = 0;	//line is in the middle
static uint8_t stop_or_go = GO;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

uint32_t compute_average(uint8_t *buffer){

	uint32_t mean = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	return mean;
}

//Returns the error line position
uint16_t error_line_position(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_line = 0;
	uint32_t mean = 0;
	int16_t error_position =0;

	mean = compute_average(buffer);

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
		    //if an end was not found, we return a positive error to turn right
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		       error_position= begin/2;
		       return error_position;
		    }
		}
		else//if no begin was found, we return a negative error to turn left
		{
	        error_position= -IMAGE_BUFFER_SIZE/2;
	        return error_position;
		}

		//if a line too small has been detected, continues the search
		if((end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);
	error_position= (begin + end)/2 - IMAGE_BUFFER_SIZE/2;

	return error_position;
}

uint8_t traffic_light(uint8_t *buffer){

	uint32_t max_mean = 0, local_mean = 0, general_mean =0;
	uint16_t center_of_light = 0;
	uint8_t contrast_number = 0;

	general_mean = compute_average (buffer);

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

	max_mean /= NB_PX_LOCAL_MEAN;

	chprintf((BaseSequentialStream *)&SD3,"- MAX_MEAN %d -", max_mean);
	chprintf((BaseSequentialStream *)&SD3,"- GENERAL_MEAN %d -", general_mean);

	// if no light is on, the max_mean won't be much higher than general_mean, then it returns GO
	if(max_mean < general_mean * MEAN_COEFF){
			chprintf((BaseSequentialStream *)&SD3,"- GO -");
			return GO;
	}

	// compute contrast of the light around the peak of intensity
	for(uint16_t i = center_of_light - NB_PX_LOCAL_MEAN/2 ; i < center_of_light + NB_PX_LOCAL_MEAN/2 ; i++ ){
		if(buffer[i] < general_mean * CONTRAST_COEFF){
			contrast_number ++;
		}
	}

	chprintf((BaseSequentialStream *)&SD3,"- CONTRAST %d -", contrast_number);

	// if there are many pixels at low red intensity
	// then it means that the light is green
	if(contrast_number > MIN_CONTRAST){

		chprintf((BaseSequentialStream *)&SD3,"- GO -");
		return GO;
	}
	// else, the light is red
	else{

		chprintf((BaseSequentialStream *)&SD3,"- STOP -");
		return STOP;
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 1, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

// ancienne version thread
static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	bool send_to_computer = true;

    while(1){

    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its position error
		error_line_pos = error_line_position(image);
		stop_or_go = traffic_light(image);

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}

		//invert the bool
		send_to_computer = !send_to_computer;
    }
}

uint16_t get_error_line_position(void){
	return error_line_pos;
}

uint8_t get_stop_or_go(void){
	return stop_or_go;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
