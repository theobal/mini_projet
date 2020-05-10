#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <main.h>
#include <camera/po8030.h>
#include <process_image.h>
#include <selector.h>

static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE); // @suppress("Field cannot be resolved")

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_position(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
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
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
	}else{
		line_position = (begin + end)/2; //gives the line position.
	}

	return line_position;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 470, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t image_16b[IMAGE_BUFFER_SIZE] = {0};
	uint16_t mask = 0x0000;
	uint8_t shift = 0;
	bool send_to_computer = true;

	//selection of color to extract
	while(1){

		switch(get_selector()){
		case 1:
			mask = MASK_R;
			shift = SHIFT_R;
			break;
		case 2:
			mask = MASK_B;
			shift = 0;
			break;
		case 3:
			mask = MASK_G;
			shift = SHIFT_G;
			break;
		default:
			mask = MASK_R;
			shift = SHIFT_R;
		}

		//waits until an image has been captured
		chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts one color from the image
		for(uint16_t i = 0 ; i < (2*IMAGE_BUFFER_SIZE) ; i+=2){
			//concatenation of the two bytes from 3 color pixels
			image_16b[i/2] = (img_buff_ptr[i] << 8) | (img_buff_ptr[i+1]&0x00FF);
			//extract one color and shift the bytes to the lowest value, then convert into uint8
			image[i/2] = (uint8_t)((image_16b[i/2]&mask) >> shift);
		}
		//search for a line in the image and gets its position
		line_position = extract_line_position(image);

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
	}
}


uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
