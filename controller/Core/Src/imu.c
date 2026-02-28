/*
 * imu.c
 *
 *  Created on: Feb 3, 2026
 *      Author: Tarik Esen
 */

#include "imu.h"

static int imu_collect_measure(I2C_HandleTypeDef*, int, uint8_t*);

int imu_send_window(I2C_HandleTypeDef* hi2c, UART_HandleTypeDef* huart, uint8_t* uart_tx_buf, uint8_t buf_size, uint8_t param_size, uint8_t window_size, int wait_per_frame){
	uint8_t measure_buf[MAX_WINDOW_SIZE*12];
	uint8_t ret_val;
	uint8_t sample_per_frame;
	int sample_index=0, frame_index;

	//Magic header
	memcpy((void*)(uart_tx_buf + param_size), (const void *__restrict)"TE", 2);

	//Number of frames
	sample_per_frame = (buf_size-(param_size+4))/(12); //Sample per frame
	uart_tx_buf[param_size + 2] = (window_size/sample_per_frame) + (window_size%sample_per_frame ? 1:0); //Number of frames

	//Proceed with measurements until the amount enough to fill the window is reached
	ret_val = imu_collect_measure(hi2c, window_size, measure_buf);
	if(ret_val == 0){
		//Start UART stream after the measurements are collected
		for(int i=0; i<uart_tx_buf[param_size + 2]; i++){
			uart_tx_buf[param_size + 3] = i; //Frame number

			//Fill the rest of the frame with enough number of samples
			frame_index = 0;
			while(sample_index < window_size && frame_index < 4){
				memcpy(&(uart_tx_buf[param_size + 4 + 12*frame_index]), &(measure_buf[12*sample_index]), 12);
				sample_index++;
				frame_index++;
			}

			//Send the frame through UART bus
			if(HAL_UART_Transmit(huart, uart_tx_buf, buf_size, 100) == HAL_OK){
				//Wait until the UART module is ready
				HAL_Delay(wait_per_frame);
			}
		}
	}
	else return ret_val;

	return 0;
}

//Bulk measure as many samples as passed window size
static int imu_collect_measure(I2C_HandleTypeDef* hi2c, int measure_count, uint8_t* measure_buf){
	mpu6050_raw_t imu_r;

	if(measure_count > (MAX_WINDOW_SIZE))
		return -1;
	else if(measure_buf == NULL)
		return -2;

	for(int i=0; i < measure_count; i++){
		if (mpu6050_read_raw(hi2c, &imu_r) == 0) {
			memcpy(&(measure_buf[i*12]), &imu_r, 12);
		}
		else return -3;
	}

	return 0;
}
