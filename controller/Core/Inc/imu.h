/*
 * imu.h
 *
 *  Created on: Feb 3, 2026
 *      Author: Tarik Esen
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f0xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "mpu6050.h"

#define MAX_WINDOW_SIZE 255

int imu_send_window(I2C_HandleTypeDef*, UART_HandleTypeDef*, uint8_t*, uint8_t, uint8_t, uint8_t, int);

#endif /* INC_IMU_H_ */
