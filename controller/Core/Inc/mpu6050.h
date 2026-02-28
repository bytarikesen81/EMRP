/*
 * mpu6050.h
 *
 *  Created on: Feb 3, 2026
 *      Author: Tarik Esen
 */


#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU_ADDR        (0x68 << 1) // If AD0=0 0x68, AD0=1 0x69
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

#include "stm32f0xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

//IMU Packet
#pragma pack(1)
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} mpu6050_raw_t;

//Scaled IMU Packet
typedef struct {
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
    float temp_c;
} mpu6050_scaled_t;
#pragma pack(0)

int mpu6050_init(I2C_HandleTypeDef*);
int mpu6050_read_raw(I2C_HandleTypeDef*, mpu6050_raw_t *);
void mpu6050_scale(const mpu6050_raw_t *, mpu6050_scaled_t *);
#endif /* INC_MPU6050_H_ */
