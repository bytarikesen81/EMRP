/*
 * mpu6050.c
 *
 *  Created on: Feb 3, 2026
 *      Author: Tarik Esen
 */

#include "mpu6050.h"

static HAL_StatusTypeDef mpu_read(I2C_HandleTypeDef*, uint8_t, uint8_t *, uint16_t);
static HAL_StatusTypeDef mpu_write(I2C_HandleTypeDef*, uint8_t, uint8_t);

// 0:OK, <0 ERROR
int mpu6050_init(I2C_HandleTypeDef* hi2c)
{
    uint8_t who = 0;
    if (mpu_read(hi2c, REG_WHO_AM_I, &who, 1) != HAL_OK) return -1;
    if (who != 0x68) return -2;  // WHO_AM_I beklenen

    // Wake up (disable sleep bit)
    if (mpu_write(hi2c, REG_PWR_MGMT_1, 0x00) != HAL_OK) return -3;
    HAL_Delay(50);

    // Sample rate = Gyro output / (1 + SMPLRT_DIV). Gyro output default 8kHz (DLPF=0) or 1kHz (DLPF!=0)
    mpu_write(hi2c, REG_SMPLRT_DIV, 0x07);   // örnek: 1kHz/(1+7)=125Hz (DLPF ON is assumed)
    mpu_write(hi2c, REG_CONFIG, 0x03);       // DLPF ~44Hz (typical)
    mpu_write(hi2c, REG_GYRO_CONFIG, 0x00);  // ±250 dps
    mpu_write(hi2c, REG_ACCEL_CONFIG, 0x00); // ±2g

    return 0;
}

// 0:OK, <0 ERROR
int mpu6050_read_raw(I2C_HandleTypeDef* hi2c, mpu6050_raw_t *out)
{
    uint8_t b[14];
    if (mpu_read(hi2c, REG_ACCEL_XOUT_H, b, 14) != HAL_OK) return -1;

    out->ax   = (int16_t)((b[0] << 8) | b[1]);
    out->ay   = (int16_t)((b[2] << 8) | b[3]);
    out->az   = (int16_t)((b[4] << 8) | b[5]);
    out->temp = (int16_t)((b[6] << 8) | b[7]);
    out->gx   = (int16_t)((b[8] << 8) | b[9]);
    out->gy   = (int16_t)((b[10] << 8) | b[11]);
    out->gz   = (int16_t)((b[12] << 8) | b[13]);

    return 0;
}

void mpu6050_scale(const mpu6050_raw_t *r, mpu6050_scaled_t *s)
{
    s->ax_g = r->ax / 16384.0f;
    s->ay_g = r->ay / 16384.0f;
    s->az_g = r->az / 16384.0f;

    s->gx_dps = r->gx / 131.0f;
    s->gy_dps = r->gy / 131.0f;
    s->gz_dps = r->gz / 131.0f;

    s->temp_c = (r->temp / 340.0f) + 36.53f;
}

//I2C bus read/write functions
static HAL_StatusTypeDef mpu_read(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 200);
}

static HAL_StatusTypeDef mpu_write(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 200);
}
