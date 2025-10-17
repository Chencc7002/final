#ifndef __MPU6500_H
#define __MPU6500_H

#include "main.h"

/* ------------------ MPU6500相关定义 ------------------ */
#define MPU6500_ADDR 0xD0
#define MPU6500_ACCEL_REG 0x3B
#define ACCEL_SENS 16384.0f
#define G 9.80665f
#define GYRO_SENS 131.0f

// MPU6500数据结构
typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    float Accel_X_mps2;
    float Accel_Y_mps2;
    float Accel_Z_mps2;
    float Gyro_X_dps;
    float Gyro_Y_dps;
    float Gyro_Z_dps;
} MPU6500_Data;

// MPU6500校准数据结构
typedef struct {
    int16_t ax_offset;
    int16_t ay_offset;
    int16_t az_offset;
    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;
    uint8_t calibrated;
    float dt;
    float velocity_x;
    float distance_x;
    float accel_x_filt;
    float accel_y_filt;
    float accel_z_filt;
    float gyro_x_filt;
    float gyro_y_filt;
    float gyro_z_filt;
} MPU6500_Calib;

uint8_t MPU6500_Init(I2C_HandleTypeDef *hi2c);
void MPU6500_Calibrate(I2C_HandleTypeDef *hi2c, MPU6500_Calib *calib);
uint8_t MPU6500_Read_All(I2C_HandleTypeDef *hi2c, MPU6500_Data *data);
void MPU6500_GetCalibratedData(MPU6500_Data *raw, MPU6500_Calib *calib, MPU6500_Data *out);
void MPU6500_Convert_Unit(MPU6500_Data *data);
void MPU6500_LowPassFilter(MPU6500_Data *data, MPU6500_Calib *calib, float alpha);
void MPU6500_Update_XMotion(MPU6500_Data *calib_data, MPU6500_Calib *calib);
void SendMPUDataToBluetooth(MPU6500_Data *data, MPU6500_Calib *calib);

#endif /* __MPU6500_H */