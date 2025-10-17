/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ??? + ADC + ???? + RPLIDAR C1 + MPU6500 ???
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms in LICENSE file or provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>   // for bool

/* ------------------ ?????? ------------------ */
#define MAX_SPEED 400
#define SPEED_STEP 150
#define SPEED_UPDATE_INTERVAL 30
#define TURN_DURATION 800
#define TURN_SPEED 300
#define DIR_STOP 0
#define DIR_FORWARD 1
#define DIR_BACKWARD 2
#define DIR_LEFT 3
#define DIR_RIGHT 4
#define DIR_TURNING 5
#define PPR 360
#define SAMPLE_TIME_MS 10

/* ------------------ RPLIDAR C1 ???? ------------------ */
#define DMA_BUFFER_SIZE 256          // DMA??????
#define LIDAR_SAMPLE_PACKET_SIZE 5   // C1????????(????4-5)
#define LIDAR_START_RESP_SIZE 7      // C1????????(????2-7)
#define ANGLE_FILTER_THRESHOLD 0.1f  // ?????? (deg)
#define MIN_VALID_DISTANCE 50.0f     // C1??????(??2-1:0.05m=50mm)
#define MAX_VALID_DISTANCE 12000.0f  // C1??????(??2-1:12m=12000mm)
#define LIDAR_TIMEOUT_THRESHOLD 3000  // ??:?5000ms??3000ms
#define LIDAR_STOP_DELAY 10          // STOP????(??4-2??=10ms)
#define LIDAR_RESET_DELAY 500        // RESET????(??4-3??=500ms)

/* ------------------ MPU6500?? ------------------ */
#define MPU6500_ADDR 0xD0
#define MPU6500_ACCEL_REG 0x3B
#define ACCEL_SENS 16384.0f
#define G 9.80665f
#define GYRO_SENS 131.0f

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// MPU6500???? (unchanged)
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

// RPLIDAR C1?????
typedef enum {
    LIDAR_STATE_WAIT_START_RESP,  // ???????
    LIDAR_STATE_WAIT_SAMPLE_DATA  // ??????
} LidarState;

// RPLIDAR C1????????(????4-5)
// ????: +0 sync_quality, +1 angle_q6 low byte (bit0 = C), +2 angle_q6 high byte, +3 distance low, +4 distance high
// sync_quality: bit7..bit2 = quality(6bit), bit1 = ~S, bit0 = S
typedef struct {
    uint8_t sync_quality;  // bit7..bit2=quality(6), bit1=~S, bit0=S
    uint16_t angle_q6;     // 16-bit little-endian
    uint16_t distance_q2;  // 16-bit little-endian, q2
} __attribute__((packed)) LidarSamplePacket;

// RPLIDAR C1????????(????2-7)
typedef struct {
    uint8_t start_flag1;  // 0xA5
    uint8_t start_flag2;  // 0x5A
    uint8_t resp_len[3];  // length (30bit, little-endian)
    uint8_t resp_mode;    // mode
    uint8_t data_type;    // data type (SCAN = 0x81)
} __attribute__((packed)) LidarStartRespPacket;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ------------------ ?????? ------------------ */
uint8_t bluetooth_rx_data = 0;
uint8_t target_direction = DIR_STOP;
uint8_t current_direction = DIR_STOP;
uint32_t current_speed = 0;
uint32_t target_speed = 0;
uint32_t last_speed_update_time = 0;

uint8_t bluetooth_connected = 0;
uint8_t connection_announced = 0;
uint8_t connection_msg[] = "Connected\r\n";

uint32_t turn_start_time = 0;
uint8_t turn_direction = DIR_LEFT;
uint8_t is_turning = 0;
uint8_t turn_state = 0;

int32_t lastEncoderA = 0;
int32_t lastEncoderB = 0;

char uart_buf[100];

/* ------------------ RPLIDAR C1 ------------------ */
uint8_t lidar_dma_buffer[DMA_BUFFER_SIZE];       // DMA receive buffer (circular)
uint32_t lidar_rxIndex = 0;                      // read index in DMA buffer
float lastLidarAngle = 0.0f;                     // last reported angle
uint32_t lastLidarRxTime = 0;                    // last rx tick
LidarState lidar_state = LIDAR_STATE_WAIT_START_RESP;  // current state
uint8_t lidar_start_resp_buf[LIDAR_START_RESP_SIZE];    // buffer for start response
uint8_t lidar_start_resp_idx = 0;                 // index

/* ------------------ MPU6500 ------------------ */
MPU6500_Data mpu_data;
MPU6500_Calib mpu_calib = {0};
uint8_t mpu_init_ok = 0;

/* ------------------ ?????? ------------------ */
uint32_t lastLidarProcessTime = 0;
uint32_t lastSensorSendTime = 0;
uint32_t lastEncoderReadTime = 0;
uint32_t lastMPUReadTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ControlMotor(uint8_t direction, uint32_t speed);
void UpdateSpeedRamp(void);
void SendConnectionNotification(void);
/* ??:ProcessLidarSampleData ?????? 5 ????????? bool */
bool ProcessLidarSampleData(const uint8_t *raw_buf);
void SendLidarToBluetooth(float angle, float distance, uint8_t quality);
void ProcessLidarDataFromDMA(void);
void LidarTimeoutRecovery(void);

/* MPU6500 prototypes (unchanged) */
uint8_t MPU6500_Init(I2C_HandleTypeDef *hi2c);
void MPU6500_Calibrate(I2C_HandleTypeDef *hi2c, MPU6500_Calib *calib);
uint8_t MPU6500_Read_All(I2C_HandleTypeDef *hi2c, MPU6500_Data *data);
void MPU6500_GetCalibratedData(MPU6500_Data *raw, MPU6500_Calib *calib, MPU6500_Data *out);
void MPU6500_Convert_Unit(MPU6500_Data *data);
void MPU6500_LowPassFilter(MPU6500_Data *data, MPU6500_Calib *calib, float alpha);
void MPU6500_Update_XMotion(MPU6500_Data *calib_data, MPU6500_Calib *calib);
void SendMPUDataToBluetooth(MPU6500_Data *data, MPU6500_Calib *calib);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ------------------ UART???? ------------------ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)  // USART1 callback (bluetooth)
    {
        if (!bluetooth_connected) bluetooth_connected = 1;
        if (bluetooth_rx_data >= '0' && bluetooth_rx_data <= '4')
        {
            target_direction = bluetooth_rx_data - '0';
            switch(target_direction) {
                case DIR_STOP:
                    target_speed = 0; break;
                case DIR_FORWARD:
                case DIR_BACKWARD:
                    target_speed = MAX_SPEED; break;
                case DIR_LEFT:
                case DIR_RIGHT:
                    target_speed = 0; break;
            }
        }
        HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);
    }
}

/* ------------------ ???? ------------------ */
void ControlMotor(uint8_t direction, uint32_t speed)
{
    switch(direction)
    {
        case DIR_STOP:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;
        case DIR_FORWARD:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(speed*0.98));
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;
        case DIR_BACKWARD:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
            break;
        case DIR_LEFT:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;
        case DIR_RIGHT:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TURN_SPEED);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;
        case DIR_TURNING:
            if (turn_direction == DIR_LEFT) {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            }
            else {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TURN_SPEED);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            }
            break;
    }
}

/* ?????? */
void UpdateSpeedRamp(void)
{
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_speed_update_time >= SPEED_UPDATE_INTERVAL)
    {
        last_speed_update_time = current_time;
        
        if (target_direction == DIR_STOP && current_direction != DIR_STOP)
        {
            current_speed = 0;
            current_direction = DIR_STOP;
            ControlMotor(DIR_STOP, 0);
            return; 
        }
        if (is_turning && turn_state == 1)
        {
            if (current_time - turn_start_time >= TURN_DURATION) {
                turn_state = 2;
                is_turning = 0;
                current_direction = DIR_STOP;
                target_speed = 0;
                ControlMotor(DIR_STOP, 0);
            } else {
                ControlMotor(DIR_TURNING, 0);
                return;
            }
        }
        if (target_direction != current_direction)
        {
            target_speed = 0;
            if (current_speed > 0)
            {
                current_speed -= SPEED_STEP;
                ControlMotor(current_direction, current_speed);
            }
            else
            {
                current_direction = target_direction;
                if (current_direction == DIR_LEFT || current_direction == DIR_RIGHT) {
                    is_turning = 1;
                    turn_state = 1;
                    turn_direction = current_direction;
                    turn_start_time = current_time;
                    ControlMotor(DIR_TURNING, 0);
                } else {
                    if (current_direction != DIR_STOP)
                        target_speed = MAX_SPEED;
                }
            }
        }
        else if (current_direction != DIR_STOP && !is_turning)
        {
            if (current_speed < target_speed)
                current_speed += SPEED_STEP;
            else if (current_speed > target_speed)
                current_speed -= SPEED_STEP;
            ControlMotor(current_direction, current_speed);
        }
    }
}

/* ???? */
void SendConnectionNotification(void)
{
    if (bluetooth_connected && !connection_announced) {
        HAL_UART_Transmit(&huart1, connection_msg, sizeof(connection_msg)-1, 1000);
        connection_announced = 1;
    }
}

/* ------------------ RPLIDAR C1 ?? ------------------ */

/*
 * ProcessLidarSampleData:
 *  - ??: raw_buf ?? 5 ???????? (??)
 *  - ??: true ????????????(??? SendLidarToBluetooth)
 *           false ????(???????????)
 */
bool ProcessLidarSampleData(const uint8_t *raw_buf)
{
    LidarSamplePacket pkt;
    memcpy(&pkt, raw_buf, sizeof(pkt)); // sizeof(pkt) == 5

    // ?? sync/quality
    uint8_t S = pkt.sync_quality & 0x01;             // bit0
    uint8_t not_S = (pkt.sync_quality >> 1) & 0x01;  // bit1
    uint8_t quality = (pkt.sync_quality >> 2) & 0x3F;// bit7..bit2 -> 6-bit quality (0..63)

    // angle_q6: 16-bit little-endian, bit0 is C (???)
    uint8_t C = (uint8_t)(pkt.angle_q6 & 0x01);      // ??? C
    uint16_t raw_angle_q6 = pkt.angle_q6;            // ?? little-endian ??
    uint16_t angle_field = (raw_angle_q6 >> 1) & 0x7FFF; // ?? C,?? 15 ?
    float angle = (float)angle_field / 64.0f;        // q6 -> deg

    // distance_q2: q2 ??, value/4.0 -> mm
    uint16_t raw_distance_q2 = pkt.distance_q2;
    float distance = (float)raw_distance_q2 / 4.0f;  // mm

    // ????????
    if ((S ^ not_S) != 0x01) {
        return false; // ????,??? 5 ????????
    }

    // ???????
    if (quality == 0) return false;
    if (distance < MIN_VALID_DISTANCE || distance > MAX_VALID_DISTANCE) return false;

    // ????/????(???????)
    if (fabsf(angle - lastLidarAngle) < ANGLE_FILTER_THRESHOLD && angle >= lastLidarAngle) {
        return false;
    }

    // ????????,?????
    SendLidarToBluetooth(angle, distance, quality);
    lastLidarAngle = angle;
    return true;
}

// ? DMA ????? LIDAR ?? (start-resp + ?? 5 ?????)
// ????????:? 5 ???????,??? 1 ????,?????
void ProcessLidarDataFromDMA(void)
{
    uint32_t currentIndex = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
    static uint32_t processed_bytes = 0;

    while (lidar_rxIndex != currentIndex) {
        uint8_t data = lidar_dma_buffer[lidar_rxIndex];
        lidar_rxIndex = (lidar_rxIndex + 1) % DMA_BUFFER_SIZE;
        lastLidarRxTime = HAL_GetTick();
        processed_bytes++;

        switch (lidar_state) {
            case LIDAR_STATE_WAIT_START_RESP:
                // ?????????
                if (lidar_start_resp_idx == 0) {
                    // ????????? 0xA5
                    if (data == 0xA5) {
                        lidar_start_resp_buf[lidar_start_resp_idx++] = data;
                    }
                } 
                else if (lidar_start_resp_idx == 1) {
                    // ?????????? 0x5A
                    if (data == 0x5A) {
                        lidar_start_resp_buf[lidar_start_resp_idx++] = data;
                    } else {
                        // ??0x5A,????
                        lidar_start_resp_idx = 0;
                    }
                }
                else {
                    // ???????
                    lidar_start_resp_buf[lidar_start_resp_idx++] = data;
                    
                    if (lidar_start_resp_idx >= LIDAR_START_RESP_SIZE) {
                        LidarStartRespPacket* resp = (LidarStartRespPacket*)lidar_start_resp_buf;
                        
                        // ????????????
                        if (resp->start_flag1 == 0xA5 && resp->start_flag2 == 0x5A) {
                            // ?????????,????0x81
                            if (resp->data_type == 0x81 || resp->data_type == 0x82 || 
                                resp->data_type == 0x83 || resp->data_type == 0x84) {
                                lidar_state = LIDAR_STATE_WAIT_SAMPLE_DATA;
                                lidar_start_resp_idx = 0;
                            } else {
                                // ???????,??????????????
                                lidar_state = LIDAR_STATE_WAIT_SAMPLE_DATA;
                                lidar_start_resp_idx = 0;
                            }
                        } else {
                            // ??????,????
                            lidar_start_resp_idx = 0;
                        }
                    }
                }
                break;

            case LIDAR_STATE_WAIT_SAMPLE_DATA:
            {
                static uint8_t sample_buf[LIDAR_SAMPLE_PACKET_SIZE];
                static uint8_t sample_idx = 0;
                static uint32_t samples_processed = 0;
                static uint32_t samples_valid = 0;
                
                sample_buf[sample_idx++] = data;
                
                if (sample_idx >= LIDAR_SAMPLE_PACKET_SIZE) {
                    samples_processed++;
                    
                    if (ProcessLidarSampleData(sample_buf)) {
                        samples_valid++;
                        sample_idx = 0;
                        
                        // ?50?????????(??????)
                        if (samples_valid % 50 == 0) {
                            char sample_msg[80];
                            snprintf(sample_msg, sizeof(sample_msg),
                                "[SAMPLE] Valid: %lu/%lu (%.1f%%)\r\n",
                                samples_valid, samples_processed, 
                                (float)samples_valid / samples_processed * 100.0f);
                            HAL_UART_Transmit(&huart1, (uint8_t*)sample_msg, strlen(sample_msg), 10);
                        }
                    } else {
                        // ????,????1??
                        memmove(sample_buf, sample_buf + 1, LIDAR_SAMPLE_PACKET_SIZE - 1);
                        sample_idx = LIDAR_SAMPLE_PACKET_SIZE - 1;
                    }
                }
                break;
            }
        }
    }
    
    // ????
    if (processed_bytes > 1000) {
        processed_bytes = 0;
    }
}

// C1 ????(????3-1:??????)
void LidarTimeoutRecovery(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t time_since_rx = current_time - lastLidarRxTime;
    
    if (time_since_rx > LIDAR_TIMEOUT_THRESHOLD) {
        // 1. ??STOP??
        uint8_t stop_cmd[] = {0xA5, 0x25};
        HAL_UART_Transmit(&huart6, stop_cmd, sizeof(stop_cmd), 100);
        HAL_Delay(LIDAR_STOP_DELAY);

        // 2. ??RESET??
        uint8_t reset_cmd[] = {0xA5, 0x40};
        HAL_UART_Transmit(&huart6, reset_cmd, sizeof(reset_cmd), 100);
        HAL_Delay(LIDAR_RESET_DELAY);

        // 3. ??SCAN??
        uint8_t scan_cmd[] = {0xA5, 0x20};
        HAL_UART_Transmit(&huart6, scan_cmd, sizeof(scan_cmd), 100);
        HAL_Delay(50);

        // 4. ????
        lidar_state = LIDAR_STATE_WAIT_START_RESP;
        lidar_start_resp_idx = 0;
        lastLidarRxTime = current_time;
    }
}

// ?????????? - ??????
void SendLidarToBluetooth(float angle, float distance, uint8_t quality)
{
    char buffer[24];  // ?32?????24??
    // ????:L:Axx.xx,Dxxxx.x,Qx
    int len = snprintf(buffer, sizeof(buffer), "D:%.0f,A:%.1f,Q:%d\r\n", distance, angle, quality);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 10);
}

/* ------------------ MPU6500 ?? (????) ------------------ */
uint8_t MPU6500_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check, data;
    HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, 0x75, 1, &check, 1, 100);
    if (check != 0x70 && check != 0x68) return 1;

    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, 0x6B, 1, &data, 1, 100);

    return 0;
}

void MPU6500_Calibrate(I2C_HandleTypeDef *hi2c, MPU6500_Calib *calib) {
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    MPU6500_Data data;
    for (int i = 0; i < 100; i++) {
        MPU6500_Read_All(hi2c, &data);
        ax_sum += data.Accel_X;
        ay_sum += data.Accel_Y;
        az_sum += data.Accel_Z;
        gx_sum += data.Gyro_X;
        gy_sum += data.Gyro_Y;
        gz_sum += data.Gyro_Z;
        HAL_Delay(10);
    }
    calib->ax_offset = ax_sum / 100;
    calib->ay_offset = ay_sum / 100;
    calib->az_offset = az_sum / 100 - ACCEL_SENS;
    calib->gx_offset = gx_sum / 100;
    calib->gy_offset = gy_sum / 100;
    calib->gz_offset = gz_sum / 100;
    calib->velocity_x = 0.0f;
    calib->distance_x = 0.0f;
    calib->dt = 0.1f;
    calib->calibrated = 1;
    calib->accel_x_filt = 0.0f;
    calib->accel_y_filt = 0.0f;
    calib->accel_z_filt = 0.0f;
    calib->gyro_x_filt = 0.0f;
    calib->gyro_y_filt = 0.0f;
    calib->gyro_z_filt = 0.0f;
}

uint8_t MPU6500_Read_All(I2C_HandleTypeDef *hi2c, MPU6500_Data *data) {
    uint8_t buf[14];
    if (HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, MPU6500_ACCEL_REG, 1, buf, 14, 100) != HAL_OK)
        return 1;

    data->Accel_X = (int16_t)(buf[0] << 8 | buf[1]);
    data->Accel_Y = (int16_t)(buf[2] << 8 | buf[3]);
    data->Accel_Z = (int16_t)(buf[4] << 8 | buf[5]);
    data->Gyro_X  = (int16_t)(buf[8] << 8 | buf[9]);
    data->Gyro_Y  = (int16_t)(buf[10] << 8 | buf[11]);
    data->Gyro_Z  = (int16_t)(buf[12] << 8 | buf[13]);
    return 0;
}

void MPU6500_GetCalibratedData(MPU6500_Data *raw, MPU6500_Calib *calib, MPU6500_Data *out) {
    out->Accel_X = raw->Accel_X - calib->ax_offset;
    out->Accel_Y = raw->Accel_Y - calib->ay_offset;
    out->Accel_Z = raw->Accel_Z - calib->az_offset;
    out->Gyro_X  = raw->Gyro_X - calib->gx_offset;
    out->Gyro_Y  = raw->Gyro_Y - calib->gy_offset;
    out->Gyro_Z  = raw->Gyro_Z - calib->gz_offset;
}

void MPU6500_Convert_Unit(MPU6500_Data *data) {
    data->Accel_X_mps2 = ((float)data->Accel_X / ACCEL_SENS) * G;
    data->Accel_Y_mps2 = ((float)data->Accel_Y / ACCEL_SENS) * G;
    data->Accel_Z_mps2 = ((float)data->Accel_Z / ACCEL_SENS) * G;
    data->Gyro_X_dps = (float)data->Gyro_X / GYRO_SENS;
    data->Gyro_Y_dps = (float)data->Gyro_Y / GYRO_SENS;
    data->Gyro_Z_dps = (float)data->Gyro_Z / GYRO_SENS;
}

void MPU6500_LowPassFilter(MPU6500_Data *data, MPU6500_Calib *calib, float alpha) {
    calib->accel_x_filt = alpha * data->Accel_X_mps2 + (1.0f - alpha) * calib->accel_x_filt;
    calib->accel_y_filt = alpha * data->Accel_Y_mps2 + (1.0f - alpha) * calib->accel_y_filt;
    calib->accel_z_filt = alpha * data->Accel_Z_mps2 + (1.0f - alpha) * calib->accel_z_filt;
    calib->gyro_x_filt = alpha * data->Gyro_X_dps + (1.0f - alpha) * calib->gyro_x_filt;
    calib->gyro_y_filt = alpha * data->Gyro_Y_dps + (1.0f - alpha) * calib->gyro_y_filt;
    calib->gyro_z_filt = alpha * data->Gyro_Z_dps + (1.0f - alpha) * calib->gyro_z_filt;
}

void MPU6500_Update_XMotion(MPU6500_Data *calib_data, MPU6500_Calib *calib) {
    float accel_x = calib->accel_x_filt;
    calib->velocity_x += accel_x * calib->dt;
    calib->distance_x += calib->velocity_x * calib->dt;
}

// ????MPU?????? - ??????
void SendMPUDataToBluetooth(MPU6500_Data *data, MPU6500_Calib *calib)
{
    char buffer[64];  // ?128?????64??
    // ????:M:AXx.xx,AYx.xx,AZx.xx,GXx.xx,GYx.xx,GZx.xx,VXx.xx,DXx.xx
    int len = snprintf(buffer, sizeof(buffer), 
        "M:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
        data->Accel_X_mps2, data->Accel_Y_mps2, data->Accel_Z_mps2,
        data->Gyro_X_dps, data->Gyro_Y_dps, data->Gyro_Z_dps,
        calib->velocity_x, calib->distance_x);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 10);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();  // ???USART1????(115200bps)
  MX_USART6_UART_Init();  // ???USART6??LIDAR C1(460800bps,????2-8)
  MX_I2C1_Init();         // ???I2C1??MPU6500
  /* USER CODE BEGIN 2 */
  // ??PWM?Encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // ?????????
  HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);

  // ???RPLIDAR C1(????4-2/4-4:?STOP?SCAN)
  HAL_Delay(100);  // ?????????
  uint8_t lidar_stop_cmd[] = {0xA5, 0x25};  // STOP??(????)
  HAL_UART_Transmit(&huart6, lidar_stop_cmd, sizeof(lidar_stop_cmd), 100);
  HAL_Delay(LIDAR_STOP_DELAY);  // ??????

  uint8_t lidar_scan_cmd[] = {0xA5, 0x20};  // SCAN??(????,????4-4)
  HAL_UART_Transmit(&huart6, lidar_scan_cmd, sizeof(lidar_scan_cmd), 100);
  HAL_Delay(50);

  // ??C1 DMA??
  HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);
  lidar_state = LIDAR_STATE_WAIT_START_RESP;  // ???C1??
  lidar_start_resp_idx = 0;
  lastLidarRxTime = HAL_GetTick();

  // ????????
  HAL_UART_Transmit(&huart1, (uint8_t*)"[INIT] Checking LIDAR communication...\r\n", 38, 100);
  uint8_t get_info_cmd[] = {0xA5, 0x90};
  HAL_UART_Transmit(&huart6, get_info_cmd, sizeof(get_info_cmd), 100);
  HAL_Delay(100);
  uint32_t initial_data = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
  if (initial_data > 0) {
      char msg[60];
      snprintf(msg, sizeof(msg), "[INIT] LIDAR responded with %lu bytes\r\n", initial_data);
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  } else {
      HAL_UART_Transmit(&huart1, (uint8_t*)"[INIT] LIDAR no response\r\n", 25, 100);
  }

  // ???????
  HAL_UART_Transmit(&huart1, (uint8_t*)"System Started\r\n", 16, 100);

  // ???MPU6500
  mpu_init_ok = MPU6500_Init(&hi2c1);
  if(mpu_init_ok == 0) {
      MPU6500_Calibrate(&hi2c1, &mpu_calib);
  }

  // ?????????
  lastEncoderA = __HAL_TIM_GET_COUNTER(&htim2);
  lastEncoderB = __HAL_TIM_GET_COUNTER(&htim4);
  
  // ???????
  uint32_t currentTime = HAL_GetTick();
  lastLidarProcessTime = currentTime;
  lastSensorSendTime = currentTime;
  lastEncoderReadTime = currentTime;
  lastMPUReadTime = currentTime;
  last_speed_update_time = currentTime;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t currentTime = HAL_GetTick();
    
    SendConnectionNotification();          // ??????
    UpdateSpeedRamp();                     // ??????(30ms??)
    LidarTimeoutRecovery();                // C1???????

    // ?5ms????LIDAR C1??(????4-10:????5KHz,5ms??25??)
    if(currentTime - lastLidarProcessTime >= 5) {
        lastLidarProcessTime = currentTime;
        ProcessLidarDataFromDMA();
    }

    // ?20ms??????ADC(?50ms??20ms)
    if(currentTime - lastEncoderReadTime >= 20) {
        lastEncoderReadTime = currentTime;
        
        // ??????
        int32_t encoderA = __HAL_TIM_GET_COUNTER(&htim2);
        int32_t encoderB = __HAL_TIM_GET_COUNTER(&htim4);
        int32_t deltaA = encoderA - lastEncoderA;
        int32_t deltaB = encoderB - lastEncoderB;
        lastEncoderA = encoderA;
        lastEncoderB = encoderB;

        // ??RPM(20ms=0.02s)
        float rpmA = (float)deltaA / PPR * (60.0f / 0.02f);
        float rpmB = (float)deltaB / PPR * (60.0f / 0.02f);
        snprintf(uart_buf, sizeof(uart_buf), "E:%.1f,%.1f\r\n", rpmA, rpmB);
        HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 10);

        // ??ADC??
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
        snprintf(uart_buf, sizeof(uart_buf), "A:%d\r\n", adc_val);
        HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 10);
    }

    // ?50ms??MPU6500??(?100ms??50ms)
    if(currentTime - lastMPUReadTime >= 50) {
        lastMPUReadTime = currentTime;
        
        // ??MPU6500??
        if(mpu_init_ok == 0 && mpu_calib.calibrated) {
            MPU6500_Data raw_data, calib_data;
            if(MPU6500_Read_All(&hi2c1, &raw_data) == 0) {
                MPU6500_GetCalibratedData(&raw_data, &mpu_calib, &calib_data);
                MPU6500_Convert_Unit(&calib_data);
                MPU6500_LowPassFilter(&calib_data, &mpu_calib, 0.3f);
                MPU6500_Update_XMotion(&calib_data, &mpu_calib);
                SendMPUDataToBluetooth(&calib_data, &mpu_calib);
            }
        }
    }

    // ?????CPU??
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */