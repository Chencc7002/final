#ifndef __RPLIDAR_H
#define __RPLIDAR_H

#include "main.h"
#include <stdbool.h>

/* ------------------ RPLIDAR C1 相关定义 ------------------ */
#define DMA_BUFFER_SIZE 256
#define LIDAR_SAMPLE_PACKET_SIZE 5
#define LIDAR_START_RESP_SIZE 7
#define ANGLE_FILTER_THRESHOLD 0.1f
#define MIN_VALID_DISTANCE 50.0f
#define MAX_VALID_DISTANCE 12000.0f
#define LIDAR_TIMEOUT_THRESHOLD 3000
#define LIDAR_STOP_DELAY 10
#define LIDAR_RESET_DELAY 500

// RPLIDAR C1状态机
typedef enum {
    LIDAR_STATE_WAIT_START_RESP,
    LIDAR_STATE_WAIT_SAMPLE_DATA
} LidarState;

// RPLIDAR C1采样数据包
typedef struct {
    uint8_t sync_quality;
    uint16_t angle_q6;
    uint16_t distance_q2;
} __attribute__((packed)) LidarSamplePacket;

// RPLIDAR C1启动响应包
typedef struct {
    uint8_t start_flag1;
    uint8_t start_flag2;
    uint8_t resp_len[3];
    uint8_t resp_mode;
    uint8_t data_type;
} __attribute__((packed)) LidarStartRespPacket;

void ProcessLidarDataFromDMA(void);
void LidarTimeoutRecovery(void);
void SendLidarToBluetooth(float angle, float distance, uint8_t quality);

#endif /* __RPLIDAR_H */