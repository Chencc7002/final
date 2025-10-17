#include "rplidar.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

extern uint8_t lidar_dma_buffer[DMA_BUFFER_SIZE];
extern uint32_t lidar_rxIndex;
extern float lastLidarAngle;
extern uint32_t lastLidarRxTime;
extern LidarState lidar_state;
extern uint8_t lidar_start_resp_buf[LIDAR_START_RESP_SIZE];
extern uint8_t lidar_start_resp_idx;

/* 处理LIDAR采样数据 */
bool ProcessLidarSampleData(const uint8_t *raw_buf)
{
    LidarSamplePacket pkt;
    memcpy(&pkt, raw_buf, sizeof(pkt));

    uint8_t S = pkt.sync_quality & 0x01;
    uint8_t not_S = (pkt.sync_quality >> 1) & 0x01;
    uint8_t quality = (pkt.sync_quality >> 2) & 0x3F;

    uint16_t raw_angle_q6 = pkt.angle_q6;
    uint16_t angle_field = (raw_angle_q6 >> 1) & 0x7FFF;
    float angle = (float)angle_field / 64.0f;

    uint16_t raw_distance_q2 = pkt.distance_q2;
    float distance = (float)raw_distance_q2 / 4.0f;

    if ((S ^ not_S) != 0x01) {
        return false;
    }

    if (quality == 0) return false;
    if (distance < MIN_VALID_DISTANCE || distance > MAX_VALID_DISTANCE) return false;

    if (fabsf(angle - lastLidarAngle) < ANGLE_FILTER_THRESHOLD && angle >= lastLidarAngle) {
        return false;
    }

    SendLidarToBluetooth(angle, distance, quality);
    lastLidarAngle = angle;
    return true;
}

/* 从DMA处理LIDAR数据 */
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
                if (lidar_start_resp_idx == 0) {
                    if (data == 0xA5) {
                        lidar_start_resp_buf[lidar_start_resp_idx++] = data;
                    }
                } 
                else if (lidar_start_resp_idx == 1) {
                    if (data == 0x5A) {
                        lidar_start_resp_buf[lidar_start_resp_idx++] = data;
                    } else {
                        lidar_start_resp_idx = 0;
                    }
                }
                else {
                    lidar_start_resp_buf[lidar_start_resp_idx++] = data;
                    
                    if (lidar_start_resp_idx >= LIDAR_START_RESP_SIZE) {
                        LidarStartRespPacket* resp = (LidarStartRespPacket*)lidar_start_resp_buf;
                        
                        if (resp->start_flag1 == 0xA5 && resp->start_flag2 == 0x5A) {
                            if (resp->data_type == 0x81 || resp->data_type == 0x82 || 
                                resp->data_type == 0x83 || resp->data_type == 0x84) {
                                lidar_state = LIDAR_STATE_WAIT_SAMPLE_DATA;
                                lidar_start_resp_idx = 0;
                            } else {
                                lidar_state = LIDAR_STATE_WAIT_SAMPLE_DATA;
                                lidar_start_resp_idx = 0;
                            }
                        } else {
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
                        
                        if (samples_valid % 50 == 0) {
                            char sample_msg[80];
                            snprintf(sample_msg, sizeof(sample_msg),
                                "[SAMPLE] Valid: %lu/%lu (%.1f%%)\r\n",
                                samples_valid, samples_processed, 
                                (float)samples_valid / samples_processed * 100.0f);
                            HAL_UART_Transmit(&huart1, (uint8_t*)sample_msg, strlen(sample_msg), 10);
                        }
                    } else {
                        memmove(sample_buf, sample_buf + 1, LIDAR_SAMPLE_PACKET_SIZE - 1);
                        sample_idx = LIDAR_SAMPLE_PACKET_SIZE - 1;
                    }
                }
                break;
            }
        }
    }
    
    if (processed_bytes > 1000) {
        processed_bytes = 0;
    }
}

/* LIDAR超时恢复 */
void LidarTimeoutRecovery(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t time_since_rx = current_time - lastLidarRxTime;
    
    if (time_since_rx > LIDAR_TIMEOUT_THRESHOLD) {
        uint8_t stop_cmd[] = {0xA5, 0x25};
        HAL_UART_Transmit(&huart6, stop_cmd, sizeof(stop_cmd), 100);
        HAL_Delay(LIDAR_STOP_DELAY);

        uint8_t reset_cmd[] = {0xA5, 0x40};
        HAL_UART_Transmit(&huart6, reset_cmd, sizeof(reset_cmd), 100);
        HAL_Delay(LIDAR_RESET_DELAY);

        uint8_t scan_cmd[] = {0xA5, 0x20};
        HAL_UART_Transmit(&huart6, scan_cmd, sizeof(scan_cmd), 100);
        HAL_Delay(50);

        lidar_state = LIDAR_STATE_WAIT_START_RESP;
        lidar_start_resp_idx = 0;
        lastLidarRxTime = current_time;
    }
}

/* 发送LIDAR数据到蓝牙 */
void SendLidarToBluetooth(float angle, float distance, uint8_t quality)
{
    char buffer[24];
    int len = snprintf(buffer, sizeof(buffer), "D:%.0f,A:%.1f,Q:%d\r\n", distance, angle, quality);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 10);
}