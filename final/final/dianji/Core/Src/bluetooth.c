#include "bluetooth.h"
#include "usart.h"
#include "motor_control.h"
#include <string.h>

extern uint8_t bluetooth_rx_data;
extern uint8_t target_direction;
extern uint32_t target_speed;
extern uint8_t bluetooth_connected;
extern uint8_t connection_announced;
extern uint8_t connection_msg[];

/* 处理蓝牙接收 */
void HandleBluetoothRx(void)
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

/* 发送连接通知 */
void SendConnectionNotification(void)
{
    if (bluetooth_connected && !connection_announced) {
        HAL_UART_Transmit(&huart1, connection_msg, strlen((char*)connection_msg), 1000);
        connection_announced = 1;
    }
}