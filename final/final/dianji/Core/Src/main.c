/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序 + ADC + 编码器 + RPLIDAR C1 + MPU6500 传感器
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
#include "motor_control.h"
#include "rplidar.h"
#include "mpu6500.h"
#include "bluetooth.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/* Private variables ---------------------------------------------------------*/
/* ------------------ 运动控制 ------------------ */
uint8_t target_direction = DIR_STOP;
uint8_t current_direction = DIR_STOP;
uint32_t current_speed = 0;
uint32_t target_speed = 0;
uint32_t last_speed_update_time = 0;
uint32_t turn_start_time = 0;
uint8_t turn_direction = DIR_LEFT;
uint8_t is_turning = 0;
uint8_t turn_state = 0;

/* ------------------ 蓝牙 ------------------ */
uint8_t bluetooth_rx_data = 0;
uint8_t bluetooth_connected = 0;
uint8_t connection_announced = 0;
uint8_t connection_msg[] = "Connected\r\n";

/* ------------------ RPLIDAR C1 ------------------ */
uint8_t lidar_dma_buffer[DMA_BUFFER_SIZE];
uint32_t lidar_rxIndex = 0;
float lastLidarAngle = 0.0f;
uint32_t lastLidarRxTime = 0;
LidarState lidar_state = LIDAR_STATE_WAIT_START_RESP;
uint8_t lidar_start_resp_buf[LIDAR_START_RESP_SIZE];
uint8_t lidar_start_resp_idx = 0;

/* ------------------ MPU6500 ------------------ */
MPU6500_Data mpu_data;
MPU6500_Calib mpu_calib = {0};
uint8_t mpu_init_ok = 0;

/* ------------------ 其他 ------------------ */
int32_t lastEncoderA = 0;
int32_t lastEncoderB = 0;
char uart_buf[100];
uint32_t lastLidarProcessTime = 0;
uint32_t lastSensorSendTime = 0;
uint32_t lastEncoderReadTime = 0;
uint32_t lastMPUReadTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        HandleBluetoothRx();
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // 启动PWM和编码器
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // 启动蓝牙接收
  HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);

  // 初始化RPLIDAR C1
  HAL_Delay(100);
  uint8_t lidar_stop_cmd[] = {0xA5, 0x25};
  HAL_UART_Transmit(&huart6, lidar_stop_cmd, sizeof(lidar_stop_cmd), 100);
  HAL_Delay(LIDAR_STOP_DELAY);

  uint8_t lidar_scan_cmd[] = {0xA5, 0x20};
  HAL_UART_Transmit(&huart6, lidar_scan_cmd, sizeof(lidar_scan_cmd), 100);
  HAL_Delay(50);

  // 启动C1 DMA接收
  HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);
  lidar_state = LIDAR_STATE_WAIT_START_RESP;
  lidar_start_resp_idx = 0;
  lastLidarRxTime = HAL_GetTick();

  // 检查LIDAR通信
  HAL_UART_Transmit(&huart1, (uint8_t*)"[INIT] Checking LIDAR communication...\r\n", 38, 100);
  uint8_t get_info_cmd[] = {0xA5, 0x90};
  HAL_UART_Transmit(&huart6, get_info_cmd, sizeof(get_info_cmd), 100);
  HAL_Delay(100);
  uint32_t initial_data = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
  if (initial_data > 0) {
      char msg[60];
      snprintf(msg, sizeof(msg), "[INIT] LIDAR responded with %u bytes\r\n", initial_data);
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  } else {
      HAL_UART_Transmit(&huart1, (uint8_t*)"[INIT] LIDAR no response\r\n", 25, 100);
  }

  // 系统启动消息
  HAL_UART_Transmit(&huart1, (uint8_t*)"System Started\r\n", 16, 100);

  // 初始化MPU6500
  mpu_init_ok = MPU6500_Init(&hi2c1);
  if(mpu_init_ok == 0) {
      MPU6500_Calibrate(&hi2c1, &mpu_calib);
  }

  // 初始化编码器读数
  lastEncoderA = __HAL_TIM_GET_COUNTER(&htim2);
  lastEncoderB = __HAL_TIM_GET_COUNTER(&htim4);
  
  // 初始化时间戳
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
    
    SendConnectionNotification();
    UpdateSpeedRamp();
    LidarTimeoutRecovery();

    // 处理LIDAR数据
    if(currentTime - lastLidarProcessTime >= 5) {
        lastLidarProcessTime = currentTime;
        ProcessLidarDataFromDMA();
    }

    // 读取编码器和ADC
    if(currentTime - lastEncoderReadTime >= 20) {
        lastEncoderReadTime = currentTime;
        
        int32_t encoderA = __HAL_TIM_GET_COUNTER(&htim2);
        int32_t encoderB = __HAL_TIM_GET_COUNTER(&htim4);
        int32_t deltaA = encoderA - lastEncoderA;
        int32_t deltaB = encoderB - lastEncoderB;
        lastEncoderA = encoderA;
        lastEncoderB = encoderB;

        float rpmA = (float)deltaA / 360 * (60.0f / 0.02f);
        float rpmB = (float)deltaB / 360 * (60.0f / 0.02f);
        snprintf(uart_buf, sizeof(uart_buf), "E:%.1f,%.1f\r\n", rpmA, rpmB);
        HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 10);

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
        snprintf(uart_buf, sizeof(uart_buf), "A:%d\r\n", adc_val);
        HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 10);
    }

    // 读取MPU6500数据
    if(currentTime - lastMPUReadTime >= 50) {
        lastMPUReadTime = currentTime;
        
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
  * where the assert_param error has occurred.
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