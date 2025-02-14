/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CANSPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
// Define structures for CAN messages
uCAN_MSG rxMessage;
uCAN_MSG txMessage;
/* USER CODE BEGIN PV */

typedef struct {
    uint32_t last_heartbeat;
    uint8_t active;
} DeviceStatus;

#define NUM_HEARTBEATS 4
#define NUM_THRUSTERS 6
#define NUM_ARMS 4
#define CAN_ID_HEARTBEAT_BASE 0x200
#define CAN_ID_THRUSTER_CURRENT_BASE 0x300
#define CAN_ID_THRUSTER_PWM_BASE 0x310
#define CAN_ID_IMU 0x400
#define CAN_ID_ARM_BASE 0x500
#define CAN_ID_DEPTH 0x600
#define CAN_ID_INDICATOR 0x700

DeviceStatus heartbeats[NUM_HEARTBEATS];

void process_heartbeat(uint8_t device_id) {
    if (device_id < NUM_HEARTBEATS) {
        heartbeats[device_id].last_heartbeat = HAL_GetTick();
        heartbeats[device_id].active = 1;
    }
}

void check_heartbeats() {
    uint32_t current_time = HAL_GetTick();
    for (int i = 0; i < NUM_HEARTBEATS; i++) {
        if (heartbeats[i].active && (current_time - heartbeats[i].last_heartbeat > 5000)) {
            heartbeats[i].active = 0;
            log_error("Heartbeat lost: Device %d", i);
        }
    }
}

void process_thruster_current(uint8_t thruster_id, uint8_t data[8]) {
    if (thruster_id < NUM_THRUSTERS) {
        log_info("Thruster %d current: %d", thruster_id, data[0]);
    }
}

void process_thruster_pwm(uint8_t thruster_id, uint8_t data[8]) {
    if (thruster_id < NUM_THRUSTERS) {
        log_info("Thruster %d PWM: %d", thruster_id, data[0]);
    }
}

void process_imu(uint8_t data[8]) {
    log_info("IMU Data - Roll: %d, Pitch: %d, Yaw: %d, AccX: %d, AccY: %d, AccZ: %d",
             data[0], data[1], data[2], data[3], data[4], data[5]);
}

void process_arm(uint8_t arm_id, uint8_t data[8]) {
    if (arm_id < NUM_ARMS) {
        log_info("Arm %d feedback: %d", arm_id, data[0]);
    }
}

void process_depth(uint8_t data[8]) {
    log_info("Depth sensor reading: %d", data[0]);
}

void process_indicator(uint8_t data[8]) {
    log_info("Indicator status: %d", data[0]);
}

void CAN_Receive_Callback() {
    if (CANSPI_Receive(&rxMessage)) {
        uint8_t data[8] = {
            rxMessage.frame.data0, rxMessage.frame.data1, rxMessage.frame.data2, rxMessage.frame.data3,
            rxMessage.frame.data4, rxMessage.frame.data5, rxMessage.frame.data6, rxMessage.frame.data7
        };

        uint32_t id = rxMessage.frame.id;

        if (id >= CAN_ID_HEARTBEAT_BASE && id < CAN_ID_HEARTBEAT_BASE + NUM_HEARTBEATS) {
            process_heartbeat(id - CAN_ID_HEARTBEAT_BASE);
        } else if (id >= CAN_ID_THRUSTER_CURRENT_BASE && id < CAN_ID_THRUSTER_CURRENT_BASE + NUM_THRUSTERS) {
            process_thruster_current(id - CAN_ID_THRUSTER_CURRENT_BASE, data);
        } else if (id >= CAN_ID_THRUSTER_PWM_BASE && id < CAN_ID_THRUSTER_PWM_BASE + NUM_THRUSTERS) {
            process_thruster_pwm(id - CAN_ID_THRUSTER_PWM_BASE, data);
        } else if (id == CAN_ID_IMU) {
            process_imu(data);
        } else if (id >= CAN_ID_ARM_BASE && id < CAN_ID_ARM_BASE + NUM_ARMS) {
            process_arm(id - CAN_ID_ARM_BASE, data);
        } else if (id == CAN_ID_DEPTH) {
            process_depth(data);
        } else if (id == CAN_ID_INDICATOR) {
            process_indicator(data);
        } else {
            log_warning("Unknown CAN message ID: %lx", id);
        }
    }
}

void log_error(const char *format, ...) {
    // Implement logging mechanism (UART, file, etc.)
}

void log_warning(const char *format, ...) {
    // Implement warning mechanism
}

void log_info(const char *format, ...) {
    // Implement info logging
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

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

		HAL_Init();
	    SystemClock_Config();
	    MX_GPIO_Init();
	    MX_SPI1_Init();
	    CANSPI_Initialize();

	    while (1) {
	        CAN_Receive_Callback();
	        check_heartbeats();
	        HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAN_CS_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
