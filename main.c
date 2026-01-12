/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : KLS6030HC Motor Driver CAN-Bus Data Reader
 * @description    : Reads motor data from KLS6030HC motor driver via CAN-Bus
 *                   and transmits the data over UART for monitoring
 * @board          : STM33F407V Discovery
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>  // Required for sprintf
#include <string.h> // Required for strlen

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief Structure to hold KLS6030HC motor data
 */
typedef struct {
  uint16_t rpm;       // Motor RPM value
  float speed_kmh;    // Calculated vehicle speed in km/h
  uint16_t current;   // Motor current (if available)
  uint16_t voltage;   // Battery voltage (if available)
  uint8_t error_code; // Error code from motor controller
  uint8_t data_valid; // Flag indicating if data is valid
} KLS_MotorData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* KLS6030HC CAN Message IDs (Extended Frame Format - 29-bit) */
#define KLS_MESSAGE1_ID 0x0CF11E05 // RPM and Current data
#define KLS_MESSAGE2_ID 0x0CF11F05 // Voltage and Temperature data (if needed)

/* Vehicle Configuration */
#define WHEEL_DIAMETER_METERS 0.56f // Wheel diameter in meters
#define GEAR_RATIO 4.0f             // Motor to wheel gear ratio
#define PI_VALUE 3.14159265f        // Pi constant

/* CAN Configuration */
#define CAN_TIMEOUT_MS 100 // CAN message timeout in milliseconds

/* UART Buffer Size */
#define UART_BUFFER_SIZE 128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* CAN Receive Variables */
CAN_RxHeaderTypeDef rxHeader; // Received message header (ID, DLC, etc.)
uint8_t rxData[8];            // Received data payload (max 8 bytes)

/* Motor Data Storage */
KLS_MotorData_t motorData = {0}; // Global motor data structure

/* Data Reception Flag (for interrupt mode) */
volatile uint8_t canDataReceived = 0;

/* UART Transmit Buffer */
char uartTxBuffer[UART_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* CAN Functions */
void CAN_Filter_Config(void);                 // CAN filter configuration
void CAN_ProcessReceivedMessage(void);        // Process received CAN message
HAL_StatusTypeDef CAN_EnableInterrupts(void); // Enable CAN interrupts

/* Conversion Functions */
float ConvertRpmToKmh(float motorRpm); // Convert motor RPM to vehicle speed

/* Utility Functions */
void UART_SendString(const char *str); // Send string over UART
void LED_ErrorIndication(void);        // Indicate error with onboard LED

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Converts motor RPM to vehicle speed in km/h
 * @param  motorRpm: Motor RPM value from KLS6030HC
 * @retval Vehicle speed in km/h
 * @note   Formula: Speed = (Motor_RPM / Gear_Ratio) * Wheel_Circumference * 60
 * / 1000
 */
float ConvertRpmToKmh(float motorRpm) {
  /* Calculate wheel RPM considering gear ratio */
  float wheelRpm = motorRpm / GEAR_RATIO;

  /* Calculate wheel circumference in meters */
  float wheelCircumference = PI_VALUE * WHEEL_DIAMETER_METERS;

  /* Calculate speed in meters per minute */
  float speedMetersPerMinute = wheelRpm * wheelCircumference;

  /* Convert to km/h (m/min * 60/1000 = m/min * 0.06) */
  float speedKmh = speedMetersPerMinute * 0.06f;

  return speedKmh;
}

/**
 * @brief  Sends a string over UART
 * @param  str: Null-terminated string to send
 * @retval None
 */
void UART_SendString(const char *str) {
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), CAN_TIMEOUT_MS);
}

/**
 * @brief  Indicates error using onboard LEDs (Discovery board has 4 LEDs)
 * @retval None
 * @note   LED pins: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *         This function toggles the Red LED (PD14) for error indication
 */
void LED_ErrorIndication(void) {
  /* Toggle error LED - requires GPIO configuration for PD14 */
  /* HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); */
}

/**
 * @brief  Configures CAN filter for receiving KLS6030HC messages
 * @retval None
 * @note   Current configuration accepts ALL CAN messages (promiscuous mode)
 *         For production, configure specific filter for KLS_MESSAGE1_ID
 */
void CAN_Filter_Config(void) {
  CAN_FilterTypeDef canFilterConfig;

  /* Basic filter configuration */
  canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  canFilterConfig.FilterBank = 0;                      // Use filter bank 0
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO 0
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // Mask mode
  canFilterConfig.FilterScale =
      CAN_FILTERSCALE_32BIT; // 32-bit scale for extended ID

  /*
   * Promiscuous Mode Configuration (Accept ALL messages)
   * Setting both ID and Mask to 0 accepts all CAN messages.
   * This is useful for debugging and initial development.
   *
   * For production, you should configure the filter to only accept
   * the specific KLS6030HC message ID (0x0CF11E05):
   *
   * Extended ID format in filter registers:
   * FilterIdHigh = (ID >> 13) & 0xFFFF
   * FilterIdLow  = ((ID << 3) & 0xFFF8) | CAN_ID_EXT
   */
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;

  /* Apply filter configuration */
  if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  Enables CAN receive interrupts
 * @retval HAL status
 * @note   Enables interrupt for FIFO0 message pending
 */
HAL_StatusTypeDef CAN_EnableInterrupts(void) {
  return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief  Processes received CAN message and extracts motor data
 * @retval None
 * @note   Call this function after receiving a CAN message
 */
void CAN_ProcessReceivedMessage(void) {
  /* Check if message is in Extended ID format (29-bit) */
  if (rxHeader.IDE == CAN_ID_EXT) {
    /* Check if the message is from KLS6030HC (Message 1: RPM and Current) */
    if (rxHeader.ExtId == KLS_MESSAGE1_ID) {
      /*
       * KLS6030HC Message 1 Format (0x0CF11E05):
       * Byte 0: RPM Low Byte
       * Byte 1: RPM High Byte
       * Byte 2: Current Low Byte
       * Byte 3: Current High Byte
       * Byte 4-7: Reserved/Other data
       */

      /* Extract RPM (Little Endian: LSB first) */
      uint8_t rpmLSB = rxData[0];
      uint8_t rpmMSB = rxData[1];
      motorData.rpm = (uint16_t)((rpmMSB << 8) | rpmLSB);

      /* Extract Current if available (Little Endian) */
      uint8_t currentLSB = rxData[2];
      uint8_t currentMSB = rxData[3];
      motorData.current = (uint16_t)((currentMSB << 8) | currentLSB);

      /* Calculate vehicle speed from RPM */
      motorData.speed_kmh = ConvertRpmToKmh((float)motorData.rpm);

      /* Mark data as valid */
      motorData.data_valid = 1;
    }
  }
}

/**
 * @brief  CAN RX FIFO0 message pending callback (Interrupt mode)
 * @param  hcan: CAN handle pointer
 * @retval None
 * @note   This callback is called when a message is received in FIFO0
 *         Uncomment the interrupt enable in main() to use this
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN1) {
    /* Read the received message */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
      /* Set flag for main loop processing */
      canDataReceived = 1;
    }
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* Configure CAN filters */
  CAN_Filter_Config();

  /* Start CAN module */
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    /* Enter error handler if CAN start fails */
    Error_Handler();
  }

  /*
   * Optional: Enable CAN receive interrupts for interrupt-driven mode
   * Uncomment the following lines to use interrupt mode instead of polling
   */
  /*
  if (CAN_EnableInterrupts() != HAL_OK)
  {
      Error_Handler();
  }
  */

  /* Send startup message over UART */
  UART_SendString("\r\n========================================\r\n");
  UART_SendString("  KLS6030HC CAN-Bus Data Reader\r\n");
  UART_SendString("  STM32F407V Discovery Board\r\n");
  UART_SendString("========================================\r\n");
  UART_SendString("Waiting for CAN messages...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* ============================================================
     * Polling Method: Check for CAN messages in the main loop
     * This is simpler but less efficient than interrupt mode
     * ============================================================ */

    /* Check if there are messages waiting in FIFO 0 */
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
      /* Read the message from FIFO 0 */
      if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) ==
          HAL_OK) {
        /* Process the received message */
        CAN_ProcessReceivedMessage();

        /* If valid motor data was received, send it over UART */
        if (motorData.data_valid) {
          /* Format and send the data */
          snprintf(uartTxBuffer, UART_BUFFER_SIZE,
                   "RPM: %5d | Speed: %3d km/h | Current: %4d A\r\n",
                   motorData.rpm, (int)motorData.speed_kmh, motorData.current);

          UART_SendString(uartTxBuffer);

          /* Reset valid flag after processing */
          motorData.data_valid = 0;
        }
      }
    }

    /*
     * Optional: Process interrupt-received data
     * Uncomment if using interrupt mode
     */
    /*
    if (canDataReceived)
    {
        CAN_ProcessReceivedMessage();

        if (motorData.data_valid)
        {
            snprintf(uartTxBuffer, UART_BUFFER_SIZE,
                     "RPM: %5d | Speed: %3d km/h | Current: %4d A\r\n",
                     motorData.rpm,
                     (int)motorData.speed_kmh,
                     motorData.current);

            UART_SendString(uartTxBuffer);
            motorData.data_valid = 0;
        }

        canDataReceived = 0;
    }
    */

    /* Small delay to prevent busy-waiting (adjust as needed) */
    HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 * @note   Using HSI (16MHz) as system clock source
 *         For better CAN timing accuracy, consider using HSE oscillator
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 * @note   CAN Baud Rate Calculation:
 *         Baud Rate = APB1_CLK / (Prescaler * (SyncJumpWidth + TimeSeg1 +
 * TimeSeg2))
 *
 *         Current Configuration (HSI = 16MHz, APB1 = 16MHz):
 *         - Prescaler = 16
 *         - Timing: 1 + 1 + 2 = 4 TQ
 *         - Baud Rate = 16MHz / (16 * 4) = 250 kbps
 *
 *         For 500 kbps: Use Prescaler = 8, or adjust timing
 *         For 125 kbps: Use Prescaler = 32
 *
 *         Verify KLS6030HC baud rate in product documentation!
 */
static void MX_CAN1_Init(void) {
  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;              // Baud rate prescaler
  hcan1.Init.Mode = CAN_MODE_NORMAL;      // Normal operating mode
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ; // Synchronization jump width
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;      // Time segment 1
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;      // Time segment 2
  hcan1.Init.TimeTriggeredMode = DISABLE; // Time-triggered communication
  hcan1.Init.AutoBusOff = ENABLE;         // Enable automatic bus-off recovery
  hcan1.Init.AutoWakeUp = DISABLE;        // Automatic wakeup mode
  hcan1.Init.AutoRetransmission = ENABLE; // Enable automatic retransmission
  hcan1.Init.ReceiveFifoLocked = DISABLE; // Receive FIFO locked mode
  hcan1.Init.TransmitFifoPriority = DISABLE; // Transmit FIFO priority

  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 * @note   USART2 is connected to ST-LINK Virtual COM Port on Discovery board
 *         Use PA2 (TX) and PA3 (RX)
 */
static void MX_USART2_UART_Init(void) {
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 * @note   Configure GPIO pins for CAN, UART, and optional LED indicators
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE(); // Enable for onboard LEDs

  /* Configure GPIO pin Output Level for LEDs (optional) */
  HAL_GPIO_WritePin(GPIOD,
                    GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                    GPIO_PIN_RESET);

  /* Configure Discovery Board LEDs: PD12(Green), PD13(Orange), PD14(Red),
   * PD15(Blue) */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 * @note   In debug mode, a breakpoint can be set here to identify errors
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add custom error handling implementation here */

  /* Disable interrupts to prevent further execution */
  __disable_irq();

  /* Turn on Red LED to indicate error (PD14 on Discovery board) */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  /* Send error message over UART if possible */
  /* Note: This may not work if UART is the source of error */

  /* Infinite loop - system halted */
  while (1) {
    /* Toggle Red LED to indicate error visually */
    /* This won't work since interrupts are disabled */
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add custom implementation to report the file name and line number,
     Example: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  char assertBuffer[100];
  snprintf(assertBuffer, sizeof(assertBuffer),
           "Assert failed: file %s on line %lu\r\n", (char *)file, line);
  UART_SendString(assertBuffer);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
