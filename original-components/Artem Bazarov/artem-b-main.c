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

#define USER_BTN_PORT GPIOA
#define USER_BTN_PIN  GPIO_PIN_0

#define SYNC1 0xAC
#define SYNC2 0x53
#define MAX_DATA_SIZE 250

#define ACK_OK            0
#define ACK_BAD_ADDR      1
#define ACK_BAD_CRC       2
#define ACK_BAD_PARAM     3

#define MY_DEVICE_ADDR    0x01

UART_HandleTypeDef huart1;

typedef struct
{
    uint8_t SQN;
    uint8_t ADDR;
    uint8_t CODE;                  // код команды (1 байт)
    uint8_t PARAM[MAX_DATA_SIZE];  // параметры команды
    uint8_t PARAM_SIZE;            // размер параметров
    uint8_t CRC_OK;
} RX_PACKET_T;

RX_PACKET_T parsedPacket = {
    .SQN = 0x10,
    .ADDR = 0x01,
    .CODE = 0x01,
    .PARAM = {0x05},
    .PARAM_SIZE = 1,
    .CRC_OK = 1
};

typedef struct
{
    uint8_t  SQN;
    uint8_t  ADDR;
    uint8_t  ACK;
    uint8_t  DATA[MAX_DATA_SIZE];
    uint8_t  DATA_SIZE;
} TX_RESPONSE_T;

uint8_t CRC8_Calc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    uint8_t poly = 0x31;

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void buildPrd3Response(uint8_t *outBuf, uint16_t *outSize, const TX_RESPONSE_T *resp)
{
    uint16_t idx = 0;

    outBuf[idx++] = SYNC1;
    outBuf[idx++] = SYNC2;

    /* ---------- LEN ---------- */
    uint8_t dataSize = resp->DATA_SIZE;
    if (dataSize > MAX_DATA_SIZE)
        dataSize = MAX_DATA_SIZE;

    uint8_t len = 4 + dataSize;   // SQN + ADDR + ACK + DATA + CRC
    outBuf[idx++] = len;

    /* ---------- ТЕЛО ПАКЕТА ---------- */
    outBuf[idx++] = resp->SQN;
    outBuf[idx++] = resp->ADDR;
    outBuf[idx++] = resp->ACK;

    for (uint8_t i = 0; i < dataSize; i++)
        outBuf[idx++] = resp->DATA[i];

    /* ---------- CRC ---------- */
    uint8_t crc = CRC8_Calc(&outBuf[2], len - 1);
    outBuf[idx++] = crc;

    *outSize = idx;
}


void sendResponsePacket(const TX_RESPONSE_T *resp)
{
    uint8_t txBuf[256];
    uint16_t size = 0;

    buildPrd3Response(txBuf, &size, resp);
    HAL_UART_Transmit(&huart1, txBuf, size, HAL_MAX_DELAY);


}

void handlePacket(void)
{
    TX_RESPONSE_T rsp = {0};

    rsp.SQN  = parsedPacket.SQN;   /* обязано совпадать */
    rsp.ADDR = 0x01;               /* адрес ПРД-3 */
    rsp.DATA_SIZE = 0;             /* без телеметрии */

    // 1. Проверка CRC
    if (!parsedPacket.CRC_OK)
    {
        rsp.ACK = ACK_BAD_CRC;
        sendResponsePacket(&rsp);
        return;
    }
    // 2. Проверка адреса
    if (parsedPacket.ADDR != MY_DEVICE_ADDR)
    {
        rsp.ACK = ACK_BAD_ADDR;
        sendResponsePacket(&rsp);
        return;
    }
    /* 3. Проверка параметров команды
    if (!checkCommandParams(&parsedPacket))
    {
        rsp.ACK = ACK_BAD_PARAM;
        sendResponsePacket(&rsp);
        return;
    }
    rsp.ACK = ACK_OK;*/

    sendResponsePacket(&rsp);

}

void checkUserButtonAndSendPacket(void)
{
    static uint8_t prevState = GPIO_PIN_RESET;
    uint8_t currState;

    currState = HAL_GPIO_ReadPin(USER_BTN_PORT, USER_BTN_PIN);

    // Нажатие: было 0 → стало 1
    if (prevState == GPIO_PIN_RESET && currState == GPIO_PIN_SET)
    {
        handlePacket();   // ← отправка ответного пакета
    }

    prevState = currState;
}
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

char msg[] = "Hello its STM32\r\n";
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /*if (packetReady) {
      packetReady = 0;
      handlePacket();         // отправка ответного пакета
  }

  handlePacket();*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  checkUserButtonAndSendPacket();
	  HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : A_Pin */
  GPIO_InitStruct.Pin = A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A_GPIO_Port, &GPIO_InitStruct);

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
