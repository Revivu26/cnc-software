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
#include <string.h>
#include <stdio.h>

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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

static uint8_t  uart_ring[UART_RING_SIZE];
static volatile uint16_t uart_head = 0;
static volatile uint16_t uart_tail = 0;
static uint8_t  uart_rx_byte;

static uint8_t fsm_state = 0;
/*
  0 = WAIT_SYNC1
  1 = WAIT_SYNC2
  2 = WAIT_LEN
  3 = WAIT_SEQ
  4 = WAIT_ADDR
  5 = WAIT_DATA
  6 = WAIT_CRC
*/

static uint8_t data_buf[MAX_PACKET_LEN];
static uint8_t data_cnt = 0;
static uint8_t data_len_expected = 0;

static uint8_t pkt_len_total = 0;
static uint8_t pkt_seq = 0;
static uint8_t pkt_addr = 0;
static uint8_t pkt_crc = 0;

static uint32_t last_byte_time = 0;
static const uint32_t PARSER_TIMEOUT_MS = 10;

static uint8_t command_ready = 0;
static uint8_t command_type = 0;
static char    command_text[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void parser_process_byte(uint8_t b);
void uart_poll_process(void);
void parser_check_timeout(void);
uint8_t crc8_calc(const uint8_t *data, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* putchar → printf по UART1 */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

  printf("\r\n\r\n");
  printf("=============================================\r\n");
  printf("STM32 UART Packet Parser Started\r\n");
  printf("System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
  printf("UART Baud Rate: %lu\r\n", (uint32_t)huart1.Init.BaudRate);
  printf("Waiting for packets...\r\n");
  printf("=============================================\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uart_poll_process();
	  parser_check_timeout();
  }
}
/* USER CODE END 3 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : A_Pin */
  GPIO_InitStruct.Pin = A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint16_t next = (uart_head + 1) % UART_RING_SIZE;
        if (next != uart_tail)
        {
            uart_ring[uart_head] = uart_rx_byte;
            uart_head = next;
        }

        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}
void uart_poll_process(void)
{
    uint8_t local_buf[UART_RING_SIZE];
    uint16_t count = 0;

    while (uart_tail != uart_head)
    {
        local_buf[count++] = uart_ring[uart_tail];
        uart_tail = (uart_tail + 1) % UART_RING_SIZE;
    }

    for (uint16_t i = 0; i < count; i++)
    {
        parser_process_byte(local_buf[i]);
    }
}
void parser_process_byte(uint8_t b)
{
	last_byte_time = HAL_GetTick();
    switch (fsm_state)
    {
        case 0: // WAIT_SYNC1
            if (b == SYNC1)
            {
                printf("SYNC1 OK\r\n");
                fsm_state = 1;
            }
            break;

        case 1: // WAIT_SYNC2
            if (b == SYNC2)
            {
                printf("SYNC2 OK\r\n");
                fsm_state = 2;
            }
            else
            {
                printf("SYNC2 FAIL (0x%02X)\r\n", b);
                fsm_state = 0;
            }
            break;

        case 2: // WAIT_LEN
            pkt_len_total = b;
            printf("LEN = %u\r\n", pkt_len_total);

            if (pkt_len_total < 4 || pkt_len_total > MAX_PACKET_LEN)
            {
                printf("LEN BAD\r\n");
                fsm_state = 0;
            }
            else
            {
                data_len_expected = pkt_len_total - 4; // LEN-SEQ-ADDR-CRC
                data_cnt = 0;

                printf("DATA_LEN = %u\r\n", data_len_expected);
                fsm_state = 3;
            }
            break;

        case 3: // WAIT_SEQ
            pkt_seq = b;
            printf("SEQ = 0x%02X\r\n", pkt_seq);
            fsm_state = 4;
            break;

        case 4: // WAIT_ADDR
            pkt_addr = b;
            printf("ADDR = 0x%02X\r\n", pkt_addr);

            if (data_len_expected == 0)
                fsm_state = 6;
            else
                fsm_state = 5;
            break;

        case 5: // WAIT_DATA
            data_buf[data_cnt++] = b;
            printf("DATA[%u] = 0x%02X\r\n", data_cnt - 1, b);

            if (data_cnt == data_len_expected)
                fsm_state = 6;
            break;

        case 6: // WAIT_CRC
            pkt_crc = b;
            printf("CRC = 0x%02X\r\n", pkt_crc);

            printf("\r\n======= PACKET RECEIVED =======\r\n");
            printf("SEQ:  0x%02X\r\n", pkt_seq);
            printf("ADDR: 0x%02X\r\n", pkt_addr);
            printf("DATA:");


            for (uint8_t i = 0; i < data_cnt; i++)
                printf(" %02X", data_buf[i]);

            printf("\r\n");
            uint8_t crc_buf[1 + 1 + 1 + data_cnt];
            uint8_t pos = 0;

            crc_buf[pos++] = pkt_len_total;
            crc_buf[pos++] = pkt_seq;
            crc_buf[pos++] = pkt_addr;

            for (uint8_t i = 0; i < data_cnt; i++)
                crc_buf[pos++] = data_buf[i];

            uint8_t crc_calc = crc8_calc(crc_buf, pos);

            printf("CRC CALC = 0x%02X\r\n", crc_calc);
            printf("CRC RECV = 0x%02X\r\n", pkt_crc);

            if (crc_calc == pkt_crc)
            {
                printf("CRC OK ✅\r\n");
            }
            else
            {
                printf("CRC FAIL ❌\r\n");
            }
            if (data_cnt >= 1)
            {
                uint8_t flag = data_buf[0];
                if (flag == 0x01)
                {
                    command_type = 1;

                    uint8_t str_len = data_cnt - 1;
                    if (str_len >= sizeof(command_text))
                        str_len = sizeof(command_text) - 1;

                    memcpy(command_text, &data_buf[1], str_len);
                    command_text[str_len] = '\0';

                    command_ready = 1;

                    printf("GCODE COMMAND RECEIVED: \"%s\"\r\n", command_text);
                }
                else if (flag == 0x02)
                {
                    printf("TELEMETRY REQUEST RECEIVED\r\n");
                }
                else
                {
                    printf("UNKNOWN PACKET TYPE: 0x%02X\r\n", flag);
                }
            }

            fsm_state = 0;
            break;
    }
}
void parser_check_timeout(void)
{
    if (fsm_state != 0) //
    {
        if ((HAL_GetTick() - last_byte_time) > PARSER_TIMEOUT_MS)
        {
            printf("TIMEOUT! Reset parser state.\r\n");

            fsm_state = 0;
            data_cnt = 0;
            pkt_len_total = 0;

        }
    }
}
uint8_t crc8_calc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    uint8_t poly = 0x31;      // x^8 + x^5 + x^4 + 1

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
#ifdef USE_FULL_ASSERT
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
