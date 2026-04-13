/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint16_t id;
    uint8_t data[2];
} uart_msg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_DMA_RX_SZ 256 // UART circular DMA RX buffer size

/* UART message packet protocol */
#define UART_ID_VOLTAGE 0x0001
#define UART_ID_CURRENT 0x0002

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

// Variables storing UART RX data
static float target_voltage = 0.0f;

// UART TX variables
uint8_t tx_frame[1+2+2+1];
static volatile uint8_t uart_tx_busy = 0;

// Buffer for storing UART messages by circular DMA
static uint8_t uart_dma_rx[UART_DMA_RX_SZ];
static volatile uint16_t uart_dma_last_pos = 0;

// UART Debugging
typedef struct {
    volatile uint32_t irq_hits;
    volatile uint32_t rx_cb_hits;
    volatile uint32_t rx_events;
    volatile uint16_t last_size;
    volatile uint8_t last_bytes[8];
} uart_dbg_t;

uart_dbg_t g_uart_dbg = {0};

volatile uint32_t g_uart_err_hits = 0;
volatile uint32_t g_uart_last_err = 0;
volatile uint32_t g_uart_last_isr = 0;

static volatile uint8_t dbg_captured = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

// === UART Debugging ===
static uint8_t xor_crc(const uint8_t *p, uint16_t n);

// === UART data TX, encoding and framing ===
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
static void uart_send_voltage(float v);
static void uart_send_current(float i);
static void uart_send_msg(uint16_t id, const uint8_t *data);

// === UART data RX, parsing and processing ===
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
static int parser_feed_byte(uint8_t b, uart_msg_t *out);
static void uart_rx_process_dma(void);
static void log_uart_data(const uart_msg_t *m);

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* UART initialization */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_dma_rx, UART_DMA_RX_SZ);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  /* Processes UART messages stored in buffer */
	 uart_rx_process_dma(); // TODO: parse and process RX without polling

	 /* Sample UART TX for voltage and current readings */
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
          uart_send_voltage(250.3f);
          uart_send_current(7.52f);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED OFF
      } else {
          uart_send_voltage(567.8f);
          uart_send_current(5.71f);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED ON
      }

      HAL_Delay(5); // Wait 5 ms before processing UART again

	  /* Note that UART baud rate is 115200 bits per second, or 86.806 us per byte.
	   * For the 6 byte frame, transmission takes at least 520.836 us. */
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// === UART Debugging ===

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    g_uart_err_hits++;
    g_uart_last_err = huart->ErrorCode;
    g_uart_last_isr = huart->Instance->ISR;

    HAL_UART_AbortReceive(huart);
    uart_dma_last_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_dma_rx, UART_DMA_RX_SZ);
}

static uint8_t xor_crc(const uint8_t *p, uint16_t n)
{
	uint8_t c = 0;
	for(uint16_t i=0;i<n;i++) c ^= p[i];
	return c;
}

// === UART data TX, encoding and framing ===

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) uart_tx_busy = 0;
}

static void uart_send_voltage(float v)
{
	int val = (int)(v * 10.0f + 0.5f);   // convert to xxx.x format
	uint8_t data[2];

	uint8_t d0 = (val / 1000) % 10;
	uint8_t d1 = (val / 100)  % 10;
	uint8_t d2 = (val / 10)   % 10;
	uint8_t d3 =  val % 10;

	data[0] = (d0 << 4) | d1;
	data[1] = (d2 << 4) | d3;

    uart_send_msg(UART_ID_VOLTAGE, data);
}

static void uart_send_current(float i)
{
	int val = (int)(i * 100.0f + 0.5f);   // convert to xx.xx format
	uint8_t data[2];

	uint8_t d0 = (val / 1000) % 10;
	uint8_t d1 = (val / 100)  % 10;
	uint8_t d2 = (val / 10)   % 10;
	uint8_t d3 =  val % 10;

	data[0] = (d0 << 4) | d1;
	data[1] = (d2 << 4) | d3;

    uart_send_msg(UART_ID_CURRENT, data);
}

static void uart_send_msg(uint16_t id, const uint8_t *data)
{
    while (uart_tx_busy) {}      // wait until previous TX is complete
    uart_tx_busy = 1;

    uint16_t idx = 0;
    tx_frame[idx++] = 0xA5;                 // SOF
    tx_frame[idx++] = (uint8_t)(id & 0xFF); // ID low byte
    tx_frame[idx++] = (uint8_t)(id >> 8);   // ID high byte

    tx_frame[idx++] = data[0];
    tx_frame[idx++] = data[1];

    uint8_t crc = xor_crc(tx_frame, idx);
    tx_frame[idx++] = crc;

    if(HAL_UART_Transmit_DMA(&huart1, tx_frame, idx) != HAL_OK) {
        uart_tx_busy = 0;
    }

}

// === UART data RX, parsing and processing ===

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART1) return;

    g_uart_dbg.rx_cb_hits++;
    g_uart_dbg.last_size = Size;

}

static int parser_feed_byte(uint8_t b, uart_msg_t *out)
{
    enum { S_SOF, S_ID0, S_ID1, S_DATA0, S_DATA1, S_CRC };
    static uint8_t st = S_SOF;
    static uint8_t buf[1+2+2];
    static uint8_t idx = 0;

    switch (st)
    {
    case S_SOF:
        if (b == 0xA5) {
            idx = 0;
            buf[idx++] = b;
            st = S_ID0;
        }
        break;

    case S_ID0:
        buf[idx++] = b;
        st = S_ID1;
        break;

    case S_ID1:
        buf[idx++] = b;
        st = S_DATA0;
        break;

    case S_DATA0:
        buf[idx++] = b;
        st = S_DATA1;
        break;

    case S_DATA1:
        buf[idx++] = b;
        st = S_CRC;
        break;

    case S_CRC:
    {
        uint8_t crc = xor_crc(buf, idx);
        if (crc == b)
        {
            out->id = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
            out->data[0] = buf[3];
            out->data[1] = buf[4];
            st = S_SOF;
            return 1;
        }
        st = S_SOF;
        break;
    }
    }

    return 0;
}

static void uart_rx_process_dma(void)
{
    uart_msg_t m;
    uint16_t pos = UART_DMA_RX_SZ - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    if (pos != uart_dma_last_pos)
    {
        if (pos > uart_dma_last_pos)
        {
            for (uint16_t i = uart_dma_last_pos; i < pos; i++)
            {
                if (parser_feed_byte(uart_dma_rx[i], &m)) {
                    log_uart_data(&m);
                }
            }
        }
        else
        {
            for (uint16_t i = uart_dma_last_pos; i < UART_DMA_RX_SZ; i++)
            {
                if (parser_feed_byte(uart_dma_rx[i], &m)) {
                    log_uart_data(&m);
                }
            }
            for (uint16_t i = 0; i < pos; i++)
            {
                if (parser_feed_byte(uart_dma_rx[i], &m)) {
                    log_uart_data(&m);
                }
            }
        }

        uart_dma_last_pos = pos;
    }
}

static void log_uart_data(const uart_msg_t *m)
{
    uint8_t d0 = (m->data[0] >> 4) & 0x0F;
    uint8_t d1 =  m->data[0]       & 0x0F;
    uint8_t d2 = (m->data[1] >> 4) & 0x0F;
    uint8_t d3 =  m->data[1]       & 0x0F;

    // Check that each nibble must be a decimal digit 0..9
    if(d0 > 9 || d1 > 9 || d2 > 9 || d3 > 9) return;

    if(m->id == 0x01)
    {
        // Voltage format: xxx.x
        target_voltage = (float)(d0 * 100 + d1 * 10 + d2) + ((float)d3 / 10.0f);
    }

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
