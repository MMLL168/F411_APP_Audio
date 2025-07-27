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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>


#include "ssd1306.h"
#include "ssd1306_fonts.h"

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    // 使用 10ms 超時，失敗也返回成功
    HAL_UART_Transmit(DEBUG_UART_PORT, (uint8_t*)ptr, len, 10);
    return len;  // 總是返回成功
}

// === ADC DMA 相關 ===
#define ADC_BUF_SIZE 1024
uint16_t adc_dma_buf[ADC_BUF_SIZE];
volatile uint8_t adc_data_ready = 0;
volatile uint32_t adc_sum = 0;
volatile uint16_t adc_count = 0;

// 配置參數
#define SAMPLE_RATE     16000  // 16kHz 採樣率
#define FRAME_SIZE      512    // 每幀數據大小
#define BUFFER_SIZE     1024   // 雙緩衝區
#define TX_BUFFER_SIZE  (4 + FRAME_SIZE*2 + 2)  // 幀頭(4) + 數據(FRAME_SIZE*2) + 校驗和(2)
// 全局變量
uint16_t adc_buffer[BUFFER_SIZE];  // DMA 雙緩衝
volatile uint8_t buffer_half = 0;   // 緩衝區標誌
volatile uint8_t buffer_full = 0;   // 緩衝區標誌
uint8_t tx_buffer[TX_BUFFER_SIZE];  // UART 發送緩衝區

// UART 發送函數
HAL_StatusTypeDef UART_SendData(uint8_t* data, uint16_t size)
{
    return HAL_UART_Transmit(&huart6, data, size, 100);  // 100ms 超時
}

// 數據處理函數
void ProcessAudioData(uint16_t* data, uint16_t size)
{
    static uint32_t frame_count = 0;
    frame_count++;

    // 簡單的 DC 偏移去除
    int32_t sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += data[i];
    }
    int16_t dc_offset = sum / size;

    // 幀頭
    tx_buffer[0] = 0xAA;
    tx_buffer[1] = 0x55;
    tx_buffer[2] = size & 0xFF;
    tx_buffer[3] = (size >> 8) & 0xFF;

    // 數據
    uint16_t checksum = 0;
    for(int i = 0; i < size; i++)
    {
        int16_t sample = data[i] - dc_offset;
        checksum += sample;
        tx_buffer[4 + i*2] = sample & 0xFF;
        tx_buffer[4 + i*2 + 1] = (sample >> 8) & 0xFF;
    }

    // 校驗和
    tx_buffer[4 + size*2] = checksum & 0xFF;
    tx_buffer[4 + size*2 + 1] = (checksum >> 8) & 0xFF;

    // 發送數據
    if(UART_SendData(tx_buffer, TX_BUFFER_SIZE) != HAL_OK)
    {
        // 可以添加 LED 閃爍來指示傳輸錯誤
        Error_Handler();
    }
}

// ADC DMA 回調
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        buffer_half = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        buffer_full = 1;
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    // 設置向量表到APP區域
  SCB->VTOR = 0x08008000;
  __disable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  __enable_irq();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  // 🔧 簡化測試：先測試基本功能
  printf("=== APP STARTED ===\r\n");

  // 測試 LED（PA5 是板載 LED）
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = HM_OPA_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HM_OPA_ADC_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(HM_OPA_ADC_GPIO_Port, HM_OPA_ADC_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(HM_OPA_ADC_GPIO_Port, HM_OPA_ADC_Pin, GPIO_PIN_RESET);

  printf("LED Test completed\r\n");

  /* 初始化SSD1306 */
  char buf[32];
  printf("ssd1306_Init...\r\n");
  ssd1306_Init();

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("ssd1306_Init OK!!", Font_6x8, White);
  ssd1306_UpdateScreen();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t Counter = 0;

  // 啟動 ADC DMA
  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE) != HAL_OK)
  {
      Error_Handler();
  }

  // 啟動定時器基礎功能
  if (HAL_TIM_Base_Start(&htim4) != HAL_OK)
  {
      Error_Handler();
  }

  // 啟動定時器
  if(HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
  {
      Error_Handler();
  }

  // 發送啟動消息
  uint8_t start_msg[] = "Audio Sampling Start\r\n";
  HAL_UART_Transmit(&huart6, start_msg, sizeof(start_msg)-1, 100);


  while (1)
  {
      if(buffer_half)
      {
    	  //printf("adc_buffer[0]: %d\r\n", adc_buffer[0]);
          ProcessAudioData(&adc_buffer[0], FRAME_SIZE);
          buffer_half = 0;
      }

      if(buffer_full)
      {
    	  //printf("adc_buffer[10]: %d\r\n", adc_buffer[10]);
          ProcessAudioData(&adc_buffer[FRAME_SIZE], FRAME_SIZE);
          buffer_full = 0;
      }

      //HAL_GPIO_TogglePin(HM_OPA_ADC_Pin, HM_OPA_ADC_Pin);  // 假設 PC13 是板載 LED
      //HAL_Delay(100);  // LED 閃爍頻率
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
