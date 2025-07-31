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
#include "stdio.h"
#include "stdbool.h"
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
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// InvFreqs计算使用的数据
double w[50];
//complex_double h[50];
volatile uint16_t invs_cnt = 0;


volatile uint8_t UART_data_buf[100];

volatile int freq_out = 1000;
volatile float amp_out = 1.0f;

// 保存上一次的值
static int last_freq = 0;
static float last_amp = 0.0f;

// FPGA操作标志位
volatile bool freq_changed = false;    // 频率变化标志
volatile bool amp_changed = false;     // 幅值变化标志
volatile bool start_study_flag = false;// 启动学习标志

// 模式状态变量
volatile bool single_freq_output = false;  // 单频输出(a1)
volatile bool auto_gain_mode = false;   // 自动增益(b2)
volatile bool simulate_mode = false;    // 模仿模式(c3)

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart1) {
        // 1. 检测开始学习命令(AA AA AA AA)
        if (Size >= 4) {
            bool is_study_cmd = true;
            for (int i = 0; i < 4; i++) {
                if (UART_data_buf[i] != 0xAA) {
                    is_study_cmd = false;
                    break;
                }
            }
            if (is_study_cmd) {
                start_study_flag = true;
                memset(UART_data_buf, 0, 100);
                HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART_data_buf, 100);
                return;
            }
        }

        // 2. 检测模式控制命令(6字节固定格式)
        if (Size == 6) {
            // 单频输出控制(A1开头结尾)
            if (UART_data_buf[0] == 0xA1 && UART_data_buf[5] == 0xA1) {
                single_freq_output = (UART_data_buf[1] == 0x01);
                
            }
            // 自动增益控制(B2开头结尾)
            else if (UART_data_buf[0] == 0xB2 && UART_data_buf[5] == 0xB2) {
                auto_gain_mode = (UART_data_buf[1] == 0x01);
                
            }
            // 模仿模式控制(C3开头结尾)
            else if (UART_data_buf[0] == 0xC3 && UART_data_buf[5] == 0xC3) {
                simulate_mode = (UART_data_buf[1] == 0x01);
                
            }
        }

        // 3. 解析频率和幅值(保留原有AD帧头逻辑)
        int current_freq = 0;
        float current_amp = 0.0f;
        int decode_status = decode_uart(UART_data_buf, Size, &current_freq, &current_amp);
        
        if (decode_status == 1) {
            // 检测频率变化
            if (current_freq != last_freq) {
                freq_out = current_freq;
                last_freq = current_freq;
                freq_changed = true;
            }
            
            // 检测幅值变化
            if (fabs(current_amp - last_amp) > 0.05f) {
                amp_out = current_amp;
                last_amp = current_amp;
                amp_changed = true;
            }
        }

        // 重新启动DMA接收
        memset(UART_data_buf, 0, 100);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART_data_buf, 100);
    }
}

// 发送模式状态到串口屏
void send_mode_status() {
    uint8_t buf[6];
    
    // 单频输出状态(A1 01/00 00 00 00 A1)
    buf[0] = 0xA1;
    buf[1] = single_freq_output ? 0x01 : 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0xA1;
    HAL_UART_Transmit(&huart1, buf, 6, 10);
    
    // 自动增益状态(B2 01/00 00 00 00 B2)
    buf[0] = 0xB2;
    buf[1] = auto_gain_mode ? 0x01 : 0x00;
    buf[5] = 0xB2;
    HAL_UART_Transmit(&huart1, buf, 6, 10);
    
    // 模仿模式状态(C3 01/00 00 00 00 C3)
    buf[0] = 0xC3;
    buf[1] = simulate_mode ? 0x01 : 0x00;
    buf[5] = 0xC3;
    HAL_UART_Transmit(&huart1, buf, 6, 10);
}

int decode_uart(uint8_t *uart_buf, uint16_t buf_len, int *current_freq, float *current_amp) {
    if (uart_buf == NULL || current_freq == NULL || current_amp == NULL) {
        return 0;
    }

    // 保留原有AD帧头解析逻辑(频率和幅值)
    uint8_t ad_header[4] = {0xAD, 0xAD, 0xAD, 0xAD};
    if (buf_len >= 12) {
        for (uint16_t i = 0; i <= buf_len - 12; i++) {
            if (memcmp(&uart_buf[i], ad_header, 4) == 0) {
                // 解析频率(4字节小端)
                uint32_t freq_raw = (uint32_t)uart_buf[i+4] |
                                   ((uint32_t)uart_buf[i+5] << 8) |
                                   ((uint32_t)uart_buf[i+6] << 16) |
                                   ((uint32_t)uart_buf[i+7] << 24);

                // 解析幅值(4字节小端)
                uint32_t amp_raw = (uint32_t)uart_buf[i+8] |
                                  ((uint32_t)uart_buf[i+9] << 8) |
                                  ((uint32_t)uart_buf[i+10] << 16) |
                                  ((uint32_t)uart_buf[i+11] << 24);

                *current_freq = (int)freq_raw;
                *current_amp = (float)amp_raw / 10.0f;

                return 1;
            }
        }
    }

    return 0;
}

int send_data(const char* str_b0, const char* str_b1, 
                          const char* str_b2, const char* str_a1, const char* str_a2) {
    if (str_b0 == NULL || str_b1 == NULL || str_b2 == NULL || str_a1 == NULL || str_a2 == NULL) {
        return -1;
    }

    uint8_t uart_send_buf[256];
    size_t buf_offset = 0;
    int write_len;

    write_len = sprintf((char*)(uart_send_buf + buf_offset), "para1.txt=\"b0=%s\"", str_b0);
    if (write_len < 0) return -2;
    buf_offset += write_len;
    if (buf_offset + 3 > sizeof(uart_send_buf)) return -2;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;

    write_len = sprintf((char*)(uart_send_buf + buf_offset), "para2.txt=\"b1=%s\"", str_b1);
    if (write_len < 0) return -2;
    buf_offset += write_len;
    if (buf_offset + 3 > sizeof(uart_send_buf)) return -2;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;

    write_len = sprintf((char*)(uart_send_buf + buf_offset), "para3.txt=\"b2=%s\"", str_b2);
    if (write_len < 0) return -2;
    buf_offset += write_len;
    if (buf_offset + 3 > sizeof(uart_send_buf)) return -2;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;

    write_len = sprintf((char*)(uart_send_buf + buf_offset), "para4.txt=\"a1=%s\"", str_a1);
    if (write_len < 0) return -2;
    buf_offset += write_len;
    if (buf_offset + 3 > sizeof(uart_send_buf)) return -2;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;

    write_len = sprintf((char*)(uart_send_buf + buf_offset), "para5.txt=\"a2=%s\"", str_a2);
    if (write_len < 0) return -2;
    buf_offset += write_len;
    if (buf_offset + 3 > sizeof(uart_send_buf)) return -2;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;

    if (HAL_UART_Transmit(&huart1, uart_send_buf, buf_offset, 1000) != HAL_OK) {
        return -3;
    }

    return 0;
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,UART_data_buf,100);
  while (1)
  {
		// 在main函数中处理FPGA相关操作
    if (freq_changed) {
        // *************************** 通知FPGA输出频率变化 ***************************//
        freq_changed = false;  // 清除标志位
    }
    
    if (amp_changed) {
        // *************************** 通知FPGA输出幅值变化 ***************************//
        amp_changed = false;   // 清除标志位
    }
    
		char studying_string[]="loading.aph=127\xff\xff\xffhook.aph=0\xff\xff\xff";
		char study_funished_string[]="loading.aph=0\xff\xff\xffhook.aph=127\xff\xff\xff";
		
    if (start_study_flag) {
        // *************************** 通知FPGA启动学习操作 ***************************//
        HAL_UART_Transmit(&huart1,studying_string,sizeof(studying_string)-1,10);//长度要-1否则会发一个空字节出问题
        // 学习触发后，调用send_data发送指定数值（示例：b0=1.2, b1=3.4, b2=5.6, a1=7.8, a2=9.0）
        // 实际使用时可根据需求修改参数值
        send_data("1.2", "3.4", "5.6", "7.8", "9.0");
        HAL_Delay(1000);
				HAL_UART_Transmit(&huart1,study_funished_string,sizeof(study_funished_string)-1,10);//长度要-1否则会发一个空字节出问题
        start_study_flag = false;  // 清除标志位
    }
		send_data("0", "0", "0","0","0" );
		
		
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
