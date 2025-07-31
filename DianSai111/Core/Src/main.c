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
#include <stdint.h>

#include "invfreqs_stm32.c"
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
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// InvFreqs计算使用的数据
double w[50];
complex_double h[50];
uint16_t invs_cnt = 0;

uint8_t UART_data_buf[100];
bool start_study = 0;
volatile int freq_out = 0;
volatile float amp_out = 0.0f;

// 新增静态变量用于保存上一次的值（替代decode_uart中的静态变量）
static int last_freq = 0;
static float last_amp = 0.0f;

// 新增FPGA操作标志位
volatile bool freq_changed = false;    // 频率变化标志
volatile bool amp_changed = false;     // 幅值变化标志
volatile bool start_study_flag = false;// 启动学习标志

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart1) {
        
        // 1. 解析当前帧的频率和幅值（仅获取原始值，不检测变化）
        int current_freq = 0;
        float current_amp = 0.0f;
        int decode_status = decode_uart(UART_data_buf, Size, &current_freq, &current_amp);
        
        // 2. 在回调函数中进行变化检测（核心迁移逻辑）
        if (decode_status == 1) {  // 解析到有效频率和幅值
            // 检测频率变化
            if (current_freq != last_freq) {
                freq_out = current_freq;
                last_freq = current_freq;
                freq_changed = true;  // 设置频率变化标志
            }
            
            // 检测幅值变化（使用0.05容差确保0.1精度的变化被正确识别）
            if (fabs(current_amp - last_amp) > 0.05f) {
                amp_out = current_amp;
                last_amp = current_amp;
                amp_changed = true;   // 设置幅值变化标志
            }
        }
        
        // 3. 处理启动命令检测
        start_study = check_start_study_command(UART_data_buf, Size);
        if (start_study == 1) {
            start_study_flag = true;  // 设置启动学习标志
            start_study = 0;  // 清除标志
        }
        
        // 4. 清空缓冲区并重新启动DMA接收
        memset(UART_data_buf, 0, 100);  // 更高效的清空方式
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART_data_buf, 100);
    }
}

bool check_start_study_command(uint8_t *uart_buf, uint16_t buf_len) {
    if (uart_buf == NULL || buf_len < 6) {
        return false;
    }
    uint8_t target_frame[6] = {0xAA, 0xAA, 0xAA, 0xAA, 0x11, 0x11};
    for (uint16_t i = 0; i <= buf_len - 6; i++) {
        if (uart_buf[i] == target_frame[0] &&
            uart_buf[i+1] == target_frame[1] &&
            uart_buf[i+2] == target_frame[2] &&
            uart_buf[i+3] == target_frame[3] &&
            uart_buf[i+4] == target_frame[4] &&
            uart_buf[i+5] == target_frame[5]) {
            return true;
        }
    }
    return false;
}

// 修改后的decode_uart：仅解析数据，不检测变化，返回是否解析成功
int decode_uart(uint8_t *uart_buf, uint16_t buf_len, int *current_freq, float *current_amp) {
    // 检查缓冲区合法性（4帧头+4频率+4幅值=12字节）
    if (uart_buf == NULL || buf_len < 12 || current_freq == NULL || current_amp == NULL) {
        return 0;  // 解析失败
    }

    // 帧头：AD AD AD AD
    uint8_t header[4] = {0xAD, 0xAD, 0xAD, 0xAD};

    // 遍历缓冲区查找帧头
    for (uint16_t i = 0; i <= buf_len - 12; i++) {
        if (memcmp(&uart_buf[i], header, 4) == 0) {
            // 解析频率（4字节小端模式）
            uint32_t freq_raw = (uint32_t)uart_buf[i+4] |
                               ((uint32_t)uart_buf[i+5] << 8) |
                               ((uint32_t)uart_buf[i+6] << 16) |
                               ((uint32_t)uart_buf[i+7] << 24);

            // 解析幅值（4字节小端模式）
            uint32_t amp_raw = (uint32_t)uart_buf[i+8] |
                              ((uint32_t)uart_buf[i+9] << 8) |
                              ((uint32_t)uart_buf[i+10] << 16) |
                              ((uint32_t)uart_buf[i+11] << 24);

            // 转换为实际值（频率为整数，幅值为浮点数精确到0.1）
            *current_freq = (int)freq_raw;
            *current_amp = (float)amp_raw / 10.0f;

            return 1;  // 解析成功
        }
    }

    return 0;  // 未找到有效帧头，解析失败
}
int send_data(const char* str_b0, const char* str_b1, 
                          const char* str_b2, const char* str_a1, const char* str_a2) {
    // 1. 检查输入参数有效性（避免空指针）
    if (str_b0 == NULL || str_b1 == NULL || str_b2 == NULL || str_a1 == NULL || str_a2 == NULL) {
        return -1; // 输入参数无效
    }

    // 2. 定义发送缓冲区（根据最大可能长度设置，256字节足够容纳5个参数）
    uint8_t uart_send_buf[256];
    size_t buf_offset = 0; // 缓冲区偏移量，记录当前写入位置
    int write_len; // 临时变量，记录单次sprintf写入的字节数

    // 3. 组装para1指令：格式 "para1.txt=\"para1=xxx\"" + 分隔符（3个0xFF）
    write_len = sprintf((char*)(uart_send_buf + buf_offset), "para1.txt=\"b0=%s\"", str_b0);
    if (write_len < 0) return -2; // 格式化失败
    buf_offset += write_len;
    // 添加分隔符（3个0xFF），检查缓冲区剩余空间
    if (buf_offset + 3 > sizeof(uart_send_buf)) return -2; // 缓冲区溢出
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;

    // 4. 组装para2指令：格式 "para2.txt=\"para2=xxx\""
    write_len = sprintf((char*)(uart_send_buf + buf_offset), "para2.txt=\"b1=%s\"", str_b1);
    if (write_len < 0) return -2;
    buf_offset += write_len;
    if (buf_offset + 3 > sizeof(uart_send_buf)) return -2;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;

    // 5. 组装para3指令：格式 "para3.txt=\"para3=xxx\""
    write_len = sprintf((char*)(uart_send_buf + buf_offset), "para3.txt=\"b2=%s\"", str_b2);
    if (write_len < 0) return -2;
    buf_offset += write_len;
    if (buf_offset + 3 > sizeof(uart_send_buf)) return -2;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;

    // 6. 组装para4指令：格式 "para4.txt=\"para4=xxx\""
    write_len = sprintf((char*)(uart_send_buf + buf_offset), "para4.txt=\"a1=%s\"", str_a1);
    if (write_len < 0) return -2;
    buf_offset += write_len;
    if (buf_offset + 3 > sizeof(uart_send_buf)) return -2;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;

    // 7. 组装para5指令：格式 "para5.txt=\"para5=xxx\""
    write_len = sprintf((char*)(uart_send_buf + buf_offset), "para5.txt=\"a2=%s\"", str_a2);
    if (write_len < 0) return -2;
    buf_offset += write_len;
    if (buf_offset + 3 > sizeof(uart_send_buf)) return -2;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;
    uart_send_buf[buf_offset++] = 0xFF;

    // 8. 通过USART1发送组装好的指令
    if (HAL_UART_Transmit(&huart1, uart_send_buf, buf_offset, 1000) != HAL_OK) {
        return -3; // 串口发送失败（超时或错误）
    }

    return 0; // 全部指令发送成功
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // 拉高使能引脚
  HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
  // 复响应数据缓冲
  // 角频率数据缓冲

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
        // 先通知fpga开始学习
        uint16_t spi_data = 0x0002;  // 16位数据，只最低2位为1
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
        // 修改屏幕图标到繁忙
        HAL_UART_Transmit(&huart1, (uint8_t*)&studying_string,sizeof(studying_string),10);
        // 开始接受FPGA返回的数据
        invs_cnt = 0;
        while (1){
          // 接受频率
          HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
          HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
          HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
          // 频率收到0x0002退出
          if (spi_data == 0x0002) break;
          // 处理频率
          w[invs_cnt] = spi_data*100.0 * 2 * 3.141592654;

          // 接受响应
          HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
          HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
          HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
          // 处理响应
          uint8_t amp_buffer = (spi_data >> 8) & 0xFF;
          uint8_t pha_buffer = spi_data & 0xFF;
          double phase_rad = ((double)pha_buffer - 90.0) * 2.0;
          double amp = 0.001 + amp_buffer * (10.0 - 0.001) / 240.0;
          h[invs_cnt].real = cos(phase_rad) * amp;
          h[invs_cnt].imag = sin(phase_rad) * amp;

          invs_cnt += 1;
        }

        // 计算InvFreqs
        complex_double b_coeffs[3];
        complex_double a_coeffs[2];
        invfreqs_stm32(h, w, 50, b_coeffs, a_coeffs);

        // 转为字符串发送
        char real_str1[64];
        char real_str2[64];
        char real_str3[64];
        char real_str4[64];
        char real_str5[64];

        // 将实部转换为科学计数法字符串
        snprintf(real_str1, sizeof(real_str1), "%.3e", b_coeffs[0].real);
        snprintf(real_str2, sizeof(real_str1), "%.3e", b_coeffs[1].real);
        snprintf(real_str3, sizeof(real_str1), "%.3e", b_coeffs[2].real);
        snprintf(real_str4, sizeof(real_str1), "%.3e", a_coeffs[0].real);
        snprintf(real_str5, sizeof(real_str1), "%.3e", a_coeffs[1].real);
        send_data(real_str1,real_str2,real_str3,real_str4,real_str5);
        // 修改屏幕图标到完成
				HAL_UART_Transmit(&huart1,(uint8_t*)study_funished_string,sizeof(study_funished_string),10);

        // 清除标志位
        start_study_flag = false;  
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI1_EN_Pin */
  GPIO_InitStruct.Pin = SPI1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_EN_GPIO_Port, &GPIO_InitStruct);

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
