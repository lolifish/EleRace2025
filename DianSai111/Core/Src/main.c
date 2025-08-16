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
#include <string.h>

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

#define M_PI 3.14159265358979323846
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double unwrap_deg(double deg1, double deg2) {
    double delta = deg2 - deg1;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;
    return delta;
}


volatile uint8_t simulation_en = 0;
volatile double basic_freq;

// InvFreqs计算使用的数据
const double DAC_OUT_VPP = 3.7418;
const double ADC_IN_VPP = 4.04;
volatile uint16_t spi_data;

volatile double w[125];
volatile complex_double h[125];
// 幅频相频

volatile double amp_freq[125];
volatile double phase_freq[125];

volatile uint16_t invs_cnt = 0;   // 频响总数


volatile uint8_t UART_data_buf[100];

volatile int freq_out = 1000;
volatile float amp_out = 1.0f;

// 系统标定值
const float biaoding_values[30] = {
    0.206, 0.216, 0.229, 0.247, 0.269, 0.293, 0.320, 0.349, 0.380, 0.410,
    0.442, 0.476, 0.510, 0.546, 0.580, 0.610, 0.658, 0.697, 0.739, 0.780,
    0.810, 0.864, 0.907, 0.953, 1.0, 1.046, 1.094, 1.143, 1.191, 1.245
};

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

volatile bool single_freq_output_StatusChanged = false;  // 单频输出开关状态改变标志位
volatile bool auto_gain_mode_StatusChanged = false;   // 自动增益开关状态改变标志位
volatile bool simulate_mode_StatusChanged = false;    // 模仿模式开关状态改变标志位


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
                HAL_UARTEx_ReceiveToIdle_DMA(&huart1,UART_data_buf, 100);
                return;
            }
        }

        // 2. 检测模式控制命令(6字节固定格式)
        if (Size == 6) {
            // 单频输出控制(A1开头结尾)
            if (UART_data_buf[0] == 0xA1 && UART_data_buf[5] == 0xA1) {
                single_freq_output = (UART_data_buf[1] == 0x01);
                single_freq_output_StatusChanged=1;//标志位
            }
            // 自动增益控制(B2开头结尾)
            else if (UART_data_buf[0] == 0xB2 && UART_data_buf[5] == 0xB2) {
                auto_gain_mode = (UART_data_buf[1] == 0x01);
                auto_gain_mode_StatusChanged=1;//标志位
            }
            // 模仿模式控制(C3开头结尾)
            else if (UART_data_buf[0] == 0xC3 && UART_data_buf[5] == 0xC3) {
                simulate_mode = (UART_data_buf[1] == 0x01);
                simulate_mode_StatusChanged=1;//标志位
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
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)UART_data_buf, 100);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // 拉高使能引脚
  HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
  uint8_t simulation_mode_ready_quit = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *)UART_data_buf,100);


  while (1)
  {
		// 在main函数中处理FPGA相关操作
    if (freq_changed || amp_changed || single_freq_output_StatusChanged || auto_gain_mode_StatusChanged) {
        // *************************** 通知FPGA单频输出变化 ***************************//
        // 开始通信
        spi_data = 0x0001; 
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
        // 发送目标标志位和幅度信息
        spi_data = 0;
        double transfer_value = 1;
        if (auto_gain_mode) transfer_value = biaoding_values[freq_out/100-1];
        uint16_t fixed_point = (uint16_t)roundf(amp_out*transfer_value * (1 << 13));
        fixed_point &= 0x7FFF;
        spi_data = (single_freq_output ? 0x8000 : 0x0000) | fixed_point;
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
        // 发送目标频率信息
        spi_data = (uint16_t)(freq_out / 25);
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
        // 结束通信
        spi_data = 0x0000; 
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
        // 清除标志位
        freq_changed = false;  
        amp_changed = false;
        single_freq_output_StatusChanged = false;
        auto_gain_mode_StatusChanged = false;
    }
    
		char studying_string[]="loading.aph=127\xff\xff\xffhook.aph=0\xff\xff\xff";
		char study_funished_string[]="loading.aph=0\xff\xff\xffhook.aph=127\xff\xff\xff";
    
      
      if (start_study_flag) {
          // 先通知fpga开始学习
          spi_data = 0x0002;  // 16位数据，只最低2位为1
          HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
          HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
          HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
          // 修改屏幕图标到繁忙
          HAL_UART_Transmit(&huart1, (uint8_t*)&studying_string,sizeof(studying_string)-1,10);
          // 等待fpga写准备
          while (1) {
            HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
            HAL_SPI_Receive(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
            HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
            // 收到0xF002说明fpga已经完成扫频，可进行数据传输
            if (spi_data == 0xF002) break;
            HAL_Delay(1);
          }

          // 开始接受FPGA返回的数据
          invs_cnt = 0;
          while (1){
            // 接受频率
            HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
            HAL_SPI_Receive(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
            HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
            // 频率收到0xFFFD退出
            if (spi_data == 0xFFFD) break;
            // 处理频率
            double freq = spi_data*100.0;
            w[invs_cnt] = freq * 2 * M_PI;

            // 接受幅频
            HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
            HAL_SPI_Receive(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
            HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
            // 处理幅度
            double amp = (double)spi_data * ADC_IN_VPP / DAC_OUT_VPP / 255;
            // 接受相频
            HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
            HAL_SPI_Receive(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
            HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
            // 对于幅度逼近0的情况，相位无效，丢弃此频点
            if (amp < 0.0001) continue;
            // 处理相位
            double phase_rad = (double)(int16_t)spi_data/(1 << 13)/M_PI*180.0;
            amp_freq[invs_cnt] = amp;
            // 标定的系统相移
            double delta_phase = freq * 0.000202;
            if (delta_phase > 180) delta_phase = delta_phase-360;
            phase_rad += delta_phase;
            // 设置回180度以内
            if (phase_rad > 180) phase_rad -= 360;
            if (phase_rad < -180) phase_rad+=360;
            // 执行两遍
            if (phase_rad > 180) phase_rad -= 360;
            if (phase_rad < -180) phase_rad+=360;

            phase_freq[invs_cnt] = phase_rad;

            // 复频响
            h[invs_cnt].real = cos(phase_rad*M_PI/180) * amp;
            h[invs_cnt].imag = sin(phase_rad*M_PI/180) * amp;

            invs_cnt += 1;
          }
          // 结束通信
          spi_data = 0x0000; 
          HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
          HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
          HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);

          // 计算InvFreqs
          complex_double b_coeffs[3];
          complex_double a_coeffs[3];
          invfreqs_stm32(h, w, invs_cnt, b_coeffs, a_coeffs);

          // 转为字符串发送
          char real_str1[64];
          char real_str2[64];
          char real_str3[64];
          char real_str4[64];
          char real_str5[64];

          // 将实部转换为科学计数法字符串
          snprintf(real_str1, sizeof(real_str1), "%.2e", b_coeffs[0].real);
          snprintf(real_str2, sizeof(real_str2), "%.2e", b_coeffs[1].real);
          snprintf(real_str3, sizeof(real_str3), "%.2e", b_coeffs[2].real);
          snprintf(real_str4, sizeof(real_str4), "%.2e", a_coeffs[1].real);
          snprintf(real_str5, sizeof(real_str5), "%.2e", a_coeffs[2].real);
          send_data(real_str1,real_str2,real_str3,real_str4,real_str5);
          // 修改屏幕图标到完成
          HAL_UART_Transmit(&huart1,(uint8_t*)study_funished_string,sizeof(study_funished_string)-1,10);

          
          // 滤波器类型判断
          // t11带阻，t10带通，t9低通，t8高通
          uint8_t DaiZu[] = "vis t11,1\xff\xff\xff";
          uint8_t DaiTong[] = "vis t10,1\xff\xff\xff";
          uint8_t DiTong[] ="vis t9,1\xff\xff\xff";
          uint8_t GaoTong[]="vis t8,1\xff\xff\xff";
					uint8_t weizhi[]="vis t14,1\xff\xff\xff";
          // 判断标志位
          uint8_t start_pass = 0; // 低频有通带
          uint8_t start_stop = 0; // 低频有阻带
          uint8_t start_rise = 0; // 低频有上升趋势
          uint8_t start_drop = 0; // 低频有下降趋势
          uint8_t end_pass   = 0; // 高频有通带
          uint8_t end_stop   = 0; // 高频有阻带
          uint8_t end_rise   = 0; // 高频有上升趋势
          uint8_t end_drop   = 0; // 高频有下降趋势
          uint8_t middle_stop= 0; // 中间存在明显尖峰
          uint8_t middle_pass= 0; // 中间存在明显尖峰
          // 计算标志位
          double start_1st = (amp_freq[0]+amp_freq[1]+amp_freq[2])/3;
          double start_2nd = (amp_freq[5]+amp_freq[6]+amp_freq[7])/3;
          double end_1st = (amp_freq[invs_cnt-1]+amp_freq[invs_cnt-2]+amp_freq[invs_cnt-3])/3;
          double end_2nd = (amp_freq[invs_cnt-6]+amp_freq[invs_cnt-7]+amp_freq[invs_cnt-8])/3;
          if (start_1st > 0.5) start_pass = 1;
          if (start_1st < 0.15) start_stop = 1;
          if (start_2nd / start_1st > 1.05) start_rise = 1;
          if (start_1st / start_2nd > 1.05) start_drop = 1;
          if (end_1st > 0.5) end_pass = 1;
          if (end_1st < 0.15) end_stop = 1;
          if (end_2nd / end_1st > 1.05) end_drop = 1;
          if (end_1st / end_2nd > 1.05) end_rise = 1;
          for (int i=15; i<invs_cnt-18; i+=1){
            double amp_middle = (amp_freq[i]+amp_freq[i+1]+amp_freq[i+2])/3;
            double amp_left = (amp_freq[i-10]+amp_freq[i-10+1]+amp_freq[i-10+2])/3;
            double amp_right = (amp_freq[i+10]+amp_freq[i+10+1]+amp_freq[i+10+2])/3;
            if ((amp_middle/amp_left>1.05) && (amp_middle/amp_right>1.05)) middle_pass = 1;
            if ((amp_middle/amp_left<0.95) && (amp_middle/amp_right<0.95)) middle_stop = 1;
          }
          // 根据标志位判断滤波器类型
          // 如果低频存在阻带/低频在上升，那么是高通或者带通
          if ((start_stop||start_rise)){
            // 如果高频存在阻带/高频在下降/中频存在尖峰三个条件满足2个以上，那么就是带通，否则就认为是高通
            if (end_stop*2+end_drop+middle_pass >= 2) HAL_UART_Transmit(&huart1, DaiTong, sizeof(DaiTong), 10);
            else HAL_UART_Transmit(&huart1,GaoTong, sizeof(GaoTong), 10);
          }
          // 否则在低通和带阻间决策
          else {
            if (end_pass*2+end_rise+middle_stop >= 2) HAL_UART_Transmit(&huart1, DaiZu, sizeof(DaiZu), 10);
            else HAL_UART_Transmit(&huart1, DiTong, sizeof(DiTong), 10);
          }
          start_study_flag = false;
      }

      if(simulate_mode_StatusChanged){
        // 进入模拟状态立即执行
        if (simulate_mode) {
          spi_data = 0x0004;
          simulation_en = 1;
          simulation_mode_ready_quit = 0;
          HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
          HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
          HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
        }
        // 退出模拟状态则设置标志位，
        else simulation_mode_ready_quit=1;
        
        simulate_mode_StatusChanged = false;
      }

    // simulate_mode维持的事务
    if (simulation_en) {
      HAL_Delay(19);
      // 先询问频率计频率
      HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
      HAL_SPI_Receive(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
      basic_freq = spi_data*100.0/2;
      
      // 检测是否要停止
      if (simulation_mode_ready_quit) {
        simulation_en = 0;
        spi_data=0xF008;
      }
      else spi_data = 0xF004;

      HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
    }
    if (simulation_en){
      // 计算10个频点的IQ
      for (int i=0; i<11; i++){
        double w_now = basic_freq*(i+1)*2*M_PI;
        // 找到离w_now最近的频率低于w_now的位置j，算出多余频率在频段中的位置（0-1归一化表示）
        uint16_t j=0;
        while (1) {
            if (w[j+1] > w_now) break;
            j += 1;
            if (j>=124) break;
        }
        double dis = (w_now-w[j])/(w[j+1]-w[j]);
        // 线性拟合Amp和Phase
        double amp_now =    amp_freq[j] +     (amp_freq[j+1]-amp_freq[j])     * dis;
        double delta_phase = unwrap_deg(phase_freq[j], phase_freq[j+1]);
        double phase_now = phase_freq[j] + delta_phase * dis;
        // 计算IQ
        double I = (amp_now*255*1.18)*cos(phase_now * M_PI/180);
        double Q = (amp_now*255*1.18)*sin(phase_now * M_PI/180);
        // 发送
        int16_t int_part_I = (int16_t)I;  // 取整数部分
        uint16_t signed10bit_I = ((uint16_t)(int_part_I & 0x03FF));   // 取低10位

        int16_t int_part_Q = (int16_t)Q;  // 取整数部分
        uint16_t signed10bit_Q = ((uint16_t)(int_part_Q & 0x03FF));   // 取低10位

        uint16_t high4 = ((uint16_t)(i & 0x0F)) << 12;            // 高4位为i
        spi_data = high4 | signed10bit_I;                       // 拼装
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
        spi_data = high4 | signed10bit_Q;                       // 拼装
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, (uint8_t*)&spi_data, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin, GPIO_PIN_SET);
      }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();

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
