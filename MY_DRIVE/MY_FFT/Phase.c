/**
 ******************************************************************************
 * @file    Phase.c
 * @brief   相位计算与FFT算法源文件 - 适配STM32H7系列
 * @author  POLO_HU
 * @version V2.1
 * @date    2026-05-23
 ******************************************************************************
 * @attention
 *
 * 本文件实现了基于FFT算法的频谱分析功能，适用于STM32H7系列微控制器。
 * 主要功能：
 * 1. ADC DMA 信号采集 (TIM3 TRGO触发, Si5351 ETR时钟)
 * 2. 4096点FFT变换 (CMSIS DSP arm_cfft_f32)
 * 3. Hanning窗 / 幅值归一化 (输出实际电压)
 * 4. 串口频谱输出 (FFT_BEGIN/FFT_END 帧格式, 适配 plot_fft_from_serial.m)
 *
 * @note   V2.1 变更:
 *         - 移除 DFT/THD 计算, 简化为纯FFT频谱输出
 *         - 窗函数改用 WindowFunction.c 的 Hanning 窗
 *         - TIM3 周期覆盖为 2 (采样率 512kHz)
 *         - 修正 Si5351 ETR 时钟下的采样率自动计算
 *         - 添加 FFT_SendSpectrumFrame() 输出 MATLAB 脚本兼容格式
 *
 ******************************************************************************
 */

#include "Phase.h"
#include "WindowFunction.h"
#include "stm32h743xx.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "ZPN_Uart.h"
#include <stdio.h>

/* 当前工程ADC配置为16bit，满量程65535 */
static uint32_t ADC_MaxValue = ((1UL << ADC_EFFECTIVE_BITS) - 1UL);
static float32_t FFT_SamplingRate_Hz = SAMPLING_RATE_DEFAULT;
static volatile uint8_t g_frame_ready = 0;
static volatile uint8_t g_transmitting = 0;
static uint16_t adc_snapshot[FFT_LENGTH] = {0};
static float32_t fft_mag_snapshot[FFT_LENGTH] = {0.0f};

static uint8_t SCAN_ConfigAdc1ToFFTInput(void);

static void Phase_StartAcq(void)
{
  if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Buffer, FFT_LENGTH) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Phase_GenerateSimulatedSignal(float32_t *signal, uint16_t length)
{
  static float32_t phase1 = 0.0f;
  static float32_t phase2 = 0.0f;
  const float32_t a1 = 0.80f;
  const float32_t a2 = 0.35f;
  const float32_t two_pi = 2.0f * PI;
  float32_t fs = FFT_SamplingRate_Hz;
  float32_t f1;
  float32_t f2;

  /* 选取整点频率，减少频谱泄漏，便于校验 */
  const uint16_t k1 = 50U;
  const uint16_t k2 = 120U;

  if (fs < 1.0f)
  {
    fs = SAMPLING_RATE_DEFAULT;
  }

  f1 = ((float32_t)k1 * fs) / (float32_t)length;
  f2 = ((float32_t)k2 * fs) / (float32_t)length;

  for (uint16_t i = 0; i < length; i++)
  {
    float32_t ac = a1 * arm_sin_f32(phase1) + a2 * arm_sin_f32(phase2);
    float32_t sample = 1.65f + ac;

    if (sample < 0.0f)
    {
      sample = 0.0f;
    }
    else if (sample > 3.3f)
    {
      sample = 3.3f;
    }

    signal[i] = ac;
    ADC_Buffer[i] = (uint16_t)((sample / Reference_Voltage) * (float32_t)ADC_MaxValue);

    phase1 += two_pi * f1 / fs;
    phase2 += two_pi * f2 / fs;

    if (phase1 >= two_pi)
    {
      phase1 -= two_pi;
    }
    if (phase2 >= two_pi)
    {
      phase2 -= two_pi;
    }
  }
}

static float32_t Phase_Get_TIM3_TriggerFreq_Hz(void)
{
  uint32_t timclk_hz;

  /* TIM3 clocked by Si5351 CLK0 via ETR pin (external clock mode 2 + ECE) */
  if ((TIM3->SMCR & TIM_SMCR_ECE) != 0U)
  {
    timclk_hz = 1024000U; /* Si5351 CLK0, same as main.c setting */
  }
  else
  {
    uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
    uint32_t apb1_div = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
    timclk_hz = pclk1_hz;
    if (apb1_div >= 4U)
    {
      timclk_hz = pclk1_hz * 2U;
    }
  }

  uint32_t psc = (uint32_t)(htim3.Init.Prescaler + 1U);
  uint32_t arr = (uint32_t)(htim3.Init.Period + 1U);
  return (float32_t)timclk_hz / (float32_t)(psc * arr);
}

/* 全局变量定义 */

/*
  * @brief ADC参考电压（单位：V）
  */
float32_t Reference_Voltage = 3.3f;

/**
 * @brief ADC采集完成标志位
 * @note  在ADC DMA传输完成回调函数中置1
 */
volatile uint8_t ADC_COMPLETED = 0;
volatile int max_index_watch;

/**
 * @brief ADC DMA 原始数据缓冲区
 * @note  存储ADC采集的原始数据
 */
uint16_t ADC_Buffer[FFT_LENGTH] = {0};

/**
 * @brief ADC采集标志位
 * @note  1表示采集完成，可以处理数据
 */
volatile uint8_t ADC_Flag = 0;

/**
 * @brief FFT转换所需的浮点数缓冲区
 * @note  用于存储转换后的电压值，作为FFT输入
 */
static float32_t adc_float_buffer[FFT_LENGTH];

/**
 * @brief FFT输入缓冲区（复数格式）
 * @note  格式：[实部0, 虚部0, 实部1, 虚部1, ...]
 */
float32_t FFT_InputBuf[FFT_LENGTH * 2] = {0.0f};

/**
 * @brief FFT输出缓冲区（模值）
 * @note  存储FFT运算后的幅值
 */
float32_t FFT_OutputBuf[FFT_LENGTH] = {0.0f};

/**
 * @brief FFT变换标志位
 */
uint8_t ifftFlag = 0;

/**
 * @brief 位反转标志位
 */
uint8_t doBitReverse = 1;

/**
 * @brief 窗函数变量
 */
float32_t window;

/* 内部静态变量，用于控制输出频率 */
static const arm_cfft_instance_f32 *CFFT_Instance = NULL;
static uint8_t g_fft_channel_configured = 0;

void FFT_ResetChannelConfig(void)
{
    g_fft_channel_configured = 0;
}

static void Phase_ComputeThdAndFft(void)
{
  uint64_t adc_sum = 0;
  for (uint32_t i = 0; i < FFT_LENGTH; i++)
  {
    adc_sum += adc_snapshot[i];
  }
  float32_t adc_mean = (float32_t)adc_sum / (float32_t)FFT_LENGTH;

  for (uint32_t i = 0; i < FFT_LENGTH; i++)
  {
    float32_t centered = ((float32_t)adc_snapshot[i] - adc_mean) / (float32_t)ADC_MaxValue;
    adc_float_buffer[i] = centered * Reference_Voltage;
  }

  {
    static float32_t win_coeff[FFT_LENGTH];
    static uint8_t win_init_done = 0;
    if (!win_init_done)
    {
      hannWin(FFT_LENGTH, win_coeff);
      win_init_done = 1;
    }
    Window_Apply(adc_float_buffer, win_coeff, FFT_LENGTH);
  }

  arm_cfft_f32_app(adc_float_buffer, CFFT_Instance);

  /* Normalize FFT magnitude to actual voltage amplitude:
   *   CMSIS CFFT does NOT scale by 1/N → divide by N/2
   *   Hanning window coherent gain = 0.5 → divide by CG
   *   Combined: scale = 2/(N*0.5) = 2/(4096*0.5) ≈ 0.0009766 */
  {
    const float32_t scale = 2.0f / ((float32_t)FFT_LENGTH * 0.5f);
    for (uint32_t i = 0; i < FFT_LENGTH; i++)
    {
      FFT_OutputBuf[i] *= scale;
    }
  }

  memcpy(fft_mag_snapshot, FFT_OutputBuf, sizeof(fft_mag_snapshot));
}

/* 双ADC相关变量（保留以兼容现有逻辑，若未使用可移除） */
uint32_t ADC_Raw_Data[FFT_LENGTH] = {0};
uint16_t ADC_1_Value_DMA[FFT_LENGTH] = {0};
uint16_t ADC_2_Value_DMA[FFT_LENGTH] = {0};
float32_t ADC_1_Real_Value[FFT_LENGTH] = {0.0f};
float32_t ADC_2_Real_Value[FFT_LENGTH] = {0.0f};


/**
 * @brief  FFT应用初始化
 * @retval None
 * @note   初始化定时器和启动首次ADC DMA采集
 */
void FFT_App_Init(void)
{
    // 根据宏定义选择FFT实例
    #if FFT_LENGTH == 64
        CFFT_Instance = &arm_cfft_sR_f32_len64;
    #elif FFT_LENGTH == 128
        CFFT_Instance = &arm_cfft_sR_f32_len128;
    #elif FFT_LENGTH == 256
        CFFT_Instance = &arm_cfft_sR_f32_len256;
    #elif FFT_LENGTH == 512
        CFFT_Instance = &arm_cfft_sR_f32_len512;
    #elif FFT_LENGTH == 1024
        CFFT_Instance = &arm_cfft_sR_f32_len1024;
    #elif FFT_LENGTH == 2048
        CFFT_Instance = &arm_cfft_sR_f32_len2048;
    #elif FFT_LENGTH == 4096
        CFFT_Instance = &arm_cfft_sR_f32_len4096;
    #else
        #error "Unsupported FFT Length! Please check Phase.h"
    #endif

    // 已移除Si5351相关初始化，采样时钟由MCU内部或外部其他源提供

#ifdef FFT_TEST_SIMULATION
    FFT_SamplingRate_Hz = SAMPLING_RATE_DEFAULT;
    ADC_Flag = 1;
    return;
#endif

    // 配置ADC通道为PC4 (ADC1_INP4)
    SCAN_ConfigAdc1ToFFTInput();

    // 配置TIM3时钟源和采样周期
#if PHASE_CLOCK_SOURCE == PHASE_CLK_INTERNAL
    {
      uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
      uint32_t tim_clk = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) ? (pclk1 * 2U) : pclk1;
      uint32_t period = (uint32_t)((float)tim_clk / PHASE_TARGET_FS + 0.5f);
      if (period < 2U) period = 2U;
      htim3.Init.Period = period - 1U;
      TIM3->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_ECE);
    }
#else
    {
      uint32_t period = (uint32_t)(1024000.0f / PHASE_TARGET_FS + 0.5f);
      if (period < 2U) period = 2U;
      htim3.Init.Period = period - 1U;
      TIM3->SMCR |= TIM_SMCR_ECE;
    }
#endif
    __HAL_TIM_SET_AUTORELOAD(&htim3, htim3.Init.Period);
    FFT_SamplingRate_Hz = Phase_Get_TIM3_TriggerFreq_Hz();

    // 启动定时器（用于触发ADC）
    Phase_StartAcq();
}

void FFT_SendSpectrumFrame(void)
{
  if (g_frame_ready == 0U)
  {
    return;
  }
  g_transmitting = 1U;
  g_frame_ready = 0U;

  Phase_ComputeThdAndFft();

  {
    int32_t fs_int = (int32_t)FFT_SamplingRate_Hz;
    uint32_t fs_frac = (uint32_t)((FFT_SamplingRate_Hz - (float)fs_int) * 100.0f + 0.5f);
    char header[64];
    int header_len = snprintf(header, sizeof(header),
                              "FFT_BEGIN,Fs=%ld.%02lu,N=%u\r\n",
                              (long)fs_int, (unsigned long)fs_frac,
                              (unsigned)FFT_LENGTH);
    if (header_len > 0)
    {
      while (UART1_TxEnqueue((const uint8_t *)header, (uint16_t)header_len) < 0) {}
    }
  }

  {
    char line[64];
    uint32_t halfN = FFT_LENGTH / 2U;
    for (uint32_t i = 0; i < halfN; i++)
    {
      float val = fft_mag_snapshot[i];
      if (val < 0.0f) val = 0.0f;
      int32_t int_part = (int32_t)val;
      uint32_t frac_part = (uint32_t)((val - (float)int_part) * 1000000.0f + 0.5f);
      if (frac_part >= 1000000U) { int_part++; frac_part = 0U; }
      int len = snprintf(line, sizeof(line), "%lu,%ld.%06lu\r\n",
                         (unsigned long)i, (long)int_part, (unsigned long)frac_part);
      if (len > 0)
      {
        while (UART1_TxEnqueue((const uint8_t *)line, (uint16_t)len) < 0) {}
      }
    }
  }

  {
    static const char tail[] = "FFT_END\r\n";
    while (UART1_TxEnqueue((const uint8_t *)tail, (uint16_t)(sizeof(tail) - 1U)) < 0) {}
  }

  g_transmitting = 0U;
}

static uint8_t SCAN_ConfigAdc1ToFFTInput(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_4; // PC4 (ADC1_INP4)
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  return (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK) ? 1U : 0U;
}

/**
 * @brief  FFT应用主处理函数
 * @retval None
 * @note   需在主循环(while(1))中持续调用。
 *         负责检查采集状态、数据转换、FFT运算及结果输出。
 */
void FFT_App_Process(void)
{
    // 首次进入或从其他模式切回时，确保通道配置为 PC4 (ADC_CHANNEL_4)
    if (!g_fft_channel_configured)
    {
        HAL_ADC_Stop_DMA(&hadc1);
        SCAN_ConfigAdc1ToFFTInput();
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Buffer, FFT_LENGTH);
        g_fft_channel_configured = 1;
    }
    
#ifdef FFT_TEST_SIMULATION
    static uint32_t last_sim_tick = 0;
    if (HAL_GetTick() - last_sim_tick < 1000)
    {
      return;
    }
    last_sim_tick = HAL_GetTick();

    Phase_GenerateSimulatedSignal(adc_float_buffer, FFT_LENGTH);
    arm_cfft_f32_app(adc_float_buffer, CFFT_Instance);
    return;
#endif

    if(ADC_Flag)
    {
        // 清除标志位，准备下一次采集
        ADC_Flag = 0;

      // H7单端采样常带直流偏置，FFT前先去均值避免低频/DC抬高频谱底噪。
      uint64_t adc_sum = 0;
      for (int i = 0; i < FFT_LENGTH; i++)
      {
        adc_sum += ADC_Buffer[i];
      }
      float32_t adc_mean = (float32_t)adc_sum / (float32_t)FFT_LENGTH;
        
        // 1. 数据预处理：将ADC原始值转换为电压值（浮点）
        for(int i = 0; i < FFT_LENGTH; i++)
        {
        float32_t centered = ((float32_t)ADC_Buffer[i] - adc_mean) / (float32_t)ADC_MaxValue;
        adc_float_buffer[i] = centered * Reference_Voltage;
            
            // 如果需要加窗，可以在此处调用 Apply_Hanning_Window
            //Apply_Hanning_Window(adc_float_buffer, FFT_LENGTH); 
        }

        // 2. 执行FFT变换
        // 注意：arm_cfft_f32_app 内部会处理 FFT_InputBuf 和 FFT_OutputBuf
        arm_cfft_f32_app(adc_float_buffer, CFFT_Instance);
        
        // 4. 重新启动ADC DMA采集
        // 注意：在ADC采集完成后已经调用了 HAL_ADC_Stop_DMA
        // 这里始终启动下一次采集，不再等待 100ms
        // 在启动 DMA 前先设置标志位为 0，确保下一次中断能正确触发逻辑
        ADC_Flag = 0;
        
        // 重要：H7 在重新启动 DMA 前，有时需要清除过载错误标志
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR); 
        
        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Buffer, FFT_LENGTH) != HAL_OK)
        {
            // 如果开启失败，可能发生了状态位死锁，尝试强行重置通道配置
            HAL_ADC_Stop_DMA(&hadc1);
            g_fft_channel_configured = 0;
        }
    }
}


/**
 * @brief  FFT变换核心函数
 * @param  rawData: 输入实数数组
 * @param  fft_instance: FFT配置实例
 * @retval None
 */
void arm_cfft_f32_app(float32_t *rawData, const arm_cfft_instance_f32 *fft_instance)
{
  uint16_t n;
  uint16_t fftLen = fft_instance->fftLen;
  
  /* 1. 构建复数输入数组 */
  for (n = 0; n < fftLen; n++)
  {
    FFT_InputBuf[2 * n] = rawData[n];      // 实部
    FFT_InputBuf[2 * n + 1] = 0.0f;        // 虚部为0
  }
  
  /* 2. 执行FFT变换 */
  arm_cfft_f32(fft_instance, FFT_InputBuf, ifftFlag, doBitReverse);
  
  /* 3. 计算模值 */
  arm_cmplx_mag_f32(FFT_InputBuf, FFT_OutputBuf, fftLen);
}


/**
 * @brief  应用汉宁窗
 * @param  signal: 信号数组
 * @param  length: 长度
 */
void Apply_Hanning_Window(float32_t *signal, uint16_t length)
{
    for(uint16_t i = 0; i < length; i++) {
       window = 0.5f * (1.0f - cosf(2.0f * 3.1415926f * i / (length - 1)));
       signal[i] *= window;
    }
}

/**
 * @brief  应用4项平顶窗 (Flat-Top Window)
 * @note   幅值精度 < 0.02 dB，专门用于精确谐波幅值测量
 *         系数来源: ISO 18431-2 / IEEE 1057
 *         相干增益 ≈ 0.2156
 */
void Apply_FlatTop_Window(float32_t *signal, uint16_t length)
{
    const float32_t a0 = 0.21557895f;
    const float32_t a1 = 0.41663158f;
    const float32_t a2 = 0.277263158f;
    const float32_t a3 = 0.083578947f;
    const float32_t denom = (float32_t)(length - 1);

    for (uint16_t i = 0; i < length; i++)
    {
        float32_t ratio = (float32_t)i / denom;
        float32_t theta = 2.0f * 3.1415926f * ratio;
        window = a0 - a1 * cosf(theta) + a2 * cosf(2.0f * theta) - a3 * cosf(3.0f * theta);
        signal[i] *= window;
    }
}


/* 以下为原有双ADC相位计算相关函数，保留备用 */

void PhaseCalculate_ADC_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2)
{
#ifdef USE_ADC_CALIBRATION
  HAL_ADCEx_Calibration_Start(hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
#endif
  HAL_ADC_Start(hadc2);
  HAL_ADC_Start_DMA(hadc1, (uint32_t *)ADC_1_Value_DMA, FFT_LENGTH);
  HAL_ADC_Start_DMA(hadc2, (uint32_t *)ADC_2_Value_DMA, FFT_LENGTH);
}

int Find_nMax(const float *ARR)
{
  if (ARR == NULL) return -1;
  float aMax = ARR[80];
  uint32_t nMax = 80;
  for (uint32_t i = 81; i < FFT_LENGTH / 2; i++)
  {
    if (ARR[i] > aMax)
    {
      aMax = ARR[i];
      nMax = i;
    }
  }
  return nMax;
}

float32_t Find_PhaseAngle(float32_t *signal)
{
  arm_cfft_f32_app(signal, &arm_cfft_sR_f32_len1024);
  int n_max = Find_nMax(FFT_OutputBuf);
  float32_t phase_rad = atan2f(FFT_InputBuf[2 * n_max + 1], FFT_InputBuf[2 * n_max]);
  return phase_rad * 180.0f / PI;
}

void Process_ADC_RawData(void)
{
  if (ADC_COMPLETED)
  {
    ADC_COMPLETED = 0;
    for (int i = 0; i < FFT_LENGTH; i++)
    {
      uint16_t adc1 = ADC_1_Value_DMA[i];
      uint16_t adc2 = ADC_2_Value_DMA[i];
      ADC_1_Real_Value[i] = ((float32_t)adc1 / (float32_t)ADC_MaxValue) * Reference_Voltage;
      ADC_2_Real_Value[i] = ((float32_t)adc2 / (float32_t)ADC_MaxValue) * Reference_Voltage;
    }
  }
}

float32_t Get_PhaseDifference(void)
{
  float32_t phase1, phase2, phase_diff;
  Process_ADC_RawData();
  phase1 = Find_PhaseAngle(ADC_1_Real_Value);
  phase2 = Find_PhaseAngle(ADC_2_Real_Value);
  phase_diff = phase1 - phase2;
  if (phase_diff > 180.0f) phase_diff -= 360.0f;
  else if (phase_diff < -180.0f) phase_diff += 360.0f;
  return phase_diff;
}

/**
 * @brief  ADC转换完成回调函数
 * @param  hadc: ADC句柄指针
 * @note   由HAL库中断处理函数调用
 */
volatile uint32_t g_cb_count = 0;
volatile uint32_t g_fft_step = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
      g_cb_count++;
      HAL_ADC_Stop_DMA(hadc);

      if (g_transmitting == 0U && g_frame_ready == 0U)
      {
        memcpy(adc_snapshot, ADC_Buffer, sizeof(adc_snapshot));
        g_frame_ready = 1U;
      }

      __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
      (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Buffer, FFT_LENGTH);
      ADC_Flag = 1;
  }
  
  // 双ADC逻辑保留
  static uint8_t adc1_complete = 0;
  static uint8_t adc2_complete = 0;
  
  if (hadc->Instance == ADC1) adc1_complete = 1;
  else if (hadc->Instance == ADC2) adc2_complete = 1;
    
  if (adc1_complete && adc2_complete)
  {
    ADC_COMPLETED = 1;
    adc1_complete = 0;
    adc2_complete = 0;
  }
}

uint16_t Get_FFT_Spectrum(float32_t* buffer, uint16_t length)
{
  uint16_t copy_length = (length < FFT_LENGTH) ? length : FFT_LENGTH;
  memcpy(buffer, FFT_OutputBuf, copy_length * sizeof(float32_t));
  return copy_length;
}

void Set_Reference_Voltage(float32_t voltage)
{
  if (voltage > 0.0f && voltage <= 5.0f)
  {
    Reference_Voltage = voltage;
  }
}

float32_t Get_Reference_Voltage(void)
{
  return Reference_Voltage;
}

void ADC_Signal_Collect_To_ADC_Buffer(void)
{
    // 此函数逻辑已整合到 FFT_App_Process 中，保留此空函数或重定向，以防其他地方调用
    // 或者直接在这里调用 Process
    // 但为避免递归或混乱，建议弃用此函数，改用 FFT_App_Process
}
