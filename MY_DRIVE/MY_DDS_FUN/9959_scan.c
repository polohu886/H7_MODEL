/* 9959_scan.c - Sweep implementation using AD9959 control and ADC measurement */
#include "9959_scan.h"
#include "ad9959.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "delay.h"
#include "ZPN_Hmi.h"
#include "ZPN_Uart.h"
#include <stdio.h>

#define SCAN_UART_LOG_ENABLE 1

#if SCAN_UART_LOG_ENABLE
#define SCAN_LOG(...) printf(__VA_ARGS__)
#else
#define SCAN_LOG(...)
#endif

extern ADC_HandleTypeDef hadc1;
// extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;

/* Internal sweep buffers */
static uint32_t g_sweep_freq[501U];
static uint16_t g_sweep_level[501U];
static uint16_t g_sweep_phase[501U];

#define SCAN_LOG_LINE_BUF_SIZE 256U

static void SCAN_WaitUart1Idle(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  while (!UART1_TxIsIdle())
  {
    if ((HAL_GetTick() - t0) > timeout_ms)
    {
      break;
    }
  }
}

static void SCAN_LogAmplitudeTable(void)
{
  char line[SCAN_LOG_LINE_BUF_SIZE];
  uint16_t pos = 0U;

  SCAN_LOG("Amplitude Profile (ADC Raw):\r\n");

  for (uint16_t i = 0; i < SWEEP_POINT_COUNT; i++)
  {
    int n = snprintf(&line[pos], (size_t)(SCAN_LOG_LINE_BUF_SIZE - pos),
                     "[%lu,%u]%s",
                     (unsigned long)g_sweep_freq[i],
                     (unsigned int)g_sweep_level[i],
                     (i == SWEEP_POINT_COUNT - 1) ? "" : ",");

    if (n < 0)
    {
      continue;
    }

    if ((uint16_t)n >= (SCAN_LOG_LINE_BUF_SIZE - pos))
    {
      line[pos] = '\0';
      SCAN_LOG("%s", line);
      SCAN_WaitUart1Idle(1000U);
      pos = 0U;

      n = snprintf(&line[pos], (size_t)(SCAN_LOG_LINE_BUF_SIZE - pos),
                   "[%lu,%u]%s",
                   (unsigned long)g_sweep_freq[i],
                   (unsigned int)g_sweep_level[i],
                   (i == SWEEP_POINT_COUNT - 1) ? "" : ",");
      if (n < 0)
      {
        continue;
      }
    }

    pos = (uint16_t)(pos + (uint16_t)n);

    /* 每累计一段就分发，避免 UART1 队列溢出 */
    if (pos > 180U || i == (SWEEP_POINT_COUNT - 1U))
    {
      line[pos] = '\0';
      SCAN_LOG("%s\r\n", line);
      SCAN_WaitUart1Idle(1000U);
      pos = 0U;
    }
  }
}

static uint8_t SCAN_CalibrateAdcOnce(ADC_HandleTypeDef *hadc)
{
  static uint8_t calibrated = 0U;

  if (calibrated)
  {
    return 1U;
  }

  if (HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    return 0U;
  }

  calibrated = 1U;
  return 1U;
}

static uint8_t SCAN_ConfigAdc1ToLearnInput(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  return (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK) ? 1U : 0U;
}

static uint16_t SCAN_ReadAdcAverage(ADC_HandleTypeDef *hadc, uint16_t sample_count)
{
  if (sample_count == 0U)
  {
    sample_count = 1U;
  }

  uint32_t sum = 0U;
  uint16_t valid = 0U;

  for (uint16_t n = 0U; n < sample_count; n++)
  {
    if (HAL_ADC_Start(hadc) != HAL_OK)
    {
      continue;
    }

    if (HAL_ADC_PollForConversion(hadc, 20U) != HAL_OK)
    {
      HAL_ADC_Stop(hadc);
      continue;
    }

    sum += (uint16_t)HAL_ADC_GetValue(hadc);
    valid++;
    HAL_ADC_Stop(hadc);
  }

  if (valid == 0U)
  {
    return 0U;
  }

  return (uint16_t)(sum / valid);
}

uint16_t SCAN_LookupLevelForFreq(uint32_t freq_hz)
{
  uint16_t idx = 0;
  if (freq_hz <= SWEEP_START_FREQ_HZ) return g_sweep_level[0];
  
#ifdef SWEEP_STEP_HZ
  idx = (freq_hz - SWEEP_START_FREQ_HZ) / SWEEP_STEP_HZ;
#else
  if (SWEEP_STOP_FREQ_HZ > SWEEP_START_FREQ_HZ) {
    idx = (uint16_t)((uint64_t)(freq_hz - SWEEP_START_FREQ_HZ) * (SWEEP_POINT_COUNT - 1U) / (SWEEP_STOP_FREQ_HZ - SWEEP_START_FREQ_HZ));
  }
#endif

  if (idx >= SWEEP_POINT_COUNT) idx = SWEEP_POINT_COUNT - 1;
  return g_sweep_level[idx];
}

const char *SCAN_ModelTypeToString(uint8_t model_type)
{
  switch (model_type)
  {
  case FILTER_MODEL_LOWPASS:
    return "LOWPASS";
  case FILTER_MODEL_HIGHPASS:
    return "HIGHPASS";
  case FILTER_MODEL_BANDPASS:
    return "BANDPASS";
  case FILTER_MODEL_BANDSTOP:
    return "BANDSTOP";
  default:
    return "UNKNOWN";
  }
}

uint8_t SCAN_RunAndExtract(FilterFeature_t *feature)
{
  uint32_t peak_freq_hz = 0U;
  uint16_t peak_idx = 0;
  uint16_t peak_level = 0;
  uint16_t min_idx = 0;
  uint16_t min_level = 0xFFFFU;

  if (feature == NULL) return 0U;
  if (SWEEP_POINT_COUNT < 5U || SWEEP_STOP_FREQ_HZ <= SWEEP_START_FREQ_HZ) return 0U;

  /*
   * 独占 ADC1：避免与 FFT_App_Init() 启动的 ADC1 DMA 采样冲突。
   * 如果不先停掉 DMA，这里读到的可能是其他通道/旧数据，导致接地仍有偏置。
   */
  (void)HAL_ADC_Stop_DMA(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  
  // 确保 FFT 相关的标志位在扫频开始前清除，防止扫频结束后 FFT 逻辑误触发
  extern volatile uint8_t ADC_Flag;
  ADC_Flag = 0;

  HAL_TIM_Base_Start(&htim3);
  if (SCAN_ConfigAdc1ToLearnInput() == 0U)
  {
    return 0U;
  }

  if (SCAN_CalibrateAdcOnce(&hadc1) == 0U)
  {
    return 0U;
  }

  Write_Amplitude(SWEEP_CHANNEL, (uint16_t)(SWEEP_OUTPUT_AMPLITUDE_CODE * 2.02f / 1.86f));
  Write_Phase(SWEEP_CHANNEL, 0U);
  AD9959_IO_Update();

  // ADC2_SelectChannel(ADC_CHANNEL_1);

  for (uint16_t i = 0; i < SWEEP_POINT_COUNT; i++)
  {
    /* Compute frequency per step. Prefer fixed step if SWEEP_STEP_HZ is defined. */
  #ifdef SWEEP_STEP_HZ
    uint32_t frequency_hz = SWEEP_START_FREQ_HZ + (uint32_t)((uint64_t)SWEEP_STEP_HZ * i);
  #else
    uint32_t frequency_hz = SWEEP_START_FREQ_HZ + (uint32_t)(((uint64_t)(SWEEP_STOP_FREQ_HZ - SWEEP_START_FREQ_HZ) * i) / (SWEEP_POINT_COUNT - 1U));
  #endif
    g_sweep_freq[i] = frequency_hz;
    Write_Frequence(SWEEP_CHANNEL, frequency_hz);
    AD9959_IO_Update();

    /* 在 1kHz 频点额外停留 1s */
    if (frequency_hz == 1000U)
    {
      HAL_Delay(2000U);
    }

    HAL_Delay(SWEEP_SETTLE_MS);
    /* 每次改频后，获取幅度和相位 ADC 值。 */
    g_sweep_level[i] = SCAN_ReadAdcAverage(&hadc1, SWEEP_SAMPLES_PER_POINT);

    g_sweep_phase[i] = 0; // 屏蔽缺失的 hadc2

    if (g_sweep_level[i] > peak_level)
    {
      peak_level = g_sweep_level[i];
      peak_idx = i;
      peak_freq_hz = frequency_hz;
    }

    if (g_sweep_level[i] < min_level)
    {
      min_level = g_sweep_level[i];
      min_idx = i;
    }
  }

  if (peak_level == 0U)
  {
    SCAN_LOG("[SWEEP] fail: peak level is zero\r\n");
    return 0U;
  }

  SCAN_LOG("[SWEEP] adc raw min=%u max=%u\r\n", (unsigned int)min_level, (unsigned int)peak_level);

  SCAN_LOG("[SWEEP] table begin, points=%u\r\n", (unsigned int)SWEEP_POINT_COUNT);
  SCAN_LogAmplitudeTable();

  SCAN_LOG("[SWEEP] phase profile disabled: ADC2 phase channel not enabled\r\n");
  SCAN_LOG("[SWEEP] table end\r\n");

  /* --- 1. 滤波器特征提取逻辑 --- */
  uint16_t start_level = g_sweep_level[0];
  uint16_t end_level = g_sweep_level[SWEEP_POINT_COUNT - 1];
  uint16_t mid_level = g_sweep_level[SWEEP_POINT_COUNT / 2];
  uint16_t left_max = 0U;
  uint16_t right_max = 0U;

  for (uint16_t i = 0U; i < min_idx; i++)
  {
    if (g_sweep_level[i] > left_max)
    {
      left_max = g_sweep_level[i];
    }
  }

  for (uint16_t i = (uint16_t)(min_idx + 1U); i < SWEEP_POINT_COUNT; i++)
  {
    if (g_sweep_level[i] > right_max)
    {
      right_max = g_sweep_level[i];
    }
  }

  uint8_t notch_in_middle = (min_idx > SWEEP_EDGE_AVG_POINTS) && (min_idx < (SWEEP_POINT_COUNT - SWEEP_EDGE_AVG_POINTS));
  uint8_t deep_notch = (min_level < (uint16_t)(peak_level * 0.45f));
  uint8_t two_side_recovery = (left_max > (uint16_t)(min_level * 1.8f)) && (right_max > (uint16_t)(min_level * 1.8f));
  uint8_t right_side_rebound = (end_level > (uint16_t)(min_level * 1.4f));

  if (peak_level < 50U) {
    feature->model_type = FILTER_MODEL_UNKNOWN;
  } else if (notch_in_middle && deep_notch && two_side_recovery && right_side_rebound) {
    /* 带阻优先：中间有明显深陷，且谷底两侧均有回升 */
    feature->model_type = FILTER_MODEL_BANDSTOP;
  } else if (start_level > peak_level * 0.7f && end_level < peak_level * 0.3f) {
    feature->model_type = FILTER_MODEL_LOWPASS;
  } else if (start_level < peak_level * 0.3f && end_level > peak_level * 0.7f) {
    feature->model_type = FILTER_MODEL_HIGHPASS;
  } else if (peak_idx > SWEEP_EDGE_AVG_POINTS && peak_idx < (SWEEP_POINT_COUNT - SWEEP_EDGE_AVG_POINTS) &&
             start_level < peak_level * 0.5f && end_level < peak_level * 0.5f) {
    feature->model_type = FILTER_MODEL_BANDPASS;
  } else if (mid_level < start_level * 0.5f && mid_level < end_level * 0.5f) {
    feature->model_type = FILTER_MODEL_BANDSTOP;
  } else {
    feature->model_type = FILTER_MODEL_UNKNOWN;
  }

  feature->valid = 1U;
  feature->f0_hz = peak_freq_hz;
  feature->peak_level = peak_level;
  feature->min_level = min_level;

  return 1U;
}
