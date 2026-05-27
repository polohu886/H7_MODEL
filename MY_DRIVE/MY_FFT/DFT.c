/**
 ******************************************************************************
 * @file    DFT.c
 * @brief   离散傅里叶变换谐波分析
 * @author  POLO_HU
 * @version V1.1
 * @date    2026-05-23
 * @note    流程: 粗FFT找基波 → 频点精修 → DFT精确计算1-10次谐波幅值 → THD
 *          V1.1: 基波搜索范围改为可配置参数
 ******************************************************************************
 */

#include "DFT.h"
#include "arm_const_structs.h"
#include "WindowFunction.h"
#include "adc.h"
#include "tim.h"
#include <math.h>
#include <string.h>

/* ==================== 静态变量 ==================== */
static float32_t g_fs = 64000.0f;
static float32_t g_vref = 3.3f;
static float32_t g_search_min = DFT_SEARCH_MIN_HZ;
static float32_t g_search_max = DFT_SEARCH_MAX_HZ;
static float32_t g_fund_freq = 0.0f;
static float32_t g_thd = 0.0f;
static float32_t g_mags[DFT_MAX_HARMONIC + 1] = {0.0f}; /* [0]空，[1..10]各次谐波DFT幅值 */

/* DMA缓冲区（驱动内部管理） */
static uint16_t dft_adc_buf[DFT_FFT_LENGTH];
static uint16_t dft_snapshot[DFT_FFT_LENGTH];
static volatile uint8_t dft_snapshot_ready = 0;

#ifndef USE_FFT  /* DFT/FFT互斥: CMakeLists.txt定义USE_FFT时跳过 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    HAL_ADC_Stop_DMA(hadc);
    if (!dft_snapshot_ready)
    {
      memcpy(dft_snapshot, dft_adc_buf, sizeof(dft_snapshot));
      dft_snapshot_ready = 1;
    }
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dft_adc_buf, DFT_FFT_LENGTH);
  }
}
#endif

/* FFT工作缓冲 */
static float32_t fft_in[DFT_FFT_LENGTH * 2];
static float32_t fft_mag[DFT_FFT_LENGTH];

/* 信号缓冲（去直流后的电压值） */
static float32_t sig_buf[DFT_FFT_LENGTH];

/* 诊断：FFT峰检测原始数据 */
static uint32_t g_diag_peak_bin = 0;
static float32_t g_diag_y0 = 0, g_diag_y1 = 0, g_diag_y2 = 0;

/* ==================== 内部函数 ==================== */

/**
 * @brief  单频点DFT幅值计算
 * @note   直接累加，不依赖CMSIS-DSP
 */
static float32_t dft_single(const float32_t *x, uint32_t N, float32_t freq)
{
  float32_t re = 0.0f, im = 0.0f;
  float32_t omega = 2.0f * 3.14159265358979f * freq / g_fs;

  for (uint32_t n = 0; n < N; n++)
  {
    float32_t theta = omega * (float32_t)n;
    re += x[n] * cosf(theta);
    im -= x[n] * sinf(theta);
  }

  return sqrtf(re * re + im * im);
}

/**
 * @brief  一次遍历计算1-10次谐波的DFT幅值
 * @note   用三角递推避免重复计算cos/sin，提高效率
 */
static void dft_harmonics_all(const float32_t *x, uint32_t N,
                              float32_t f_fund, float32_t *mags)
{
  float32_t re[DFT_MAX_HARMONIC + 1] = {0.0f};
  float32_t im[DFT_MAX_HARMONIC + 1] = {0.0f};
  float32_t omega = 2.0f * 3.14159265358979f * f_fund / g_fs;

  /* 生成Hanning窗 (一次) */
  static float32_t win[DFT_FFT_LENGTH];
  static uint8_t win_ready = 0;
  if (!win_ready) {
    hannWin(DFT_FFT_LENGTH, win);
    win_ready = 1;
  }

  for (uint32_t n = 0; n < N; n++)
  {
    float32_t theta = omega * (float32_t)n;
    float32_t c = cosf(theta);
    float32_t s = sinf(theta);

    float32_t ch = 1.0f, sh = 0.0f;
    float32_t sample = x[n] * win[n];  /* 加窗抑制旁瓣泄漏 */

    for (uint32_t h = 1; h <= DFT_MAX_HARMONIC; h++)
    {
      float32_t ch_new = ch * c - sh * s;
      float32_t sh_new = sh * c + ch * s;
      ch = ch_new;
      sh = sh_new;

      re[h] += sample * ch;
      im[h] -= sample * sh;
    }
  }

  /* Hanning窗相干增益=0.5, 幅值修正: 2.0/(N*0.5) = 4.0/N */
  for (uint32_t h = 1; h <= DFT_MAX_HARMONIC; h++)
  {
    mags[h] = sqrtf(re[h] * re[h] + im[h] * im[h]) * 4.0f / (float32_t)N;
  }
}

/**
 * @brief  粗FFT + 频点精修：找到准确的基波频率
 * @return 基波频率(Hz)
 */
static float32_t find_fundamental(void)
{
  static float32_t window[DFT_FFT_LENGTH];
  static uint8_t win_ready = 0;
  if (!win_ready) {
    hannWin(DFT_FFT_LENGTH, window);
    win_ready = 1;
  }

  /* 1. 准备FFT输入: 加Hanning窗 → 抑制谐波泄漏 */
  for (uint32_t i = 0; i < DFT_FFT_LENGTH; i++)
  {
    fft_in[2 * i] = sig_buf[i] * window[i];
    fft_in[2 * i + 1] = 0.0f;
  }

  /* 2. 执行FFT */
  arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_in, 0, 1);

  /* 3. 计算幅值 */
  arm_cmplx_mag_f32(fft_in, fft_mag, DFT_FFT_LENGTH);

  /* 4. 在搜索范围内找最大峰 */
  uint32_t half = DFT_FFT_LENGTH / 2U;
  uint32_t min_idx = (uint32_t)((g_search_min * (float32_t)DFT_FFT_LENGTH) / g_fs + 0.5f);
  uint32_t max_idx = (uint32_t)((g_search_max * (float32_t)DFT_FFT_LENGTH) / g_fs + 0.5f);
  if (min_idx < 1U) min_idx = 1U;
  if (max_idx > half - 1U) max_idx = half - 1U;
  if (min_idx > max_idx) min_idx = max_idx;

  float32_t peak = 0.0f;
  uint32_t peak_idx = min_idx;
  for (uint32_t i = min_idx; i <= max_idx; i++)
  {
    if (fft_mag[i] > peak)
    {
      peak = fft_mag[i];
      peak_idx = i;
    }
  }

  /* 5. 抛物线插值精修频率 (3点: y0,y1,y2 → 峰在 offset=-b/(2a)) */
  {
    float32_t y0 = (peak_idx > 0) ? fft_mag[peak_idx - 1] : 0.0f;
    float32_t y1 = fft_mag[peak_idx];
    float32_t y2 = fft_mag[peak_idx + 1];

    /* 保存诊断数据 */
    g_diag_peak_bin = peak_idx;
    g_diag_y0 = y0;
    g_diag_y1 = y1;
    g_diag_y2 = y2;

    float32_t denom = 2.0f * (2.0f * y1 - y0 - y2);
    float32_t delta = 0.0f;
    if (denom > 1e-12f)
    {
      delta = (y2 - y0) / denom;  /* δ = -b/(2a) */
      if (delta > 0.5f) delta = 0.5f;
      if (delta < -0.5f) delta = -0.5f;
    }
    return ((float32_t)peak_idx + delta) * g_fs / (float32_t)DFT_FFT_LENGTH;
  }
}

/* ==================== 公开接口 ==================== */

void DFT_App_Init(float32_t sampling_rate, float32_t ref_voltage,
                  float32_t search_min_hz, float32_t search_max_hz)
{
  /* 保存参数 */
  g_vref = ref_voltage;
  g_search_min = (search_min_hz > 0.0f) ? search_min_hz : DFT_SEARCH_MIN_HZ;
  g_search_max = (search_max_hz > 0.0f) ? search_max_hz : DFT_SEARCH_MAX_HZ;
  memset(g_mags, 0, sizeof(g_mags));
  g_fund_freq = 0.0f;
  g_thd = 0.0f;

  /* 配置ADC1通道 → PC4 (延长采样时间以保证16-bit精度) */
  {
    ADC_ChannelConfTypeDef s = {0};
    s.Channel = ADC_CHANNEL_4;
    s.Rank = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;  /* 8.5 ADC cycles */
    s.SingleDiff = ADC_SINGLE_ENDED;
    s.OffsetNumber = ADC_OFFSET_NONE;
    HAL_ADC_ConfigChannel(&hadc1, &s);
  }

  /* 配置TIM3时钟源和采样周期，回算实际采样率写入g_fs */
  {
    uint32_t period;
#if DFT_CLOCK_SOURCE == DFT_CLK_INTERNAL
    /* APB1 Timer Clock = PCLK1 * 2 (when APB1 prescaler ≠ 1) */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t tim_clk = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) ? (pclk1 * 2U) : pclk1;
    period = tim_clk / (uint32_t)sampling_rate;
    if (period < 2U) period = 2U;
    g_fs = (float32_t)tim_clk / (float32_t)period;  /* 实际采样率，非目标值 */
    TIM3->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_ECE);
#else
    /* 外部时钟: Si5351 CLK0 = 1.024MHz → TIM3 ETR */
    period = 1024000U / (uint32_t)sampling_rate;
    if (period < 2U) period = 2U;
    g_fs = 1024000.0f / (float32_t)period;
    TIM3->SMCR |= TIM_SMCR_ECE;
#endif
    htim3.Init.Period = period - 1U;
    __HAL_TIM_SET_AUTORELOAD(&htim3, htim3.Init.Period);
  }

  /* 启动TIM3 + ADC DMA */
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dft_adc_buf, DFT_FFT_LENGTH);
}

void DFT_Process(void)
{
  if (!dft_snapshot_ready) return;
  dft_snapshot_ready = 0;
  DFT_ProcessFrame(dft_snapshot, DFT_FFT_LENGTH);
}

void DFT_ProcessFrame(const uint16_t *adc_data, uint32_t length)
{
  if (adc_data == NULL || length != DFT_FFT_LENGTH)
  {
    return;
  }

  /* 1. 去直流，转为电压值 */
  uint64_t sum = 0;
  for (uint32_t i = 0; i < length; i++)
  {
    sum += adc_data[i];
  }
  float32_t mean = (float32_t)sum / (float32_t)length;
  float32_t adc_max = (float32_t)((1UL << 16U) - 1UL);

  for (uint32_t i = 0; i < length; i++)
  {
    float32_t centered = ((float32_t)adc_data[i] - mean) / adc_max;
    sig_buf[i] = centered * g_vref;
  }

  /* 2. 粗FFT + 精修 → 基波频率 */
  g_fund_freq = find_fundamental();

  if (g_fund_freq < 1.0f)
  {
    memset(g_mags, 0, sizeof(g_mags));
    g_thd = 0.0f;
    return;
  }

  /* 3. DFT精确计算1-10次谐波幅值（不加窗） */
  dft_harmonics_all(sig_buf, length, g_fund_freq, g_mags);

  /* 4. 计算THD = sqrt(Σmag²[h=2..10]) / mag[1] */
  float32_t harm_pwr = 0.0f;
  for (uint32_t h = 2; h <= DFT_MAX_HARMONIC; h++)
  {
    harm_pwr += g_mags[h] * g_mags[h];
  }
  g_thd = (g_mags[1] > 0.0f) ? sqrtf(harm_pwr) / g_mags[1] : 0.0f;
}

float32_t DFT_GetFundFreq(void)  { return g_fund_freq; }
float32_t DFT_GetTHD(void)       { return g_thd; }
float32_t DFT_GetHarmonicMag(uint32_t h)
{
  if (h < 1 || h > DFT_MAX_HARMONIC) return 0.0f;
  return g_mags[h];
}

void DFT_DiagGetPeak(uint32_t *bin, float32_t *y0, float32_t *y1, float32_t *y2)
{
  if (bin) *bin = g_diag_peak_bin;
  if (y0)  *y0  = g_diag_y0;
  if (y1)  *y1  = g_diag_y1;
  if (y2)  *y2  = g_diag_y2;
}
