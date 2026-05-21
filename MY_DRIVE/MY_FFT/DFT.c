/**
 ******************************************************************************
 * @file    DFT.c
 * @brief   离散傅里叶变换谐波分析
 * @note    流程: 粗FFT找基波 → 频点精修 → DFT精确计算1-10次谐波幅值 → THD
 ******************************************************************************
 */

#include "DFT.h"
#include "arm_const_structs.h"
#include <math.h>
#include <string.h>

/* ==================== 静态变量 ==================== */
static float32_t g_fs = 64000.0f;
static float32_t g_vref = 3.3f;
static float32_t g_fund_freq = 0.0f;
static float32_t g_thd = 0.0f;
static float32_t g_mags[DFT_MAX_HARMONIC + 1] = {0.0f}; /* [0]空，[1..10]各次谐波DFT幅值 */

/* FFT工作缓冲 */
static float32_t fft_in[DFT_FFT_LENGTH * 2];
static float32_t fft_mag[DFT_FFT_LENGTH];

/* 信号缓冲（去直流后的电压值） */
static float32_t sig_buf[DFT_FFT_LENGTH];

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

  for (uint32_t n = 0; n < N; n++)
  {
    float32_t theta = omega * (float32_t)n;
    float32_t c = cosf(theta);
    float32_t s = sinf(theta);

    /* 三角递推: cos(hθ) = cos((h-1)θ)cosθ - sin((h-1)θ)sinθ */
    /*           sin(hθ) = sin((h-1)θ)cosθ + cos((h-1)θ)sinθ */
    float32_t ch = 1.0f, sh = 0.0f;  /* h=0 */
    float32_t sample = x[n];

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

  for (uint32_t h = 1; h <= DFT_MAX_HARMONIC; h++)
  {
    mags[h] = sqrtf(re[h] * re[h] + im[h] * im[h]);
  }
}

/**
 * @brief  粗FFT + 频点精修：找到准确的基波频率
 * @return 基波频率(Hz)
 */
static float32_t find_fundamental(void)
{
  /* 1. 准备FFT输入: 实数→复数交错 */
  for (uint32_t i = 0; i < DFT_FFT_LENGTH; i++)
  {
    fft_in[2 * i] = sig_buf[i];
    fft_in[2 * i + 1] = 0.0f;
  }

  /* 2. 执行FFT */
  arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_in, 0, 1);

  /* 3. 计算幅值 */
  arm_cmplx_mag_f32(fft_in, fft_mag, DFT_FFT_LENGTH);

  /* 4. 在搜索范围内找最大峰 */
  uint32_t half = DFT_FFT_LENGTH / 2U;
  uint32_t min_idx = (uint32_t)((DFT_SEARCH_MIN_HZ * (float32_t)DFT_FFT_LENGTH) / g_fs + 0.5f);
  uint32_t max_idx = (uint32_t)((DFT_SEARCH_MAX_HZ * (float32_t)DFT_FFT_LENGTH) / g_fs + 0.5f);
  if (min_idx < 1U) min_idx = 1U;
  if (max_idx > half - 1U) max_idx = half - 1U;

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

  /* 5. 用相邻bin做频率精修（矩形窗） */
  float32_t delta = 0.0f;
  float32_t m_prev = (peak_idx > 0) ? fft_mag[peak_idx - 1] : 0.0f;
  float32_t m_next = fft_mag[peak_idx + 1];

  if (m_next > m_prev)
  {
    delta = m_next / (fft_mag[peak_idx] + m_next);
  }
  else
  {
    delta = -m_prev / (fft_mag[peak_idx] + m_prev);
  }

  return ((float32_t)peak_idx + delta) * g_fs / (float32_t)DFT_FFT_LENGTH;
}

/* ==================== 公开接口 ==================== */

void DFT_Init(float32_t sampling_rate, float32_t ref_voltage)
{
  g_fs = sampling_rate;
  g_vref = ref_voltage;
  memset(g_mags, 0, sizeof(g_mags));
  g_fund_freq = 0.0f;
  g_thd = 0.0f;
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
