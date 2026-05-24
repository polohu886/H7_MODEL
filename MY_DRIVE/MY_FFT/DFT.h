/**
 ******************************************************************************
 * @file    DFT.h
 * @brief   离散傅里叶变换谐波分析模块
 * @author  POLO_HU
 * @version V1.1
 * @date    2026-05-23
 * @note    使用DFT直接计算各次谐波频率处的幅值，无bin偏移误差，无需加窗
 *          流程: 粗FFT找基波 → 频点精修 → DFT精确计算1-10次谐波幅值 → THD
 ******************************************************************************
 */

#ifndef __DFT_H
#define __DFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "arm_math.h"

#define DFT_MAX_HARMONIC    10U
#define DFT_FFT_LENGTH      4096U

/* TIM3时钟源选择: 0=外部Si5351(1.024MHz), 1=内部APB1(240MHz) */
#define DFT_CLK_EXTERNAL    0
#define DFT_CLK_INTERNAL    1
#define DFT_CLOCK_SOURCE    DFT_CLK_INTERNAL  /* <-- 改这里切换时钟源 */

/* 基波搜索默认范围 (Hz), 可在 DFT_Init 中覆盖 */
#define DFT_SEARCH_MIN_HZ   1.0f
#define DFT_SEARCH_MAX_HZ   256000.0f

/**
 * @brief  初始化DFT驱动 (含ADC DMA硬件配置)
 * @param  sampling_rate: 采样率 (Hz), 用于配置 TIM3 周期 = 1.024MHz / fs
 * @param  ref_voltage:   ADC参考电压 (V)
 * @param  search_min_hz: 基波搜索下限 (Hz), 0=使用默认
 * @param  search_max_hz: 基波搜索上限 (Hz), 0=使用默认
 */
void DFT_App_Init(float32_t sampling_rate, float32_t ref_voltage,
                  float32_t search_min_hz, float32_t search_max_hz);

/* 处理内部DMA缓冲区数据，计算各次谐波幅值与THD */
void DFT_Process(void);

/* 处理外部数据 (兼容接口) */
void DFT_ProcessFrame(const uint16_t *adc_data, uint32_t length);

/* 获取计算结果 */
float32_t DFT_GetFundFreq(void);
float32_t DFT_GetTHD(void);
float32_t DFT_GetHarmonicMag(uint32_t h);
void DFT_DiagGetPeak(uint32_t *bin, float32_t *y0, float32_t *y1, float32_t *y2);

#ifdef __cplusplus
}
#endif

#endif /* __DFT_H */
