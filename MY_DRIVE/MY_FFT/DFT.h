/**
 ******************************************************************************
 * @file    DFT.h
 * @brief   离散傅里叶变换谐波分析模块
 * @note    使用DFT直接计算各次谐波频率处的幅值，无bin偏移误差，无需加窗
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
#define DFT_SEARCH_MIN_HZ   800.0f
#define DFT_SEARCH_MAX_HZ   1200.0f

/* 初始化：设置采样率和参考电压 */
void DFT_Init(float32_t sampling_rate, float32_t ref_voltage);

/* 处理一帧ADC数据，计算各次谐波幅值与THD */
void DFT_ProcessFrame(const uint16_t *adc_data, uint32_t length);

/* 获取计算结果 */
float32_t DFT_GetFundFreq(void);
float32_t DFT_GetTHD(void);
float32_t DFT_GetHarmonicMag(uint32_t h);

#ifdef __cplusplus
}
#endif

#endif /* __DFT_H */
