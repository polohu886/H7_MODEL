# DFT 谐波分析驱动使用说明 (STM32H7)

> @author POLO_HU  
> @version V1.1  
> @date 2026-05-23

## 1. 功能概述

基于单频 DFT 的谐波分析驱动。先用粗 FFT 找基波频率，再用单频 DFT 精确计算 1~10 次谐波幅值，最后算 THD。

| 特性 | 说明 |
|------|------|
| 基波检测 | 4096点 FFT + 抛物线插值精修，精度 ±几Hz |
| 谐波计算 | 单频 DFT，一次遍历 1~10 次谐波，三角递推加速 |
| 归一化 | 谐波幅值 ×2/N → 输出实际电压幅值 (V) |
| 自包含 | 驱动内部管理 ADC DMA、TIM3 时钟、数据快照 |

## 2. 数据流

```
Si5351 CLK0 (1.024MHz) → PD2 → TIM3 ETR
  → TIM3 TRGO @ 用户指定的采样率
  → ADC1 CH4 (PC4)
  → DMA → dft_adc_buf[4096]
  → HAL_ADC_ConvCpltCallback: memcpy → dft_snapshot
  → DFT_Process() → DFT_ProcessFrame(snapshot)
      ├─ 去直流 → 转电压
      ├─ 4096点 CFFT → 峰值搜索 → 抛物线插值 → 基波频率
      ├─ 单频 DFT: H1~H10 谐波幅值
      └─ THD = √(ΣH²[h=2..10]) / H1
```

## 3. 接口

```c
// 初始化驱动 (含ADC/TIM3/DMA硬件配置)
void DFT_App_Init(float32_t fs, float32_t vref,
                  float32_t search_min, float32_t search_max);

// 处理内部DMA缓冲区 (主循环调用)
void DFT_Process(void);

// 获取结果
float32_t DFT_GetFundFreq(void);       // 基波频率 (Hz)
float32_t DFT_GetTHD(void);            // THD (0~1, 即 0~100%)
float32_t DFT_GetHarmonicMag(uint32_t h); // 第h次谐波幅值 (V)
```

## 4. main.c 接入示例

```c
#include "DFT.h"

int main(void) {
    // ... CubeMX 初始化 ...
    ZPN_UART_Init();
    si5351_Init();
    SI5351_SetFrequency(0, 1024000);

    DFT_App_Init(512000.0f, 3.3f, 0.0f, 0.0f);
    //           采样率    参考电压 搜下限 搜上限 (0=默认全频段)

    while (1) {
        DFT_Process();
        float f0 = DFT_GetFundFreq();
        float thd = DFT_GetTHD();
        // 处理/打印结果...
    }
}
```

## 5. 配置参数

| 宏 (DFT.h) | 默认值 | 说明 |
|------|--------|------|
| `DFT_MAX_HARMONIC` | 10 | THD 计算包含高次谐波数 |
| `DFT_FFT_LENGTH` | 4096 | FFT 点数 |
| `DFT_SEARCH_MIN_HZ` | 1.0 | 基波搜索下限 |
| `DFT_SEARCH_MAX_HZ` | 256000.0 | 基波搜索上限 |

## 6. 采样率配置

`DFT_App_Init` 传入的采样率自动换算 TIM3 周期：

```
TIM3 Period = 1,024,000 / 采样率
```

| 传入采样率 | TIM3 Period | 实际采样率 |
|------|------|------|
| 512000 | 2 | 512 kHz |
| 256000 | 4 | 256 kHz |
| 64000 | 16 | 64 kHz |

## 7. THD 理论参考值

| 波形 | THD |
|------|------|
| 正弦波 | ≈ 0% |
| 三角波 | ≈ 12% |
| 方波 | ≈ 43% |

## 8. 注意事项

1. **硬件依赖**：需要 Si5351 CLK0 连接到 PD2 (TIM3_ETR)，信号输入 PC4
2. **与 Phase.c 互斥**：DFT 和 Phase 都占用 ADC1 DMA，同一时间只能启用一个
3. **DMA 回调**：驱动内部定义 `HAL_ADC_ConvCpltCallback`，不要在其他文件中重复定义
