# Phase 频谱分析库使用说明文档（STM32H7）

> @author POLO_HU  
> @version V2.1  
> @date 2026-05-23

本文档说明如何在当前 STM32H7 工程中使用 `Phase.c` / `Phase.h` 进行 FFT 频谱分析及 MATLAB 实时绘图。

---

## 1. 功能概述

| 功能 | 说明 |
|------|------|
| 单通道 FFT 频谱分析 | TIM3 ETR 外部时钟 (Si5351) 触发 ADC1，DMA 采集 4096 点，Hanning 窗 |
| 幅值归一化 | 输出实际电压值 (V)，补偿 CFFT 未缩放 + 窗函数相干增益 |
| 串口频谱输出 | `FFT_BEGIN` / `FFT_END` 帧格式，`plot_fft_from_serial.m` 实时绘图 |

---

## 2. 数据流

```
Si5351 CLK0 (1.024MHz)
    → PD2 (TIM3_ETR)
    → TIM3 TRGO @ 采样率 (由 PHASE_TARGET_FS 宏决定)
    → ADC1 CH4 (PC4) 触发采样
    → DMA1_Stream3 搬移 4096 点到 ADC_Buffer
    → HAL_ADC_ConvCpltCallback: 停止DMA → memcpy快照 → 重启DMA → 置 frame_ready
    → main loop: FFT_SendSpectrumFrame()
        ├─ Phase_ComputeThdAndFft()
        │   ├─ 去直流偏置
        │   ├─ 转电压 (ADC / 65535  3.3V)
        │   ├─ Hanning 窗 (WindowFunction.c: hannWin)
        │   ├─ arm_cfft_f32 (4096 点)
        │   ├─ arm_cmplx_mag_f32
        │   └─ 幅值归一化 ( 2/(N 0.5))
        └─ 串口输出: FFT_BEGIN → 2048 行 idx,mag → FFT_END
```

---

## 3. 关键配置（Phase.h）

| 宏 | 值 | 说明 |
|------|------|------|
| `PHASE_CLOCK_SOURCE` | `PHASE_CLK_INTERNAL` | **重要**：设为 `PHASE_CLK_EXTERNAL` 才启用 PD2 外部时钟 (Si5351 1.024MHz)。内部模式用 APB1 时钟 |
| `PHASE_TARGET_FS` | 1024000.0f | 目标采样率 (Hz)，自动换算 TIM3 周期 |
| `FFT_LENGTH` | 4096 | FFT 点数，2 的幂 |
| `ADC_EFFECTIVE_BITS` | 16 | ADC 分辨率 |
| `SAMPLING_RATE_DEFAULT` | 64000 | 兜底值，运行时自动覆盖 |

---

## 4. 外部接口

### 4.1 void FFT_App_Init(void)
初始化 FFT 实例，配置 ADC 通道为 PC4，覆盖 TIM3 周期，启动采集。
```c
// main.c 中调用 (在 Si5351 配置之后)
si5351_Init();
SI5351_SetFrequency(0, 1024000);
FFT_App_Init();
```

### 4.2 void FFT_SendSpectrumFrame(void)
主循环调用。检查帧就绪 → 执行 FFT 计算 → 串口输出频谱数据。
```c
while (1) {
    FFT_SendSpectrumFrame();
}
```

### 4.3 void FFT_App_Process(void)
主循环调用（与 `FFT_SendSpectrumFrame` 二选一）。处理 ADC 采集、FFT 变换，但**不输出串口频谱帧**，适用于不需要串口输出的场景。
```c
while (1) {
    FFT_App_Process();
}
```

---

## 5. main.c 接入方式

```c
/* USER CODE BEGIN Includes */
#include "Phase.h"
#include "si5351.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
ZPN_UART_Init();
si5351_Init();
SI5351_SetFrequency(0, 1024000);
FFT_App_Init();
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
while (1) {
    FFT_SendSpectrumFrame();
}
/* USER CODE END 3 */
```

---

## 6. CubeMX 关键配置

| 外设 | 配置项 | 值 |
|------|--------|------|
| ADC1 | External Trigger | TIM3 TRGO, Rising Edge |
| ADC1 | DMA | Circular, HalfWord |
| ADC1 | Resolution | 16-bit |
| TIM3 | Clock Source | ETR Mode 2 (PD2) |
| TIM3 | Prescaler | 0 ( 1) |
| TIM3 | TRGO | Update Event |
| DMA1_Stream3 | Request | ADC1 |
| I2C1 | | Si5351 通信 |

---

## 7. 修改采样率

修改 `Phase.h` 中的 `PHASE_TARGET_FS` 宏，`FFT_App_Init()` 会自动计算 TIM3 周期。

**外部时钟模式下（`PHASE_CLK_EXTERNAL`）：**

```
TIM3 Period = 1,024,000 / PHASE_TARGET_FS
```

| PHASE_TARGET_FS | TIM3 Period | 采样率 | 奈奎斯特 |
|------|------|------|------|
| 1024000 | 1 (实际 2) | 512 kHz | 256 kHz |
| 512000 | 2 | 512 kHz | 256 kHz |
| 256000 | 4 | 256 kHz | 128 kHz |
| 128000 | 8 | 128 kHz | 64 kHz |
| 64000 | 16 | 64 kHz | 32 kHz |

内部时钟模式下由 APB1 频率计算，公式不同。

---

## 8. 串口输出格式

```
FFT_BEGIN,Fs=512000.00,N=4096
0,0.000123
1,0.000456
...
2047,0.000789
FFT_END
```

- 每行 `bin索引,幅值(V)`，共 2048 行
- MATLAB 脚本 `plot_fft_from_serial.m` 自动解析 Fs/N 参数
- 空格键暂停/恢复，红点标注峰值频率

---

## 9. 注意事项

1. **不可在中断回调中加串口打印**：UART TX DMA 与 ADC DMA 同级优先级会死锁
2. **奈奎斯特**：输入信号频率 < 采样率/2，否则混叠
3. **幅值单位**：已归一化到实际电压 (V)，1V 正弦输入 → 幅值 ~1.0
4. **与 DFT.c 互斥**：Phase 和 DFT 都占用 ADC1 DMA，同一时间只能启用一个

## CMake 工程配置

```cmake
# 添加驱动源文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    MY_DRIVE/MY_FFT/Phase.c
    MY_DRIVE/MY_FFT/WindowFunction.c
)

# 添加驱动头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    MY_DRIVE/MY_FFT
)
```
