# 相位计算库使用说明文档（STM32H7）

本文档说明如何在当前 STM32H7 工程中使用 Phase.c 和 Phase.h 进行基于 FFT 的频谱与相位相关处理。

## 1. 当前实现能力

当前代码已实现以下能力：
1. 单通道 FFT 处理链路：TIM3 触发 ADC1，DMA 采样，主循环内非阻塞处理。
2. 频谱幅值计算：对实数输入执行复数 FFT 并计算模值。
3. 峰值频点输出：可打印主峰频率和幅值。
4. 双 ADC 相位差兼容接口：保留 Get_PhaseDifference 等接口，便于后续扩展。

说明：文档仅描述当前代码中真实存在的接口与行为，不包含未实现 API。

## 2. 关键配置（Phase.h）

### 2.1 基础宏
1. FFT_LENGTH：FFT 点数，当前为 1024，必须是 2 的幂。
2. SAMPLING_RATE_DEFAULT：默认采样率，仅作为兜底值；运行时优先使用 TIM3 实际参数计算采样率。
3. FFT_OUTPUT_FULL_SPECTRUM：可选，开启后输出完整频谱数据。
4. FFT_OUTPUT_BINARY：可选，与全频谱模式配合使用，按二进制帧输出。

### 2.2 电压换算说明
当前工程 ADC 配置为 16bit，电压换算按 65535 满量程处理。

## 3. 主要接口

1. void FFT_App_Init(void)
作用：初始化 FFT 实例，启动 TIM3 与 ADC1 DMA 首次采样。

2. void FFT_App_Process(void)
作用：主循环处理函数。
行为：检测采样完成标志，执行 FFT，输出结果，并重新启动下一次 DMA 采样。

3. float32_t Get_PhaseDifference(void)
作用：兼容接口，使用双 ADC 数据计算相位差。
说明：是否有效取决于你是否已正确启动并喂入 ADC1/ADC2 数据链路。

## 4. main.c 推荐接入方式（无阻塞）

```c
/* USER CODE BEGIN Includes */
#include "Phase.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
FFT_App_Init();
/* USER CODE END 2 */

while (1)
{
    FFT_App_Process();
}
```

说明：不建议在该主循环内加入 HAL_Delay，以免影响后续控制任务与实时性。

## 5. CubeMX 关键配置检查

请确保以下配置与当前实现一致：
1. ADC1
- External Trigger: TIM3 TRGO
- Trigger Edge: Rising
- DMA: Circular
- Resolution: 16-bit

2. TIM3
- Clock Source: Internal
- TRGO: Update Event

3. DMA
- ADC1 DMA 数据宽度为 HalfWord
- Memory Increment 使能

## 6. 注意事项

1. 奈奎斯特条件
输入信号频率应满足 f_in < f_s / 2，否则频谱会发生混叠。

2. 采样率显示
峰值频率计算使用运行时 TIM3 参数换算得到的采样率，而非固定 84MHz 假设。

3. 双 ADC 相位差
若你后续要启用稳定的双通道相位测量，建议补齐双 ADC 同步触发和完整 DMA 状态管理。

