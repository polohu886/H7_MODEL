# MCP41xx 数字电位器 / 程控放大器

## 概述

MCP41xx 是 256 抽头数字电位器，通过 GPIO 模拟 SPI 控制。驱动同时封装了程控放大器功能（需配合外部运放使用）。

## 引脚连接

| MCP41xx 信号 | STM32H743 引脚 | 功能 |
|-------------|---------------|------|
| CS1 | PC8 | 电位器1 片选 |
| CS2 | PC9 | 电位器2 片选 |
| DAT | PC12 | SPI 数据 (MOSI) |
| CLK | PC13 | SPI 时钟 |

## 导入

将 `MCP41xx.c` / `MCP41xx.h` 加入工程，main.c 添加：

```c
#include "MCP41xx.h"
```

## 初始化

```c
MCP410XXInit();
```

内部自动使能 GPIOC 时钟并配置 PC8/PC9/PC12/PC13 为推挽输出。**不需要在 CubeMX 中配置这些引脚**（确保未被其他外设占用即可）。

## 核心函数

### MCP41xx_1writedata(uint8_t dat)

设置电位器1 的抽头位置。参数 `dat` 范围 0~255，对应 0Ω ~ 满量程。

### MCP41xx_2writedata(uint8_t dat)

设置电位器2 的抽头位置。用法同上。

### SetAmplifierGain(float gain)

程控放大器封装函数。输入放大倍数，自动反算电位器抽头值。

- `gain = 0.0` → 静音（输出 0）
- `gain = 1.0` → 原幅值输出
- `gain = 2.0` → 2 倍放大

## 增益计算公式

```
总增益 = (1 + Rf / R1) × dat / 255
```

默认 Rf = 1kΩ, R1 = 1kΩ, 最大增益 = 2 倍。如需修改电阻值，编辑 `MCP41xx.c` 中的：

```c
#define AMPLIFIER_Rf  1000.0f
#define AMPLIFIER_R1  1000.0f
```

## 示例

```c
MCP410XXInit();

// 直接设置电位器值
MCP41xx_1writedata(128);  // 电位器1 设为中间值

// 使用程控放大器接口
SetAmplifierGain(1.5f);   // 1.5倍放大
```
