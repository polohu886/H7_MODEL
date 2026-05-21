#ifndef __AD9959_H
#define __AD9959_H

#include "main.h"
#include "stdint.h"

// AD9959 寄存器地址定义
#define CSR_ADD   0x00   // 通道选择寄存器
#define FR1_ADD   0x01   // 功能寄存器1
#define FR2_ADD   0x02   // 功能寄存器2
#define CFR_ADD   0x03   // 通道功能寄存器
#define CFTW0_ADD 0x04   // 通道频率调谐字寄存器0
#define CPOW0_ADD 0x05   // 通道相位偏移字寄存器0
#define ACR_ADD   0x06   // 幅度控制寄存器

// 基础 IO 控制宏 (适配 HAL 库)
#define CS_1()          HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)
#define CS_0()          HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define SCLK_1()        HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET)
#define SCLK_0()        HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET)
#define UPDATE_1()      HAL_GPIO_WritePin(UPDATE_GPIO_Port, UPDATE_Pin, GPIO_PIN_SET)
#define UPDATE_0()      HAL_GPIO_WritePin(UPDATE_GPIO_Port, UPDATE_Pin, GPIO_PIN_RESET)
#define SDIO0_1()       HAL_GPIO_WritePin(SDIO0_GPIO_Port, SDIO0_Pin, GPIO_PIN_SET)
#define SDIO0_0()       HAL_GPIO_WritePin(SDIO0_GPIO_Port, SDIO0_Pin, GPIO_PIN_RESET)
#define Reset_1()       HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET)
#define Reset_0()       HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET)
#define AD9959_PWR_0()  HAL_GPIO_WritePin(PDC_GPIO_Port, PDC_Pin, GPIO_PIN_RESET)

// 其他控制引脚
#define PS0_0()         HAL_GPIO_WritePin(PS0_GPIO_Port, PS0_Pin, GPIO_PIN_RESET)
#define PS1_0()         HAL_GPIO_WritePin(PS1_GPIO_Port, PS1_Pin, GPIO_PIN_RESET)
#define PS2_0()         HAL_GPIO_WritePin(PS2_GPIO_Port, PS2_Pin, GPIO_PIN_RESET)
#define PS3_0()         HAL_GPIO_WritePin(PS3_GPIO_Port, PS3_Pin, GPIO_PIN_RESET)

// 函数声明
void Init_AD9959(void);
void Write_Frequence(uint8_t Channel, uint32_t Freq);
void Write_Amplitude(uint8_t Channel, uint16_t Ampli);
void Write_Phase(uint8_t Channel, uint16_t Phase);
void AD9959_IO_Update(void);

extern uint32_t SinFre[4];
extern uint32_t SinAmp[4];
extern uint32_t SinPhr[4];

#endif
