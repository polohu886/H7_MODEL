#include "ad9959.h"

uint8_t CSR_DATA0[1] = {0x10};
uint8_t CSR_DATA1[1] = {0x20};
uint8_t CSR_DATA2[1] = {0x40};
uint8_t CSR_DATA3[1] = {0x80};

uint32_t SinFre[4] = {1000, 1000, 1000, 1000};
uint32_t SinAmp[4] = {1023, 1023, 1023, 1023};
uint32_t SinPhr[4] = {0, 0, 0, 0};

// 适配 480MHz H7 的软件延时
void delay_9959(uint32_t length)
{
    length = length * 300; // 缩放延时参数，确保 H7 480MHz 时钟下延时正确
    while (length--) { __NOP(); }
}

void IntReset(void)
{
    Reset_1(); delay_9959(200);
    Reset_0(); delay_9959(200);
}

void AD9959_IO_Update(void)
{
    UPDATE_1(); delay_9959(50);
    UPDATE_0(); delay_9959(50);
}

void Intserve(void)
{
    AD9959_PWR_0(); CS_1(); SCLK_0(); UPDATE_0();
    PS0_0(); PS1_0(); PS2_0(); PS3_0(); SDIO0_0();
}

void WriteData_AD9959(uint8_t RegisterAddress, uint8_t NumberofRegisters, uint8_t *RegisterData, uint8_t update_flag)
{
    uint8_t ControlValue = RegisterAddress;
    SCLK_0(); delay_9959(2); CS_0();

    for (int i = 0; i < 8; i++) {
        SCLK_0();
        if (ControlValue & 0x80) SDIO0_1(); else SDIO0_0();
        delay_9959(5); SCLK_1(); delay_9959(5);
        ControlValue <<= 1;
    }
    SCLK_0();

    for (int i = 0; i < NumberofRegisters; i++) {
        uint8_t ValueToWrite = RegisterData[i];
        for (int j = 0; j < 8; j++) {
            SCLK_0();
            if (ValueToWrite & 0x80) SDIO0_1(); else SDIO0_0();
            delay_9959(5); SCLK_1(); delay_9959(5);
            ValueToWrite <<= 1;
        }
        SCLK_0();
    }
    CS_1();
    if (update_flag) AD9959_IO_Update();
}

void Init_AD9959(void)
{
    uint8_t FR1_DATA[3] = {0xD3, 0x00, 0x00}; // 20倍频，启用VCO范围

    Intserve();

    // 1. 硬件复位
    Reset_1(); delay_9959(500);
    Reset_0(); delay_9959(500);

    // 2. 写入 FR1 (启用 PLL)
    WriteData_AD9959(FR1_ADD, 3, FR1_DATA, 1);

    // 3. 发送 IO UPDATE 脉冲使 PLL 生效
    AD9959_IO_Update();
    delay_9959(2000); // 关键：给 PLL 锁相足够的建立时间

    // 4. 初始化四个通道默认频率幅度
    for(int i=0; i<4; i++) {
        Write_Frequence(i, 1000);
        Write_Amplitude(i, 1023);
    }
    AD9959_IO_Update();
}

void Write_Frequence(uint8_t Channel, uint32_t Freq)
{
    uint8_t CFTW0_DATA[4];
    // 频率字计算：2^32 / 500,000,000 ≈ 8.589934592
    // 如果 PLL 锁相成功后系统时钟 500MHz，系数为 8.5899...
    uint32_t Temp = (uint32_t)(Freq * 8.589934592);
    CFTW0_DATA[3] = (uint8_t)Temp;
    CFTW0_DATA[2] = (uint8_t)(Temp >> 8);
    CFTW0_DATA[1] = (uint8_t)(Temp >> 16);
    CFTW0_DATA[0] = (uint8_t)(Temp >> 24);

    uint8_t CSR_VAL = (Channel == 0) ? 0x10 : (Channel == 1) ? 0x20 : (Channel == 2) ? 0x40 : 0x80;
    WriteData_AD9959(CSR_ADD, 1, &CSR_VAL, 0);
    WriteData_AD9959(CFTW0_ADD, 4, CFTW0_DATA, 1);
}

void Write_Amplitude(uint8_t Channel, uint16_t Ampli)
{
    uint8_t ACR_DATA[3] = {0x00, 0x00, 0x00};
    if (Ampli > 1023) Ampli = 1023;
    ACR_DATA[0] = 0x00;
    ACR_DATA[1] = (uint8_t)((Ampli >> 8) & 0x03) | 0x10;
    ACR_DATA[2] = (uint8_t)Ampli;

    uint8_t CSR_VAL = (Channel == 0) ? 0x10 : (Channel == 1) ? 0x20 : (Channel == 2) ? 0x40 : 0x80;
    WriteData_AD9959(CSR_ADD, 1, &CSR_VAL, 0);
    WriteData_AD9959(ACR_ADD, 3, ACR_DATA, 1);
}

void Write_Phase(uint8_t Channel, uint16_t Phase)
{
    uint8_t CPOW_DATA[2];
    CPOW_DATA[1] = (uint8_t)Phase; CPOW_DATA[0] = (uint8_t)(Phase >> 8);

    uint8_t CSR_VAL = (Channel == 0) ? 0x10 : (Channel == 1) ? 0x20 : (Channel == 2) ? 0x40 : 0x80;
    WriteData_AD9959(CSR_ADD, 1, &CSR_VAL, 0);
    WriteData_AD9959(CPOW0_ADD, 2, CPOW_DATA, 1);
}
