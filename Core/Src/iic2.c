#include "iic.h"
/* iic2.c：第二路软件 IIC，循迹模块专用（PD3/PD4）。
   与 iic.c（第一路 PB14/PB15，MPU6050）逻辑同构，头文件共用 iic.h。
   引脚直接用 CubeMX 生成的 main.h 里的名字（IIC2_SDA_Pin 等），
   PD3/PD4 的开漏配置由 gpio.c（MX_GPIO_Init）完成，无需再初始化。
   时序参数共用 IIC_DELAY_CNT / IIC_ACK_RETRY_MAX。 */

/*———————————————————— 第二路软件 IIC：循迹模块（PD3/PD4） ————————————————————*/
/* 与第一路完全相同的位 bang 实现，只是引脚换成 PD3/PD4、函数加 IIC2_ 前缀。
   两路独立，互不干扰；100 kHz 时序参数共用 IIC_DELAY_CNT / IIC_ACK_RETRY_MAX。 */
#define IIC2_SDA_H()  HAL_GPIO_WritePin(IIC2_SDA_GPIO_Port, IIC2_SDA_Pin, GPIO_PIN_SET)
#define IIC2_SDA_L()  HAL_GPIO_WritePin(IIC2_SDA_GPIO_Port, IIC2_SDA_Pin, GPIO_PIN_RESET)
#define IIC2_SCL_H()  HAL_GPIO_WritePin(IIC2_SCL_GPIO_Port, IIC2_SCL_Pin, GPIO_PIN_SET)
#define IIC2_SCL_L()  HAL_GPIO_WritePin(IIC2_SCL_GPIO_Port, IIC2_SCL_Pin, GPIO_PIN_RESET)
#define IIC2_SDA_RD() HAL_GPIO_ReadPin(IIC2_SDA_GPIO_Port, IIC2_SDA_Pin)

static void IIC2_BitDelay(void)
{
    volatile uint32_t i;
    for (i = 0; i < IIC_DELAY_CNT; i++) {
        __NOP();
    }
}

void IIC2_Start(void)
{
    IIC2_SDA_H();
    IIC2_SCL_H();
    IIC2_BitDelay();
    IIC2_SDA_L();
    IIC2_BitDelay();
    IIC2_SCL_L();
    IIC2_BitDelay();
}

void IIC2_Stop(void)
{
    IIC2_SCL_L();
    IIC2_SDA_L();
    IIC2_BitDelay();
    IIC2_SCL_H();
    IIC2_BitDelay();
    IIC2_SDA_H();
    IIC2_BitDelay();
}

static uint8_t IIC2_WaitAck(void)
{
    uint8_t retry = 0;

    IIC2_SDA_H();
    IIC2_BitDelay();
    IIC2_SCL_H();

    while (IIC2_SDA_RD() != GPIO_PIN_RESET) {
        retry++;
        if (retry >= IIC_ACK_RETRY_MAX) {
            IIC2_SCL_L();
            return IIC2_ERR_NACK;
        }
        IIC2_BitDelay();
    }

    IIC2_SCL_L();
    IIC2_BitDelay();
    return IIC2_OK;
}

uint8_t IIC2_SendByte(uint8_t byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if ((byte & 0x80) != 0) {
            IIC2_SDA_H();
        } else {
            IIC2_SDA_L();
        }
        byte = (uint8_t)(byte << 1);
        IIC2_BitDelay();
        IIC2_SCL_H();
        IIC2_BitDelay();
        IIC2_SCL_L();
        IIC2_BitDelay();
    }

    return IIC2_WaitAck();
}

uint8_t IIC2_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t byte = 0;

    IIC2_SDA_H();
    for (i = 0; i < 8; i++) {
        byte = (uint8_t)(byte << 1);
        IIC2_SCL_H();
        IIC2_BitDelay();
        if (IIC2_SDA_RD() != GPIO_PIN_RESET) {
            byte |= 0x01U;
        }
        IIC2_SCL_L();
        IIC2_BitDelay();
    }

    if (ack != 0) {
        IIC2_SDA_L();
    } else {
        IIC2_SDA_H();
    }
    IIC2_BitDelay();
    IIC2_SCL_H();
    IIC2_BitDelay();
    IIC2_SCL_L();
    IIC2_BitDelay();
    IIC2_SDA_H();

    return byte;
}

uint8_t IIC2_WriteReg(uint8_t addr_w, uint8_t reg, uint8_t value)
{
    uint8_t ret;

    IIC2_Start();
    ret = IIC2_SendByte(addr_w);
    if (ret != IIC2_OK) { IIC2_Stop(); return ret; }
    ret = IIC2_SendByte(reg);
    if (ret != IIC2_OK) { IIC2_Stop(); return ret; }
    ret = IIC2_SendByte(value);
    IIC2_Stop();

    return ret;
}

uint8_t IIC2_WriteRegs(uint8_t addr_w, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint8_t ret;

    if ((buf == NULL) || (len == 0)) {
        return IIC2_ERR_PARAM;
    }

    IIC2_Start();
    ret = IIC2_SendByte(addr_w);
    if (ret != IIC2_OK) { IIC2_Stop(); return ret; }
    ret = IIC2_SendByte(reg);
    if (ret != IIC2_OK) { IIC2_Stop(); return ret; }

    for (i = 0; i < len; i++) {
        ret = IIC2_SendByte(buf[i]);
        if (ret != IIC2_OK) { IIC2_Stop(); return ret; }
    }
    IIC2_Stop();

    return IIC2_OK;
}

uint8_t IIC2_ReadRegs(uint8_t addr_w, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint8_t ret;

    if ((buf == NULL) || (len == 0)) {
        return IIC2_ERR_PARAM;
    }

    IIC2_Start();
    ret = IIC2_SendByte(addr_w);
    if (ret != IIC2_OK) { IIC2_Stop(); return ret; }
    ret = IIC2_SendByte(reg);
    if (ret != IIC2_OK) { IIC2_Stop(); return ret; }

    IIC2_Start();
    ret = IIC2_SendByte((uint8_t)(addr_w | 0x01));
    if (ret != IIC2_OK) { IIC2_Stop(); return ret; }

    for (i = 0; i < len; i++) {
        buf[i] = IIC2_ReadByte((i == (len - 1U)) ? 0U : 1U);
    }
    IIC2_Stop();

    return IIC2_OK;
}

uint8_t IIC2_ReadReg(uint8_t addr_w, uint8_t reg, uint8_t *value)
{
    return IIC2_ReadRegs(addr_w, reg, value, 1U);
}

void IIC2_RecoverBus(void)
{
    uint8_t i;

    IIC2_SCL_L();
    IIC2_BitDelay();
    IIC2_SDA_H();

    for (i = 0; i < 9; i++) {
        IIC2_SCL_H();
        IIC2_BitDelay();
        if (IIC2_SDA_RD() != GPIO_PIN_RESET) { break; }
        IIC2_SCL_L();
        IIC2_BitDelay();
    }

    IIC2_SCL_L();
    IIC2_BitDelay();
    IIC2_Stop();
}
