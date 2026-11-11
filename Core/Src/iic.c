#include "iic.h"
/* iic.c 引脚操作节选：开漏输出下写 1 是释放，写 0 是拉低。 */
#define IIC_SDA_H()  HAL_GPIO_WritePin(IIC1_SDA_GPIO_Port, IIC1_SDA_Pin, GPIO_PIN_SET)
#define IIC_SDA_L()  HAL_GPIO_WritePin(IIC1_SDA_GPIO_Port, IIC1_SDA_Pin, GPIO_PIN_RESET)
#define IIC_SCL_H()  HAL_GPIO_WritePin(IIC1_SCL_GPIO_Port, IIC1_SCL_Pin, GPIO_PIN_SET)
#define IIC_SCL_L()  HAL_GPIO_WritePin(IIC1_SCL_GPIO_Port, IIC1_SCL_Pin, GPIO_PIN_RESET)
/* 开漏模式下 IDR 反映总线真实电平，不需要切换输入输出方向。 */
#define IIC_SDA_RD() HAL_GPIO_ReadPin(IIC1_SDA_GPIO_Port, IIC1_SDA_Pin)

/* 半位周期忙等。volatile 防止优化等级提高后循环被整体删除。 */
static void IIC_BitDelay(void)
{
    volatile uint32_t i;
    for (i = 0; i < IIC_DELAY_CNT; i++) {
        __NOP(); /* 72 MHz 下单条约 13.9 ns，轮数需实测标定。 */
    }
}

void IIC_Start(void)
{
    IIC_SDA_H();         /* 先释放 SDA，确保当前不是被拉低的状态。 */
    IIC_SCL_H();         /* 只有在 SCL 为高时动 SDA 才构成总线条件。 */
    IIC_BitDelay();      /* 满足起始建立时间，也让上一事务的总线恢复。 */
    IIC_SDA_L();         /* SCL 高期间 SDA 下降：START 条件。 */
    IIC_BitDelay();      /* 起始保持时间，从机用它确认这是 START 而不是数据位。 */
    IIC_SCL_L();         /* 钳住时钟。此后才允许改变 SDA。 */
    IIC_BitDelay();

    //ic_status.start_cnt++;
}

void IIC_Stop(void)
{
    IIC_SCL_L();         /* 先确保时钟为低，否则动 SDA 只会得到数据位。 */
    IIC_SDA_L();         /* 应答后 SDA 已被释放，这里重新拉低准备停止。 */
    IIC_BitDelay();
    IIC_SCL_H();         /* 先放高时钟。 */
    IIC_BitDelay();      /* 停止建立时间。 */
    IIC_SDA_H();         /* SCL 高期间 SDA 上升：STOP 条件，总线回到空闲。 */
    IIC_BitDelay();      /* 总线恢复时间，下一次 START 之前必须留够。 */

   // ic_status.stop_cnt++;
}
static uint8_t IIC_WaitAck(void)
{
    uint8_t retry = 0;

    IIC_SDA_H();      /* 主机必须释放 SDA，否则从机无法把 ACK 拉低。 */
    IIC_BitDelay();
    IIC_SCL_H();

    while (IIC_SDA_RD() != GPIO_PIN_RESET) {   /* 从机拉低 SDA 才算 ACK。 */
        retry++;
        if (retry >= IIC_ACK_RETRY_MAX) {       /* 未应答：记录并放弃，避免死等。 */
           // ic_status.nack_cnt++;
            IIC_SCL_L();
            return IIC_ERR_NACK;
        }
        IIC_BitDelay();
    }

    IIC_SCL_L();     /* 应答时钟周期结束，拉低时钟准备下一位。 */
    IIC_BitDelay();
    return IIC_OK;
}

uint8_t IIC_SendByte(uint8_t byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        /* 高位先行：每轮取当前最高位放到 SDA 上。 */
        if ((byte & 0x80) != 0) {
            IIC_SDA_H();
        } else {
            IIC_SDA_L();
        }
        byte = (uint8_t)(byte << 1); /* 左移让下一位成为最高位。 */
        IIC_BitDelay();              /* 数据建立时间：SDA 必须在 SCL 上升前稳定。 */
        IIC_SCL_H();
        IIC_BitDelay();              /* 数据保持时间：从机在这个窗口采样。 */
        IIC_SCL_L();                 /* 时钟先拉低，下一步才允许改 SDA。 */
        IIC_BitDelay();
    }

    return IIC_WaitAck();            /* 第 9 个时钟周期检查 ACK。 */
}



uint8_t IIC_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t byte = 0;

    IIC_SDA_H();                 /* 释放总线转为接收，数据由从机驱动。 */
    for (i = 0; i < 8; i++) {
        byte = (uint8_t)(byte << 1);        /* 左移为下一位腾出最低位。 */
        IIC_SCL_H();
        IIC_BitDelay();
        /* SCL 为高期间采样，正是高位先行的位序来源。 */
        if (IIC_SDA_RD() != GPIO_PIN_RESET) {
            byte |= 0x01U;
        }
        IIC_SCL_L();
        IIC_BitDelay();
    }

    /* 第 9 个时钟由主机发 ACK 或 NACK。 */
    if (ack != 0) {
        IIC_SDA_L();   /* 拉低表示还要继续读，从机继续驱动下一位。 */
    } else {
        IIC_SDA_H();   /* 释放表示这是最后一个字节，从机停止驱动。 */
    }
    IIC_BitDelay();
    IIC_SCL_H();
    IIC_BitDelay();
    IIC_SCL_L();
    IIC_BitDelay();
    IIC_SDA_H();       /* 释放 SDA，为下一次启动条件或停止条件做准备。 */

    return byte;
}
uint8_t IIC_WriteReg(uint8_t addr_w, uint8_t reg, uint8_t value)
{
    uint8_t ret;

    IIC_Start();
    ret = IIC_SendByte(addr_w);          /* 地址字节，已含写方向位。 */
    if (ret != IIC_OK) { IIC_Stop(); return ret; }
    ret = IIC_SendByte(reg);             /* 从机寄存器地址。 */
    if (ret != IIC_OK) { IIC_Stop(); return ret; }
    ret = IIC_SendByte(value);           /* 要写入的数据。 */
    IIC_Stop();                          /* 无论成败都要释放总线。 */

    return ret;
}

uint8_t IIC_WriteRegs(uint8_t addr_w, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint8_t ret;

    /* 边界检查：空指针或零长度直接拒绝，避免误操作总线。 */
    if ((buf == NULL) || (len == 0)) {
        return IIC_ERR_PARAM;
    }

    IIC_Start();
    ret = IIC_SendByte(addr_w);          /* 地址字节，已含写方向位。 */
    if (ret != IIC_OK) { IIC_Stop(); return ret; }
    ret = IIC_SendByte(reg);             /* 从机寄存器起始地址，之后地址自增。 */
    if (ret != IIC_OK) { IIC_Stop(); return ret; }

    for (i = 0; i < len; i++) {
        ret = IIC_SendByte(buf[i]);      /* 逐字节写，任一字节没应答立即中止。 */
        if (ret != IIC_OK) { IIC_Stop(); return ret; }
    }
    IIC_Stop();                          /* 无论成败都要释放总线。 */

    return IIC_OK;
}

uint8_t IIC_ReadRegs(uint8_t addr_w, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint8_t ret;

    /* 边界检查：空指针或零长度直接拒绝，避免写坏内存。 */
    if ((buf == NULL) || (len == 0)) {
        return IIC_ERR_PARAM;
    }

    IIC_Start();
    ret = IIC_SendByte(addr_w);          /* 第一段：写方向，送寄存器地址。 */
    if (ret != IIC_OK) { IIC_Stop(); return ret; }
    ret = IIC_SendByte(reg);
    if (ret != IIC_OK) { IIC_Stop(); return ret; }

    IIC_Start();                         /* 重复起始，切换为读方向，中途不发 STOP。 */
    ret = IIC_SendByte((uint8_t)(addr_w | 0x01));  /* 只翻转方向位得到读地址。 */
    if (ret != IIC_OK) { IIC_Stop(); return ret; }

    for (i = 0; i < len; i++) {
        /* 前 len-1 个字节 ACK，最后一个字节 NACK，结束读取。 */
        buf[i] = IIC_ReadByte((i == (len - 1U)) ? 0U : 1U);
    }
    IIC_Stop();

    return IIC_OK;
}

uint8_t IIC_ReadReg(uint8_t addr_w, uint8_t reg, uint8_t *value)
{
    /* 单字节读就是多字节读的特例，避免维护两套逻辑。 */
    return IIC_ReadRegs(addr_w, reg, value, 1U);
}

/* 典型现象：WHO_AM_I 连续 NACK，且逻辑分析仪上 STOP 后 SDA 仍为低。 */
void IIC_RecoverBus(void)
{
    uint8_t i;

    IIC_SCL_L();
    IIC_BitDelay();
    IIC_SDA_H();      /* 先释放 SDA，否则 STOP 条件不成立。 */

    for (i = 0; i < 9; i++) {
        IIC_SCL_H();  /* 一个脉冲让从机吐出正在发的位。 */
        IIC_BitDelay();
        if (IIC_SDA_RD() != GPIO_PIN_RESET) { break; }   /* 从机已释放 SDA。 */
        IIC_SCL_L();
        IIC_BitDelay();
    }

    IIC_SCL_L();
    IIC_BitDelay();
    IIC_Stop();       /* 用 STOP 让所有从机回到空闲状态。 */

    //ic_status.recover_cnt++;
}
