#ifndef __IIC_H__
#define __IIC_H__
#include "main.h" 
#include <stdbool.h> 
#include <stdint.h> 
/* iic.h 板级配置节选。 */
#include "stm32f1xx_hal.h"

#define IIC_DELAY_CNT  72U                     /* 100 kHz 用；400 kHz 改 18 */
#define IIC_ACK_RETRY_MAX 200U                 /* 等 ACK 的最大轮询次数 */

/* 错误码：让调用方区分"没应答"和"参数错误"，排错时可定位到具体阶段。 */
#define IIC_OK           0U
#define IIC_ERR_NACK     1U
#define IIC_ERR_PARAM    3U
//读取多个寄存器
uint8_t IIC_ReadRegs(uint8_t addr_w, uint8_t reg, uint8_t *buf, uint16_t len);
//读取单个寄存器
uint8_t IIC_ReadReg(uint8_t addr_w, uint8_t reg, uint8_t *value);
void IIC_RecoverBus(void);
uint8_t IIC_WriteReg(uint8_t addr_w, uint8_t reg, uint8_t value);
//写入多个寄存器
uint8_t IIC_WriteRegs(uint8_t addr_w, uint8_t reg, const uint8_t *buf, uint16_t len);

/* —— 第二路软件 IIC：循迹模块专用，引脚直接用 CubeMX main.h 里的 IIC2_xxx_Pin —— */
#define IIC2_OK          0U
#define IIC2_ERR_NACK    1U
#define IIC2_ERR_PARAM   3U

uint8_t IIC2_ReadRegs(uint8_t addr_w, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t IIC2_ReadReg(uint8_t addr_w, uint8_t reg, uint8_t *value);
uint8_t IIC2_WriteReg(uint8_t addr_w, uint8_t reg, uint8_t value);
uint8_t IIC2_WriteRegs(uint8_t addr_w, uint8_t reg, const uint8_t *buf, uint16_t len);
void IIC2_RecoverBus(void);
#endif 
