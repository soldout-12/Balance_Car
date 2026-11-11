#ifndef __MPU6050_H__
#define __MPU6050_H__
#include "main.h" 

/* mpu6050.h 节选：寄存器宏、数据结构和全部原型集中在这里，第 9、10 章的驱动代码都基于它。 */
#define MPU6050_ADDR_W             0xD0    /* AD0 接地：地址 0x68 加写方向 */
#define MPU6050_WHO_AM_I_VALUE     0x68
#define MPU6050_RAW_LEN            14

/* 寄存器地址，与 9.3 节的寄存器表一一对应。 */
#define MPU6050_REG_SMPLRT_DIV     0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CONFIG    0x1B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_FIFO_EN        0x23
#define MPU6050_REG_INT_PIN_CFG    0x37
#define MPU6050_REG_INT_ENABLE     0x38
#define MPU6050_REG_INT_STATUS     0x3A
#define MPU6050_REG_ACCEL_XOUT_H   0x3B
#define MPU6050_REG_USER_CTRL      0x6A
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_PWR_MGMT_2     0x6C
#define MPU6050_REG_FIFO_COUNTH    0x72
#define MPU6050_REG_FIFO_COUNTL    0x73
#define MPU6050_REG_WHO_AM_I       0x75    /* 只读，正常返回 0x68 */

/* 一次突发读得到的 14 字节原始数据，寄存器顺序 0x3B 到 0x48。 */
typedef struct {
    int16_t accel_x, accel_y, accel_z;   /* ACCEL_XOUT_H/L 起，各 2 字节，有符号 */
    int16_t temp;                        /* TEMP_OUT_H/L */
    int16_t gyro_x, gyro_y, gyro_z;      /* GYRO_XOUT_H/L 起 */
} MPU6050_RawData;
typedef struct {
    float accel_g[3];   /* 单位 g */
    float gyro_dps[3];  /* 单位度每秒 */
    float temp_c;       /* 单位摄氏度 */
} MPU6050_PhysData;
typedef struct
{
    float accel_g[3];
} MPU6050_AccelFilter;

typedef struct
{
    float sum[3];             /* 三轴角速度累计值 */
    float gyro_bias_dps[3];   /* 三轴零偏 */
    uint16_t sample_count;    /* 已采样次数 */
    uint16_t target_count;    /* 目标采样次数 */
    uint8_t done;             /* 校准完成标志 */
} MPU6050_GyroCalibration;
typedef struct
{
    float pitch;     // 俯仰角
    float roll;      // 横滚角

    float alpha;     // 互补滤波系数

} MPU6050_Attitude;

uint8_t MPU6050_CheckId(void);
uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadRaw(MPU6050_RawData *raw);
uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t value);
uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *value);
uint8_t Calibration_100();
//extern uint8_t ID ;

extern MPU6050_PhysData phys;
extern  MPU6050_AccelFilter accel_filter ;
extern MPU6050_GyroCalibration gyro_cal ;
extern MPU6050_Attitude attitude;
extern float gyro_dps[3] ;
void MPU6050_Task();

#endif 
