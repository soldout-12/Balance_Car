#include "mpu6050.h"
#include "iic.h"
#include "motor.h"
#include <math.h>
#include "usart.h"
#define RAD_TO_DEG 57.2957795f
#define PITCH_OFFSET 16.8f //机械中值误差
MPU6050_RawData raw;//存放14字节
MPU6050_PhysData phys;
MPU6050_AccelFilter accel_filter = {0};
MPU6050_GyroCalibration gyro_cal = {0};
float gyro_dps[3] = {0};
uint8_t ID = 0;
void MPU6050_GyroCalibrateInit(void); /* 定义在文件后部，Init 里要先调用一次。 */
/* mpu6050.c 节选。 */ 
uint8_t MPU6050_CheckId(void)
{
    uint8_t id = 0;
    uint8_t retry = 3;
    do {                      /* 上电瞬间可能需要几次才读到 ID。 */
        if (MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &id) == IIC_OK) {return IIC_OK; break; }
        retry--;
        HAL_Delay(5);
    } while (retry > 0);

    if (id != MPU6050_WHO_AM_I_VALUE) { return IIC_ERR_NACK; }   /* 地址或器件有问题。 */
}
/* mpu6050.c 节选续：只做 "配置 + 读数据"，业务解释交给上层。 */
uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    /* 参数检查放在驱动边界，避免把明显错误发到总线上。 */
    return IIC_WriteReg(MPU6050_ADDR_W, reg, value);
}

uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) { return IIC_ERR_PARAM; }
    return IIC_ReadReg(MPU6050_ADDR_W, reg, value);
}

uint8_t MPU6050_Init(void)
{
    

    /* 唤醒，并选择内部 8 MHz 时钟源；bit6 = 0 复位位，bit2:0 = 000 内部时钟。 */
    if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00) != IIC_OK) { return IIC_ERR_NACK; }
    HAL_Delay(50);           /* 退出睡眠后等振荡器稳定，留足比卡点更省时间。 */
    //检查id
    MPU6050_CheckId();

    /* 采样率分频：DLPF 打开时陀螺输出 1 kHz，1000 / (1 + 4) = 200 Hz。 */
    if (MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x04) != IIC_OK) { return IIC_ERR_NACK; }
    /* DLPF = 3：加速度带宽 44 Hz，陀螺带宽 42 Hz，抑制高频噪声。 */
    if (MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03) != IIC_OK) { return IIC_ERR_NACK; }
    /* 陀螺仪满量程 250 度每秒，131 LSB / (度每秒)。 */
    if (MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x00) != IIC_OK) { return IIC_ERR_NACK; }
    /* 加速度满量程 2 g，16384 LSB / g。 */
    if (MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00) != IIC_OK) { return IIC_ERR_NACK; }
    /* 六个轴都不待机。寄存器的复位值是单轴待机组合，必须显式写 0。 */
    if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_2, 0x00) != IIC_OK) { return IIC_ERR_NACK; }
    /* 除电源外全部使能，这里按调试期选择不开中断。 */
    MPU6050_GyroCalibrateInit(); /* 只在此初始化一次，避免 target_count 为 0 导致采样即"完成"。 */
    return IIC_OK;
}


uint8_t MPU6050_ReadRaw(MPU6050_RawData *raw)
{
    
    uint8_t buf[MPU6050_RAW_LEN];
    uint8_t ret;

    if (raw == NULL) { return IIC_ERR_PARAM; }   /* 边界检查先于总线操作。 */

    ret = IIC_ReadRegs(MPU6050_ADDR_W, MPU6050_REG_ACCEL_XOUT_H, buf, MPU6050_RAW_LEN);
    if (ret != IIC_OK) { return ret; }           /* 失败时 raw 内容不被写入。 */

    /* 寄存器高位在前，线路上也先到高字节，所以先收到的要高移 8 位。 */
    raw->accel_x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    raw->accel_y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
    raw->accel_z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
    raw->temp    = (int16_t)(((uint16_t)buf[6] << 8) | (uint16_t)buf[7]);
    raw->gyro_x  = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
    raw->gyro_y  = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
    raw->gyro_z  = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);

    return IIC_OK;
}
/*换算放在一处，后续换量程只改调用参数。 */
void MPU6050_Convert(const MPU6050_RawData *raw, MPU6050_PhysData *phys,
                     float accel_lsb_per_g, float gyro_lsb_per_dps)
{
    /* 先判空再解引用，物理层不应假设上层一定传了有效指针。 */
    if ((raw == NULL) || (phys == NULL)) { return; }

    phys->accel_g[0] = (float)raw->accel_x / accel_lsb_per_g;
    phys->accel_g[1] = (float)raw->accel_y / accel_lsb_per_g;
    phys->accel_g[2] =  (float)raw->accel_z / accel_lsb_per_g;

    phys->gyro_dps[0] = (float)raw->gyro_x / gyro_lsb_per_dps;
    phys->gyro_dps[1] = (float)raw->gyro_y / gyro_lsb_per_dps;
    phys->gyro_dps[2] =  (float)raw->gyro_z / gyro_lsb_per_dps;

	
}
//加速度低通滤波

void MPU6050_AccelLowPass(const MPU6050_PhysData *phys,
                          MPU6050_AccelFilter *filter,
                          float alpha)
{
    uint8_t i;

    if ((phys == NULL) || (filter == NULL))
    {
        return;
    }

    for (i = 0; i < 3; i++)
    {
        filter->accel_g[i] =
            alpha * filter->accel_g[i]
            + (1.0f - alpha) * phys->accel_g[i];
		//fdata[i]=filter->accel_g[i];
    }
}

//角速度零飘处理
void MPU6050_GyroCalibrateInit(void)
{
    uint8_t i;

    for (i = 0U; i < 3U; i++)
    {
        gyro_cal.sum[i] = 0.0f;
        gyro_cal.gyro_bias_dps[i] = 0.0f;
    }

    gyro_cal.sample_count = 0U;
    gyro_cal.target_count = 100U;
    gyro_cal.done = 0U;
}

uint8_t MPU6050_GyroCalibrateUpdate(
    MPU6050_GyroCalibration *cal,
    const MPU6050_PhysData *phys)
	
{
    uint8_t i;

    if ((cal == NULL) || (phys == NULL))
    {
        return 0U;
    }

    /* 已完成，不再重复累计 */
    if (cal->done)
    {
        return 1U;
    }

    /* 累计当前采样的三轴角速度 */
    cal->sum[0] += phys->gyro_dps[0];
    cal->sum[1] += phys->gyro_dps[1];
    cal->sum[2] += phys->gyro_dps[2];

    cal->sample_count++;

    /* 达到目标采样次数，计算平均零偏 */
    if (cal->sample_count >= cal->target_count)
    {
        for (i = 0U; i < 3U; i++)
        {
            cal->gyro_bias_dps[i] =
                cal->sum[i] / (float)cal->sample_count;
        }

        cal->done = 1U;
        return 1U;
    }

    return 0U;
}
uint8_t Calibration_100(){
if (MPU6050_ReadRaw(&raw) == IIC_OK) {
    MPU6050_Convert(&raw, &phys, 16384.0f, 131.0f);
    if (MPU6050_GyroCalibrateUpdate(&gyro_cal, &phys))
        {
            return 1;
        }
        return 0;
}
    return 0; /* 读失败时也要返回，否则函数无返回值是未定义行为。 */
}

void MPU6050_GyroCompensate(const MPU6050_PhysData *phys,
                            const MPU6050_GyroCalibration *cal,
                            float gyro_dps[3])
{
    uint8_t i;

    if ((phys == NULL) || (cal == NULL) || (gyro_dps == NULL))
    {
        return;
    }

    for (i = 0; i < 3; i++)
    {
        gyro_dps[i] = phys->gyro_dps[i] - cal->gyro_bias_dps[i];
        //fdata[i+3]= gyro_dps[i];
    }
	//fdata[2]=gyro_dps[1];
}
// 姿态融合 (互补滤波)
MPU6050_Attitude attitude =
{
    .pitch = 0.0f,
    .roll = 0.0f,
    .alpha = 0.98f
};
// 芯片绕 Z 轴旋转 180° 安装后，X/Y 符号已在 Convert 中取反。
// 对外语义保持不变：
//   X 轴 = 车头前后方向（前倾 → accel_g[0] 为正）
//   Y 轴 = 左右方向
//   Z 轴 = 垂直方向
// pitch(俯仰) 由 accel_g[0] 与 gyro_dps[1] 计算，roll(横滚) 由 accel_g[1] 与 gyro_dps[0] 计算。
static float Accel_GetPitch(MPU6050_AccelFilter *accel)
{
    return atan2f(accel->accel_g[1],
                  accel->accel_g[2])
           * RAD_TO_DEG;
}

//得到横滚角
static float Accel_GetRoll(MPU6050_AccelFilter *accel)
{
    return atan2f(accel->accel_g[0],
                  accel->accel_g[2])
           * RAD_TO_DEG;
}
void MPU6050_ComplementaryFilter( MPU6050_Attitude *attitude, MPU6050_AccelFilter *accel,float gyro_dps[3],float dt)
{
    float accel_pitch;
    float accel_roll;
    /*
     * 加速度计算角度
     */
    accel_roll = atan2f(accel->accel_g[1],accel->accel_g[2])* RAD_TO_DEG;
	
    accel_pitch = atan2f(accel->accel_g[0], accel->accel_g[2])* RAD_TO_DEG-PITCH_OFFSET  ;
    /*
     * 陀螺积分 + 加速度修正
     */
 
    attitude->pitch = attitude->alpha *(attitude->pitch -gyro_dps[1] * dt)+(1.0f-attitude->alpha) * accel_pitch;
	
 
    attitude->roll =attitude->alpha *(attitude->roll +gyro_dps[0] * dt) +(1.0f-attitude->alpha)* accel_roll;
	//fdata[1]=attitude->roll;//横滚角x-z
}


void MPU6050_Task(){
if (MPU6050_ReadRaw(&raw) == IIC_OK) {
    /* ±2 g 与 ±250 度每秒，与初始化写入的量程一致。 */
MPU6050_Convert(&raw, &phys, 16384.0f, 131.0f);
    //滤波
/* 加速度低通滤波 */
MPU6050_AccelLowPass(&phys, &accel_filter, 0.90f);
/* 陀螺仪零偏补偿 */
/* 正常运行时补偿陀螺仪零偏 */
MPU6050_GyroCompensate(&phys, &gyro_cal, gyro_dps);
//姿态融合 (互补滤波)
/*
 * dt 用 HAL_GetTick 实测，不能写死：主循环周期约 30ms，
 * 写 5ms 会让陀螺积分慢 6 倍，PID 拿到的角度是失真的。
 */
{
    static uint32_t last_tick = 0U;
    uint32_t now = HAL_GetTick();
    float dt = 0.005f;
    if (last_tick != 0U)
    {
        dt = (float)(now - last_tick) / 1000.0f;
        if (dt < 0.001f) { dt = 0.001f; }   /* 下限：防除零/异常 */
        if (dt > 0.1f)   { dt = 0.1f; }     /* 上限：卡顿一帧不爆积分 */
    }
    last_tick = now;
    MPU6050_ComplementaryFilter(&attitude, &accel_filter, gyro_dps, dt);
}
// PID 计算与电机输出
    Motor_Task();
}
}
