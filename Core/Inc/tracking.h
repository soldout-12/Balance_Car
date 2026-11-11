#ifndef TRACKING_H
#define TRACKING_H
#include "main.h"
extern uint8_t tracking_online;  /* 循迹传感器在线标志：1=Ping通过，0=超时。用循迹数据前先判断它 */
extern int16_t s_line_pos ;
extern int32_t sum ;
void Tracking_Task_Init();
void Tracking_Task();
/* 循迹位置解算（供控制环调用，内部已 15ms 抽稀）：
   返回 -7~+7，>0=线在右侧(需右转)，0=居中/离线。
   丢线时不立即回 0，而是保留最后一次方向并逐周期衰减，避免急弯滑出视野时突然回正。 */
int16_t LineSensor_ReadPos(void);
/* 丢线搜线标志：1=最黑通道也不够黑 或 连续IIC读失败，正在找线；
   传感器离线返回0(算循迹没开) */
uint8_t LineSensor_IsLost(void);
/* 连续丢线的读周期数，每周期约15ms。恢复状态机用 */
uint16_t LineSensor_LostCycles(void);
/* 停车/启动时复位丢线状态(计数清零、强制置为丢线、位置归零)，
   避免带着上次残留直接进恢复流程或误判"线还在"而窜车 */
void LineSensor_ResetState(void);
#endif
