#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "main.h"
#define MOTOR_PWM_MAX 1000
// 定义平衡车的运行状态状态机
typedef enum {
    STATE_STAND = 0,    // 0: 静止站立（原地平衡，不动）
    STATE_TRACK,        // 1: 正常循迹（直线、弯道）
    STATE_LOST,         // 2: 丢线找线（保持前进，朝丢线前线的位置缓转）
    STATE_ESTOP,        // 3: 紧急停止（倒地保护或按键急停）
} CarState_t;
/* 电机死区补偿：减速电机静态摩擦需要一定占空比才能起步，平衡输出在
   零附近来回过零时每次都被摩擦"吃掉"，轮子一顿一顿。pwm 非零且幅值
   大于 MOTOR_COMP_MIN_PWM 时叠加 ±MOTOR_DEADZONE 作为摩擦前馈。
   MOTOR_DEADZONE 需实测：从 60~100 逐步调小，找到"刚好能转"的
   最小值后再加 20% 余量。 */
#define MOTOR_DEADZONE        40.0f   /* 死区补偿量(占空比计数)，需实测 */
#define MOTOR_COMP_MIN_PWM     5.0f   /* 低于此值视为零附近的静止噪声 */
extern float Line_Follow_Speed ;
extern uint16_t Motor1_Duty_Cycle;
extern uint16_t Motor2_Duty_Cycle;
extern volatile uint8_t g_speed_fresh;
//测试

extern float s_speed_cmd ;//速度曲线
extern float tilt_offset;	//速度环输出的平衡点偏移量(°)，不是MPU实测俯仰角
extern float pwm_L;
extern  float pwm_R;
extern float Balance_Target;
extern CarState_t current_state;
void Motor_Task();
/* 电机直通测试：on=1 时两个电机以固定占空比正反转（绕开全部PID），
   约5秒后自动停止；用于隔离"MCU引脚->驱动->电机"链路。按键需保持未使能 */
void Motor_Test_Set(uint8_t on);
#endif
