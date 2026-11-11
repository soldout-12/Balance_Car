/*******************************************************************************
 * @file        
 * @author      eternal_fu
 * @version     V0.0.1
 * @brief       
 * @Date 2024-12-11 15:25:18
 * @LastEditTime 2024-12-11 16:37:29
 * @**************************************************************************** 
 * @attention   修改说明
 * @    版本号      日期         作者      说明
 *******************************************************************************/
#ifndef __PID_DEMO_H__
#define __PID_DEMO_H__

#include <stdint.h>     /* uint8_t */

/* pid结构体 */
typedef struct
{
    float kp;					/* 比例 */
    float ki;					/* 积分 */
    float kd;					/* 微分 */
    uint8_t enabled;            /* 1=该环参与控制，0=关闭 */
    float target;				/* 目标值 */
    float current;				/* 当前值 */
    float out;					/* 执行量 */
    float limit;                /* PID(out)输出限幅值 */
    float integral_limit;		/* 积分限幅：integral累加的上限，0=该环不积分 */

    float error;				/* 当前误差 */
    float last_error;			/* 上一次误差 */
    float last2_error;			/* 上上次误差 */
    float last_out;             /* 上一次执行量 */
	float integral;				/* 积分（累加） */
	float p_out,i_out,d_out;	/* 各控制量输出值 */
    float integral_separation;  /* pause integration during large speed error */
}PID_T;

/* ——————————————————————————要调的PID对象————————————————————————*/
/* 角度环 */
extern PID_T g_tAnglePID;
/* 左轮速度环 */
extern PID_T g_tLeftMotorPID;
/* 右轮速度环 */
extern PID_T g_tRightMotorPID;
//【1】
extern PID_T g_tSteeringPID;
/* 循迹环 */
extern PID_T g_tLinePID;
/*
    供用户调用的API
*/
void pid_Init(void);
uint8_t pid_IsActive(const PID_T *_tpPID);
void pid_reset(PID_T *_tpPID);
void pid_open(PID_T *_tpPID);
void pid_close(PID_T *_tpPID);
void pid_closeloop_angle(float _angle_target);
void pid_closeloop_motor(float _left_target, float _right_target);
void pid_closeloop_Steering(float _steer_target);
void pid_closeloop_line(float _line_target);

#if 0 /* 用户要用的值只有out */
    /* 在pid_closeloop_angle后，用户使用 */
    多环控制
		只用一个应用环：执行单元的PID输入(速度环input/目标值) = g_tAnglePID.out
	
		只用N个应用环：(边巡线、边角度) 执行单元的PID输入(速度环input/目标值) = g_tAnglePID.out + 巡线pid.out`
	
	car_run_closeloop();
    car_run(左轮速度环PID输出值, 同理)
	
while()
	pid_closeloop_angle(90) => g_tAnglePID.out

	pid_closeloop_motor(初速度 + _g_tAnglePID.out, 右轮初速度 - g_tAnglePID.out) // 要算两个轮子的PID

	car_run(leftpid.out rightpid.out)
#endif

/*
    PID相关的功能函数
    - 增量式
    - 位置式
    - 滤波算法
        - 卡尔曼滤波
        - ……
*/


#endif

