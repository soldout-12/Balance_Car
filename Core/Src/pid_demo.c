/*******************************************************************************
 * @file        pid_demo
 * @author      eternal_fu
 * @version     V0.0.1
 * @brief       实现了PID公式，各种环的闭环控制，部分PID功能函数
 * @Date 2024-12-11 15:25:07
 * @LastEditTime 2024-12-11 16:35:47
 * @**************************************************************************** 
 * @attention   修改说明
 * @    版本号      日期         作者      说明
 *******************************************************************************/
#include "pid_demo.h"
#include "mpu6050.h"
#include "usart.h"
#include "tracking.h"   /* LineSensor_ReadPos：GW_GRAY 循迹位置 */
/* 声明功能函数 */
static void pid_formula_incremental(PID_T * _tpPID);
static void pid_formula_positional(PID_T * _tpPID);
static void pid_out_limit(PID_T * _tpPID);

/* 当前系统的PID */
PID_T g_tAnglePID = {0};        /* 角度环 */
PID_T g_tLeftMotorPID = {0};    /* 左轮速度环 */
PID_T g_tRightMotorPID = {0};   /* 右轮速度环 */
PID_T g_tSteeringPID= {0};           /* 转向环 */
PID_T g_tLinePID = {0};         /* 循迹环：线位置偏差->差速PWM */
//...【2】

/*******************************************************************************
 * @brief   
 * @return {*}
 * @note    none
 *******************************************************************************/
static void pid_InitVal(void)
{
	/* 系统 - 角度环 */
	g_tAnglePID.enabled = 1U;
	g_tAnglePID.kp = -97.0;		    // 比例：error已统一为target-current，kp取负才是"倒下去向前救"
	g_tAnglePID.ki = 0;			    // 积分
	g_tAnglePID.integral_limit = 0;	// 角度环不用I：设为0则积分通路关闭
	g_tAnglePID.kd = -6;			// 微分：error取反后kd必须同时变负，否则变成正反馈
	g_tAnglePID.target = 0;			// 目标值[可以在外部调用]
	g_tAnglePID.limit = 1000;		// 限幅值

    /* 系统 - 左轮速度环 */
	g_tLeftMotorPID.enabled = 1U;
	g_tLeftMotorPID.kp = 0.08f;		    // 比例：降低大误差时的瞬时倾角冲击
	g_tLeftMotorPID.ki = 0.002f;			// 积分：提高稳态速度保持能力
	g_tLeftMotorPID.integral_limit = 1000;	// 积分限幅：i_out最大2°的平衡点修正
	g_tLeftMotorPID.kd = 0.0f;			// 微分：抑制加速后减速、减速后再加速的外环摆动
	g_tLeftMotorPID.target = 0;			// 目标值[可以在外部调用]
	g_tLeftMotorPID.limit = 21;		// 限幅值：注意单位是"度"(角度环平衡点偏移)，不是PWM
	g_tLeftMotorPID.integral_separation = 0.0f;	/* 必须为0：静止时error=目标速度(20)恒>门限5，
                                                   积分被永久禁掉 -> 车不动->误差大->没积分->更不动，死锁 */

    /* 系统 - 右轮速度环 */
	g_tRightMotorPID.enabled = 1U;
	g_tRightMotorPID.kp = 0.08f;		    // 比例：降低大误差时的瞬时倾角冲击
	g_tRightMotorPID.ki = 0.002f;			// 积分：提高稳态速度保持能力
	g_tRightMotorPID.integral_limit = 1000;	// 积分限幅：i_out最大2°的平衡点修正
	g_tRightMotorPID.kd = 0.0f;			// 微分：抑制加速后减速、减速后再加速的外环摆动
	g_tRightMotorPID.target = 0;		// 目标值[可以在外部调用]
	g_tRightMotorPID.limit = 21;		// 限幅值：注意单位是"度"(角度环平衡点偏移)，不是PWM
	g_tRightMotorPID.integral_separation = 0.0f;	/* 同左轮：见 integral_separation 死锁说明 */
	
	//【3】
	g_tSteeringPID.enabled = 1U;
	g_tSteeringPID.kp = 0.0f;		    // 比例：yaw角速度误差(°/s)->差速PWM，起步0.5~1.0，手拧测试后逐步加
	g_tSteeringPID.ki = 0;			    // 积分
	g_tSteeringPID.integral_limit = 0;	// 转向环暂不用I
	g_tSteeringPID.kd = 0;			    // 微分
	g_tSteeringPID.target = 0;			    // 目标值[可以在外部调用]
	g_tSteeringPID.limit = 300;		        // 限幅：差速修正量，须>0且小于角度环limit，0会让输出恒为0

    /* 系统 - 循迹环 */
	g_tLinePID.enabled = 1U;
	g_tLinePID.kp = -25.0f;		    // 比例：线位置(-7~+7)->差速PWM。跑偏发散就把符号取反，再调大小
	g_tLinePID.ki = 0;			    // 积分：不用
	g_tLinePID.integral_limit = 0;	// 循迹不用I：0=积分通路关闭
	g_tLinePID.kd = 0;			    // 微分：先0，循迹蛇形/过冲再加 -1~-3 试
	g_tLinePID.target = 0;			// 目标值：0=线居中
	g_tLinePID.limit = 110;		    // 限幅：差速修正量。循迹接管差速通道时转向环已让位，这个值就是偏航权限上限：
								    // 丢线/贴边时打转就降到100~120，跟不到线再微调；别指望转向环来兜底

}



/* —————————————————————————— 供用户调用的API —————————————————————————— */
/*******************************************************************************
 * @brief   
 * @return {*}
 * @note    none
 *******************************************************************************/
void pid_Init(void)
{
    pid_InitVal();
}

/* 该环是否生效：kp/ki/kd 全为 0 即视为关闭（VOFA 里把某个 Kp 置 0 就是关环）。
   循迹环与转向环共用同一条差速通道，motor.c 用这个判断当前谁接管通道。 */
uint8_t pid_IsActive(const PID_T *_tpPID)
{
    if ((_tpPID == 0) || (_tpPID->enabled == 0U)) {
        return 0U;
    }
    return ((_tpPID->kp != 0.0f) || (_tpPID->ki != 0.0f) || (_tpPID->kd != 0.0f)) ? 1U : 0U;
}


/* 速度反馈量化噪声处理：死区 + 一阶低通。
   静止时编码器只有 ±1 个计数的分辨率，直接进速度环会把噪声打进平衡点
   （P 路径每两周期跳一次目标角，I 路径积分成随机游走），表现为"抖着平衡"。 */
#define SPEED_DEADBAND   0       /* |计数/10ms| 低于此值视为静止 */
#define SPEED_LPF_ALPHA  0.1f    /* 低通系数：越大越跟手，越小越平滑 */

static float s_speed1_filt = 0.0f;
static float s_speed2_filt = 0.0f;

static float Speed_FeedbackFilter(int16_t raw, float *filt)
{
    if ((raw > -SPEED_DEADBAND) && (raw < SPEED_DEADBAND)) {
        raw = 0;                              /* 死区：掐掉量化噪声种子 */
    }
    *filt += SPEED_LPF_ALPHA * ((float)raw - *filt);   /* 一阶低通 */
    return *filt;                             /* 保留小数，低速计数不能被截断成0 */
}

/* —————————————————————————— 闭环控制[仅计算出out值] —————————————————————————— */
/*******************************************************************************
 * @brief   角度闭环控制函数[角度环]
 * @return {*}
 * @note    在位置式中，P-快速性，I-准确性，D-稳定性【一般按顺序给P,D】
 *******************************************************************************/
void pid_closeloop_angle(float _angle_target)
{
    /* [更新目标值] */
    g_tAnglePID.target = _angle_target;

    /* [获取反馈值/当前值] - 获取当前角度 */
    g_tAnglePID.current = -attitude.pitch;   /* 直接使用IMU姿态，避免VOFA覆盖fdata[0] */

    /* [通过PID公式计算PID输出值] */
    pid_formula_positional(&g_tAnglePID);
    
    /* [限幅PID输出] */
    pid_out_limit(&g_tAnglePID);
}

/*******************************************************************************
 * @brief   电机闭环控制函数[速度环]
 * @return {*}
 * @note    在增量式中，P-稳定性，I-快速性，D-准确性【一般按顺序给I,P】
 *			计算出out值后，在其他地方car_run(out1, out2);
 *******************************************************************************/
void pid_closeloop_motor(float _left_target, float _right_target)
{
    /* [更新目标值] */
    g_tLeftMotorPID.target = _left_target;
    g_tRightMotorPID.target = _right_target;

    /* [获取反馈值/当前值] - 获取当前双轮速度(current) */
    g_tLeftMotorPID.current  = Speed_FeedbackFilter(speed1, &s_speed1_filt);
    g_tRightMotorPID.current = Speed_FeedbackFilter(speed2, &s_speed2_filt);

    /* [通过PID公式计算PID输出值] */
    pid_formula_positional(&g_tLeftMotorPID);
    pid_formula_positional(&g_tRightMotorPID);
    
    /* [限幅PID输出] */
    pid_out_limit(&g_tLeftMotorPID);
    pid_out_limit(&g_tRightMotorPID);
}

//【4】添加自己闭环控制
/*******************************************************************************
 * @brief   转向闭环控制函数[转向环]
 * @return {*}
 * @note    反馈用陀螺仪Z轴角速度(偏航角速度)，输出是左右轮差速修正量：
 *          左轮PWM = 角度环输出 + g_tSteeringPID.out
 *          右轮PWM = 角度环输出 - g_tSteeringPID.out
 *          若上车后往一个方向猛转，说明yaw反馈符号与电机方向不匹配，
 *          把 kp 改成负数（或 current 取负）即可。
 *******************************************************************************/
void pid_closeloop_Steering(float _steer_target)
{
    if (g_tSteeringPID.enabled == 0U) {
        g_tSteeringPID.out = 0.0f;
        return;
    }

    /* [更新目标值] - 期望的偏航角速度(°/s)，0表示保持直行 */
    g_tSteeringPID.target = _steer_target;

    /* [获取反馈值/当前值] - 陀螺仪Z轴角速度(已扣除零偏) */
    g_tSteeringPID.current = gyro_dps[2];

    /* [通过PID公式计算PID输出值] */
    pid_formula_positional(&g_tSteeringPID);

    /* [限幅PID输出] */
    pid_out_limit(&g_tSteeringPID);
}

/*******************************************************************************
 * @brief   循迹闭环控制函数[循迹环]
 * @note    反馈=线位置(LineSensor_ReadPos)，目标=0(线居中)，
 *          输出差速PWM叠加方式与转向环一致：左+=out，右-=out。
 *          符号验证：黑线在右(读数为正)时车应右转；若左转=发散，kp取反。
 *******************************************************************************/
void pid_closeloop_line(float _line_target)
{
    /* 即使循迹环关闭也要更新线位置：丢线恢复状态机依赖它判断何时找回线。 */
    if (!pid_IsActive(&g_tLinePID)) {
        g_tLinePID.target = _line_target;
        g_tLinePID.current = (float)LineSensor_ReadPos();
        g_tLinePID.error = 0.0f;
        g_tLinePID.last_error = 0.0f;
        g_tLinePID.integral = 0.0f;
        g_tLinePID.out = 0.0f;
        g_tLinePID.p_out = 0.0f;
        g_tLinePID.i_out = 0.0f;
        g_tLinePID.d_out = 0.0f;
        return;
    }

    /* [更新目标值] - 期望的线位置，0=居中 */
    g_tLinePID.target = _line_target;

    /* [获取反馈值/当前值] - 传感器线位置 */
    g_tLinePID.current = (float)LineSensor_ReadPos();

    /* [通过PID公式计算PID输出值] */
    pid_formula_positional(&g_tLinePID);

    /* [限幅PID输出] */
    pid_out_limit(&g_tLinePID);
}

/* —————————————————————————— PID相关的功能函数 —————————————————————————— */
/*******************************************************************************
 * @brief   
 * @return {*}
 * @note    none
 * @param {PID_T *} _tpPID
 *******************************************************************************/
static void pid_out_limit(PID_T * _tpPID)
{
    if(_tpPID->out > _tpPID->limit)
		_tpPID->out = _tpPID->limit;
    else if(_tpPID->out < -_tpPID->limit)
	    _tpPID->out = -_tpPID->limit;
}

/*******************************************************************************
 * @brief   增量式PID公式
 * @return {*}
 * @note    在增量式中，P-稳定性，I-快速性，D-准确性【一般按顺序给I,P】
 * @param {PID_T *} _tpPID  传入要运算的PID对象的指针
 *******************************************************************************/
static void pid_formula_incremental(PID_T * _tpPID)
{
    _tpPID->error = _tpPID->target - _tpPID->current;															
    
	_tpPID->p_out = _tpPID->kp * (_tpPID->error - _tpPID->last_error);
	_tpPID->i_out = _tpPID->ki * _tpPID->error;
	_tpPID->d_out = _tpPID->kd * (_tpPID->error - 2 * _tpPID->last_error + _tpPID->last2_error);
	
	_tpPID->out += _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;
	
	_tpPID->last2_error = _tpPID->last_error;																
	_tpPID->last_error = _tpPID->error;
}

/*******************************************************************************
 * @brief   位置式PID公式
 * @return {*}
 * @note    在位置式中，P-快速性，I-准确性，D-稳定性【一般按顺序给P,D】
 * @param {PID_T *} _tpPID  传入要运算的PID对象的指针
 *******************************************************************************/
static void pid_formula_positional(PID_T * _tpPID)
{
    /* 本次计算用到的临时变量：候选积分值、候选总输出 */
    float integral_next;
    float out_next;
    /* 积分通路开关：默认允许积分，下面的积分分离可能把它关掉 */
    uint8_t integrate_enabled = 1U;

    /* 误差统一为"目标-当前"：正数表示还没到目标，需要正方向使劲 */
    _tpPID->error = _tpPID->target - _tpPID->current;
    /* 比例项：误差立刻转成输出，负责快速响应 */
    _tpPID->p_out = _tpPID->kp * _tpPID->error;
    /* 微分项：用"本次误差-上次误差"近似误差变化率，负责阻尼、抑制过冲。
       启动时 last_error 为 0，第一拍会有一次跳变，所以 pid_open/pid_reset
       会清掉历史，避免带着上一轮的残留误差算微分 */
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - _tpPID->last_error);

    /* ---------- 积分分离 ----------
       目的：大误差阶段先让 P 单独干活，积分暂停；等误差小了再让积分接手。
       不这么做的话，大误差时 I 会一直往一个方向猛攒(windup)，等误差终于
       变小了积分项还挂在那儿，输出照样顶到限幅，表现为超调、松手后自己蹿。
       三个条件同时满足才暂停积分：
         1) integral_separation > 0      ：该环启用了积分分离(0=功能关闭)
         2) |error| > integral_separation：误差已经大到需要"先别积分"的门限外
         3) error * integral >= 0        ：误差与积分同号，说明积分还在往同一
                                           方向使劲，才需要喊停；异号表示积分
                                           已经在往回拉了(起制动作用)，不动它
       注意：平衡车速度环必须把这个门限设 0！起步时 error 恒等于目标速度，
       任何非零门限都会把积分永久禁掉，形成"车不动->误差大->没积分->更不动"
       的死锁(见 pid_InitVal 里 g_tLeftMotorPID.integral_separation 的注释)。 */
    if ((_tpPID->integral_separation > 0.0f) &&
        ((_tpPID->error > _tpPID->integral_separation) ||
         (_tpPID->error < -_tpPID->integral_separation)) &&
        ((_tpPID->error * _tpPID->integral) >= 0.0f)) {
        integrate_enabled = 0U;
    }

    /* ---------- 积分累加 ----------
       两个前提都满足才积分：
         integral_limit > 0    ：该环想用积分(0=积分通路整体关闭，角度环/转向环/循迹环都是)
         integrate_enabled     ：没被上面的积分分离叫停 */
	if ((_tpPID->integral_limit > 0.0f) && integrate_enabled)
	{
	    /* 候选值=旧积分+本次误差，先算成临时值，过不了后面的检查就不提交 */
		integral_next = _tpPID->integral + _tpPID->error;
		/* 积分限幅：把积分本身夹在 ±integral_limit 内，这是防 windup 的主防线。
		   只限输出不够：输出被限住后误差仍然存在，积分会继续无上限地涨，
		   等误差反向时要花很长时间才能把这笔"积分债"还清 */
		if (integral_next >  _tpPID->integral_limit) integral_next =  _tpPID->integral_limit;
		else if (integral_next < -_tpPID->integral_limit) integral_next = -_tpPID->integral_limit;

		/* 条件积分(抗windup第二道防线)：用候选积分算出候选总输出，
		   如果输出已经顶到限幅、而误差还在同一方向继续推，
		   说明积分再加下去也只会让输出更饱和，纯属浪费，
		   这次就不提交(integral_next 回退成旧值 integral)。 */
		out_next = _tpPID->p_out + _tpPID->ki * integral_next + _tpPID->d_out;
		if (((out_next > _tpPID->limit) && (_tpPID->error > 0.0f)) ||
		    ((out_next < -_tpPID->limit) && (_tpPID->error < 0.0f))) {
			integral_next = _tpPID->integral;
		}
		/* 检查全部通过，正式提交积分值 */
		_tpPID->integral = integral_next;
	}
	/* 积分项输出：放在 if 外面，即使本轮没积分，历史积分仍要参与输出 */
	_tpPID->i_out = _tpPID->ki * _tpPID->integral;
	/* 位置式输出：三项直接相加(不是增量式，输出是绝对量，不带积分累积) */
	_tpPID->out =  _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;
	/* 保存本次误差，供下一拍的微分和积分分离使用 */
	_tpPID->last_error = _tpPID->error;
}

/*******************************************************************************
 * @brief   复位PID内部状态（误差、积分、输出）
 * @note    停车/重新使能时调用速度环复位，清掉残留积分，避免重启瞬间蹿车
 *******************************************************************************/
void pid_reset(PID_T * _tpPID)
{
	_tpPID->error = 0;
	_tpPID->last_error = 0;
	_tpPID->last2_error = 0;
	_tpPID->integral = 0;
	_tpPID->out = 0;
	_tpPID->p_out = 0;
	_tpPID->i_out = 0;
	_tpPID->d_out = 0;
}
/* 显式打开一个环：只使能并清一次历史状态，不修改 kp/ki/kd。
   重复调用是安全的，不会每个控制周期都清空积分。 */
void pid_open(PID_T *_tpPID)
{
    if ((_tpPID == 0) || (_tpPID->enabled != 0U)) {
        return;
    }
    pid_reset(_tpPID);
    _tpPID->enabled = 1U;
}

/* 显式关闭一个环：先清历史状态再禁用，避免重新打开时带着旧积分。 */
void pid_close(PID_T *_tpPID)
{
    if ((_tpPID == 0) || (_tpPID->enabled == 0U)) {
        return;
    }
    pid_reset(_tpPID);
    _tpPID->enabled = 0U;
}
