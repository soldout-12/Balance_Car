#include "motor.h"
#include "tim.h"
#include "mpu6050.h"
#include "pid_demo.h"
#include "tracking.h"
#include "usart.h"    /* fdata[] 在此声明 */

uint16_t Motor1_Duty_Cycle;
uint16_t Motor2_Duty_Cycle;


float Blance_Target = -5.4f;
float Line_Follow_Speed =24.0f;   /* 循迹目标速度(编码器计数/10ms)，5~10先慢后快 */

/* 弯道速度规划：中心附近保持全速，偏差越大目标速度越低。
   位置突变说明已经急入弯，需要额外降速；恢复直线后缓慢加速（加速比减速慢得多，
   形成"慢进快出"）。参数在高速直道+急弯场景下按"宁可弯里慢、直道补回来"取向。
   进入降速用 FULL_POS、退出用 FULL_EXIT，两者错开形成回差：否则 pos 在门限附近
   抖动时目标速度在 10↔2 之间阶跃跳变，砸进 tilt_offset 就是"速度不稳"。 */
#define LINE_SPEED_FULL_POS        2//进入降速门限，偏离距离单位0-2
#define LINE_SPEED_FULL_EXIT       3//退出降速门限(>FULL_POS)，回差来源
#define LINE_SPEED_MAX_POS         7//最大pos为7
#define LINE_SPEED_MIN_SCALE       0.22f//静态最低速度比例。偏差到7时降到22%
#define LINE_SPEED_TREND_POS       2//位置突变量门限：15ms内跳2格即判定急入弯
#define LINE_SPEED_TREND_SCALE     0.70f//急入弯额外减速比(在静态减速基础上再乘)
#define LINE_SPEED_EMERGENCY_SCALE 0.12f//双重减速的地板值；必须明显低于静态×动态乘积，否则trend等于没生效
#define SPEED_CMD_ACCEL_STEP       0.15f//速度斜坡函数加速步长(慢)
#define SPEED_CMD_DECEL_STEP       0.50f//速度斜坡函数减速步长(快)
#define SPEED_CMD_ZERO_STEP        1.00f//跨越零点(前进↔倒车)专用快斜坡：否则从-6爬到+10要1秒，找回归后车还会继续倒很久



// 声明一个全局状态变量，记录当前状态
CarState_t current_state = STATE_STAND;

/* ———— 丢线恢复：保持前进朝丢线前线的位置缓转，直到重新看到线 ————
   用户要求不刹停、不倒车，所以流程是：
     STATE_TRACK 发现丢线 → STATE_LOST(记录丢线瞬间 s_line_pos 的正负)
     → 保持前进 + 朝线的一侧缓转，传感器重新看到线立刻回 STATE_TRACK。
   方向依据：丢线前 s_line_pos>0 表示线在右侧，就继续往右转；<0 往左转。
   这是"往线的位置走"，不是原地转向：前进速度不归零，转向输出受斜坡限幅，
   所以车身是边走边拐，不会钉在原地打转。 */
#define REC_LOST_KEEP_SPEED      1.0f  /* 丢线期间保持的前进速度(占满速比例)。
                                         原来 0.5 太保守：降速后平衡点修正变慢，
                                         车还没走到线边上就先失稳。1.0 与循迹同速 */
#define REC_LOST_STEER_KP        80.0f /* 找线转向强度：把丢线前的位置正负放大成
                                         差速 PWM。太小转不动、太大原地打转 */
#define REC_LOST_STEER_LIMIT     400.0f/* 找线差速上限：必须明显小于 MOTOR_PWM_MAX，
                                         否则左右轮一路打满差速，车就是原地转 */
#define REC_LOST_STEER_SLEW      8.0f  /* 找线差速斜坡步长(PWM/周期)：
                                         转向量每周期最多涨这么多，把"瞬间满舵"
                                         摊成"边走边缓转"，避免砸进平衡点 */
#define REC_LOST_MIN_DWELL_MS    100U  /* 恢复最短驻留：进 LOST 至少待这么久才允许
                                         被"看到线"打断，挡掉丢线瞬间传感器边缘的
                                         单拍假信号(否则一进 LOST 就回 TRACK，死循环) */
#define REC_LOST_TIMEOUT_MS      1500U /* 找线上限：一直没找回归就退回 STAND 重新站好，
                                         否则可能一直朝一个方向绕出去 */

static uint8_t s_line_was_seen = 0;  /* 本次启动后是否真在线上跑过。
                                        没跑过就丢线不算"冲出去"，否则刚按键车还没上赛道
                                        就会先开始找线，观感像故障 */
static int8_t s_lost_dir = 0;        /* 丢线瞬间线的方向：+1=线在右(往右找)，-1=线在左(往左找) */
static float s_lost_steer = 0.0f;    /* 找线差速输出(已过斜坡)，由 0 渐入到目标值 */

float s_speed_cmd = 0.0f;            /* 实际发给速度环的速度指令(已过斜坡)，VOFA 通道4 */
static float s_speed_target = 0.0f;  /* 状态机算出的目标速度，还没过斜坡 */
static int16_t s_prev_speed_line_pos = 0;
static uint8_t s_slow_active = 0;     /* 弯道降速记忆(带回差)，停车必须清零 */
static uint16_t s_rec_ms = 0;        /* 当前丢线恢复已持续的毫秒数：最短驻留、超时都用它 */
static float s_tilt_last = 0.0f;     /* 平衡点偏移的限幅基准；停车必须清零，否则重启带着上次值 */

/* 丢线找线专用的差速通道：与循迹 line_out 互斥，同一路差速只让一方接管。
   输出随斜坡渐入(不是一进来就满舵)，这样车是"边走边拐"而不是原地打转。
   符号沿用循迹环的约定：line_out>0 = 左轮加、右轮减 = 车右转。 */
static float Lost_SteerOut(void)
{
    float steer_target = (float)s_lost_dir * REC_LOST_STEER_KP;
    float delta = steer_target - s_lost_steer;

    if (delta > REC_LOST_STEER_SLEW) {
        s_lost_steer += REC_LOST_STEER_SLEW;
    } else if (delta < -REC_LOST_STEER_SLEW) {
        s_lost_steer -= REC_LOST_STEER_SLEW;
    } else {
        s_lost_steer = steer_target;
    }

    if (s_lost_steer >  REC_LOST_STEER_LIMIT) s_lost_steer =  REC_LOST_STEER_LIMIT;
    if (s_lost_steer < -REC_LOST_STEER_LIMIT) s_lost_steer = -REC_LOST_STEER_LIMIT;
    return s_lost_steer;
}



/* 编码器差速纠偏：左右轮里程差(计数/10ms)->差速PWM，专治电机/轮胎
   不对称导致的直线跑偏。enc_diff>0 表示左轮偏快(车向右跑偏)，应压左轮、
   抬右轮。若装车后反而往一侧跑偏或打转，说明符号与电机方向不匹配，
   把 STEER_ENC_KP 改成负值即可。与转向陀螺仪环互补：陀螺仪管"别转"，
   差速管"别偏"，两个都要。 */
#define STEER_ENC_KP   0.0f   /* 起步0.3~0.5，直线还漂就加大，抖就减小 */

/* 循迹模式：1=循迹环生效并给定前进爬行速度。先装传感器、单独验证
   LineSensor_ReadPos 读数极性正确后再开，0=纯平衡/遥控模式 */



/* 启动自动校准平衡中值：
   使能后只在"车近似静止"(|θ|和角速度都小)时采样俯仰角，累计够
   MID_CAL_SAMPLES 个后取平均写入 Blance_Target，消除重心/安装不对称
   带来的直流偏置（手动值-2.7会被覆盖）。不按固定时间卡：扶着或自稳都行，
   最长等 MID_CAL_MAX_WAIT 个周期，超时未完成则放弃、保留当前值。
   注意：它只治"歪/漂"，不治"晃"。0=关闭，恢复手动调 Blance_Target */
#define AUTO_BALANCE_MID    0
#define MID_CAL_SAMPLES     400     /* 静止采样个数，约2~4秒采完 */
#define MID_CAL_MAX_WAIT    3000    /* 最长等待约15秒(按5ms控制周期估算) */
#define MID_CAL_ANGLE_TH    10.0f   /* 静止判据：|俯仰角|阈值(°) */
#define MID_CAL_GYRO_TH     20.0f   /* 静止判据：|俯仰角速度|阈值(°/s) */

static void Direction_L(uint8_t dire)
{
    if (dire == 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else if (dire == 1) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    } else if (dire == 2) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }
}

static void Direction_R(uint8_t dire)
{
    if (dire == 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else if (dire == 1) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    } else if (dire == 2) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
}

/* 死区补偿：非零小信号叠加固定摩擦前馈，跨过静态摩擦。
   不在高占空比处撤销补偿，否则 299 -> 339、300 -> 300 会形成反向下跳。 */
static float Motor_DeadzoneComp(float pwm)
{
    if (pwm > MOTOR_COMP_MIN_PWM) {
        pwm += MOTOR_DEADZONE;
    } else if (pwm < -MOTOR_COMP_MIN_PWM) {
        pwm -= MOTOR_DEADZONE;
    }
    if (pwm >  MOTOR_PWM_MAX) pwm =  MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;
    return pwm;
}

static void Motor_L_Write(float pwm)
{
    pwm = Motor_DeadzoneComp(pwm);
    if (pwm > 0) {
        Direction_L(0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm);
    } else if (pwm < 0) {
        Direction_L(1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(-pwm));
    } else {
        Direction_L(2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
}

static void Motor_R_Write(float pwm)
{
    pwm = Motor_DeadzoneComp(pwm);
    if (pwm > 0) {
        Direction_R(0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)pwm);
    } else if (pwm < 0) {
        Direction_R(1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(-pwm));
    } else {
        Direction_R(2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

void Set_Motor_PWM(float pwm_L, float pwm_R)
{
   
   

    Motor_L_Write(pwm_L);
    Motor_R_Write(pwm_R);
}

float tilt_offset;	//速度环输出：平衡点偏移量(°)。与MPU实测俯仰角fdata[0]完全是两个量，勿混淆
float pwm_L;
float pwm_R;
float Line_SpeedUpdate(void)
{
    // 1. 获取当前的外部输入（传感器数据、按键等）
    int16_t current_pos = s_line_pos;
    int16_t pos_abs = (current_pos < 0) ? -current_pos : current_pos;
    float desired_speed = Line_Follow_Speed;
    float diff = (float)(current_pos - s_prev_speed_line_pos); /* 15ms 内的位置突变量 */
    s_prev_speed_line_pos = current_pos;

    /* 弯道降速标志：进入门限 2、退出门限 3。带记忆才形成回差，否则 pos 在 2/3
       之间抖动时目标速度就在 10↔2 阶跃跳变，经 tilt_offset 砸进平衡点就是"速度不稳" */
    if (s_slow_active == 0U) {
        if (pos_abs > LINE_SPEED_FULL_POS) s_slow_active = 1U;
    } else {
        if (pos_abs <= LINE_SPEED_FULL_EXIT) s_slow_active = 0U;
    }

    /* 急入弯：位置突变量超过门限，在静态降速基础上再乘一档 */
    float trend_abs = (diff < 0.0f) ? -diff : diff;
    uint8_t trend_hit = (trend_abs > LINE_SPEED_TREND_POS) ? 1U : 0U;

    /* 2. 状态机核心：根据当前状态决定执行什么动作 */
    switch (current_state)
    {
        case STATE_STAND://初始状态
            desired_speed = 0.0f;
            if (Key_Flag && tracking_online) {
                current_state = STATE_TRACK;
                pid_open(&g_tLinePID);
                pid_close(&g_tSteeringPID);
            }
            break;

        case STATE_TRACK://启动循迹
            pid_open(&g_tLinePID);
            pid_close(&g_tSteeringPID);
            desired_speed = Line_Follow_Speed;

            if (LineSensor_IsLost() || (pos_abs > LINE_SPEED_MAX_POS)) {
                s_rec_ms = 0;
                s_lost_steer = 0.0f;
                /* 记下丢线瞬间线在哪一侧：这就是接下来要找的方向。
                   pos 已经衰减过(见 tracking.c)，这里取的是"还没衰减完"的最后方向，
                   只要符号还在就能用；万一正好衰减到 0，默认往右找 */
                s_lost_dir = (current_pos < 0) ? -1 : 1;
                current_state = STATE_LOST;
            } else {
                s_line_was_seen = 1U;
                if (s_slow_active) {
                    desired_speed = Line_Follow_Speed * LINE_SPEED_MIN_SCALE;
                }
                if (trend_hit) {
                    desired_speed *= LINE_SPEED_TREND_SCALE;
                    if (desired_speed < Line_Follow_Speed * LINE_SPEED_EMERGENCY_SCALE) {
                        desired_speed = Line_Follow_Speed * LINE_SPEED_EMERGENCY_SCALE;
                    }
                }
            }
            break;

        case STATE_LOST://丢线找线：保持前进，朝丢线前线的位置缓转
            /* 前进不停：刹停/倒车都删了，找线靠"边走边拐"完成 */
            desired_speed = Line_Follow_Speed * REC_LOST_KEEP_SPEED;
            pid_close(&g_tLinePID);
            pid_open(&g_tSteeringPID);

            /* 最短驻留没过：此时"看到线"极可能是丢线瞬间线从传感器边缘
               扫过的假信号，一看见就回 TRACK 会立刻又丢，形成死循环 */
            if ((s_rec_ms >= REC_LOST_MIN_DWELL_MS) &&
                !LineSensor_IsLost() && (pos_abs <= LINE_SPEED_MAX_POS)) {
                pid_open(&g_tLinePID);
                pid_close(&g_tSteeringPID);
                s_lost_steer = 0.0f;
                current_state = STATE_TRACK; // 找到线了，恢复循迹
             } 
            //else if (s_rec_ms >= REC_LOST_TIMEOUT_MS) {
            //     /* 一直没找到：别朝一个方向绕出去，退回 STAND 重新站好等人复位 */
            //     s_lost_steer = 0.0f;
            //     //current_state = STATE_STAND;
            // }
            break;

        case STATE_ESTOP:
            // --- 动作：紧急刹车，电机抱死或关闭PWM ---
            desired_speed = 0.0f;
            // 强制关闭电机输出（或者调用刹车函数）
            Set_Motor_PWM(0.0f, 0.0f);
            // --- 转移条件：需要人工复位才能恢复 ---
            if (!Key_Flag) {
                current_state = STATE_STAND;
            }
            break;
        default:
            current_state = STATE_STAND; // 防止跑飞
            desired_speed = 0.0f;
            break;
    }

    /* 没在线上跑过就丢线，不启动找线：刚按键车还没上赛道，否则会莫名开始转 */
    if ((current_state != STATE_TRACK) && (s_line_was_seen == 0U)) {
        desired_speed = 0.0f;
    }

    /* 3. 速度指令斜坡：目标速度阶跃直接进速度环会被积分放大成一次前冲/后冲。
       加速慢(慢进快出)、减速快、跨零最快。找线是从满速降下来的，走减速档即可 */
    float ramp_accel = SPEED_CMD_ACCEL_STEP;
    float ramp_decel = SPEED_CMD_DECEL_STEP;

    float delta = desired_speed - s_speed_target;
    if (delta > 0.0f) {
        float step = ((s_speed_target < 0.0f) && (desired_speed > 0.0f))
                     ? SPEED_CMD_ZERO_STEP : ramp_accel;
        s_speed_target += (delta > step) ? step : delta;
    } else if (delta < 0.0f) {
        float step = ((s_speed_target > 0.0f) && (desired_speed < 0.0f))
                     ? SPEED_CMD_ZERO_STEP : ramp_decel;
        s_speed_target += (delta < -step) ? -step : delta;
    }

    /* 丢线找线驻留计时：只有 STATE_LOST 累加，跨状态跳转时清零交给上面
       的跳转分支负责。单位 10ms/读周期(g_speed_fresh 由 TIM6 每 10ms 置位) */
    if (current_state == STATE_LOST) {
        if (s_rec_ms < 0xFFFFU) s_rec_ms += 10U;
    }

    s_speed_cmd = s_speed_target;
    return s_speed_cmd;
}
void Balance_PID(float target)
{
    if (current_state == STATE_ESTOP) {
        Set_Motor_PWM(0.0f, 0.0f);
        return;
    }

    float steer_target = 10.0f;
	//循迹
    pid_closeloop_line(0.0f);   /* 先更新线位置，供本轮速度规划使用 */
    if (g_speed_fresh) {//同步g_speed_fresh在定时器编码器更新
        g_speed_fresh = 0;
        float speed_target = Line_SpeedUpdate();
        pid_closeloop_motor(speed_target, speed_target);
        /* 变化率限幅：积分在停止/堵转时攒到限幅，松手瞬间全量砸出会造成"顿"。
           限太紧(0.05°)会从刹车/扰动中恢复不过来，表现为偶发"停下"。
           加偏(>0)慢、回收(<0)快：回收慢会从刹车/扰动中恢复不过来，加偏慢是防前冲。
           倒车时反向：速度环把平衡点从"前倾"拉成"后倾"，属于回收侧，必须放得快，
           否则按 0.10°/拍 要 800ms 才挪到位，车早按原来的平衡点冲出去了 */
        const float slew_add = 0.10f;   /* 加偏侧(=15°/s) */
        const float slew_rel = 0.20f;   /* 回收侧(=25°/s) */
        float tilt_new = (g_tLeftMotorPID.out + g_tRightMotorPID.out) * 0.5f;//kp为正：车向前漂时error<0，tilt_offset<0（平衡点后倾）
        if (tilt_new - s_tilt_last >  slew_add) tilt_new = s_tilt_last + slew_add;
        if (tilt_new - s_tilt_last < -slew_rel) tilt_new = s_tilt_last - slew_rel;
        s_tilt_last = tilt_new;
        tilt_offset = tilt_new;
    }

	//直立
    pid_closeloop_angle(target + tilt_offset);//前漂时tilt_offset<0，目标角后移，车减速回正；必须为加号

    /* 转向输出 = 陀螺仪角速度环(抗扭) + 编码器差速纠偏(抗跑偏) */
    float enc_diff = (float)((int16_t)speed1 - (int16_t)speed2);
    //转向
    pid_closeloop_Steering(steer_target);
    

    /* 差速通道分配：循迹时归 line PID；丢线找线时归 Lost_SteerOut(朝丢线前
       线的方向缓转)。两者互斥，否则两路输出互相打架。
       此时前进速度保持，所以是"往线的位置走"，不是原地转向。 */
    float line_out = (current_state == STATE_TRACK) ? g_tLinePID.out : 0.0f;
    float lost_out = (current_state == STATE_LOST) ? Lost_SteerOut() : 0.0f;

     pwm_L = g_tAnglePID.out + g_tSteeringPID.out + line_out + lost_out - enc_diff * STEER_ENC_KP;
     pwm_R = g_tAnglePID.out - g_tSteeringPID.out - line_out - lost_out + enc_diff * STEER_ENC_KP;

    /* 5. final limit */
    if (pwm_L >  MOTOR_PWM_MAX) pwm_L =  MOTOR_PWM_MAX;
    if (pwm_L < -MOTOR_PWM_MAX) pwm_L = -MOTOR_PWM_MAX;
    if (pwm_R >  MOTOR_PWM_MAX) pwm_R =  MOTOR_PWM_MAX;
    if (pwm_R < -MOTOR_PWM_MAX) pwm_R = -MOTOR_PWM_MAX;

    /* 6. drive motors */
    Set_Motor_PWM(pwm_L, pwm_R);
}
void Motor_Task(void)
{
    static uint8_t s_last_key = 0;//保留Key_Flag按键的状态用于复位pid
#if AUTO_BALANCE_MID
    static uint16_t mid_cal_n = 0;
    static float mid_cal_sum = 0.0f;
    static uint16_t mid_cal_wait = 0;
#endif
    if (Key_Flag) {//启动按键按下
#if AUTO_BALANCE_MID
        if (mid_cal_n < MID_CAL_SAMPLES) {
            mid_cal_wait++;
            /* 车近似静止(|θ|小且角速度小)才采样；按键按早了、车在晃都自动跳过 */
            if ((fdata[0] > -MID_CAL_ANGLE_TH) && (fdata[0] < MID_CAL_ANGLE_TH) &&
                (gyro_dps[1] > -MID_CAL_GYRO_TH) && (gyro_dps[1] < MID_CAL_GYRO_TH)) {//俯仰角横滚角在建准范围
            mid_cal_n++;
            mid_cal_sum += fdata[0];
            Blance_Target = mid_cal_sum / mid_cal_n;   /* 递推平均，目标角平滑挪动 */
            uart2_send_string("AUTO_BALANCE_OK!");
            }
            /* 超时没采够：放弃本次校准，保留现有值(手动-2.7或已采部分) */
            if (mid_cal_wait >= MID_CAL_MAX_WAIT) {
                mid_cal_n = MID_CAL_SAMPLES;
            }
        }
#endif
        Balance_PID(Blance_Target);
    } else {
        /* 刚停车的瞬间复位各环：清积分/误差，防止下次启动带着残留积分蹿车 */
        if (s_last_key) {
            pid_reset(&g_tLeftMotorPID);
            pid_reset(&g_tRightMotorPID);
            pid_reset(&g_tAnglePID);
            pid_reset(&g_tSteeringPID);
            pid_reset(&g_tLinePID);
            s_speed_cmd = 0.0f;
            s_speed_target = 0.0f;
            s_prev_speed_line_pos = 0;
            s_slow_active = 0;
            current_state = STATE_STAND;     /* 重新启动从待机状态开始 */
            s_rec_ms = 0;
            s_line_was_seen = 0;             /* 新一次启动重新判定"是否上过线" */
            s_lost_dir = 0;
            s_lost_steer = 0.0f;
            s_tilt_last = 0.0f;
            tilt_offset = 0.0f;
            LineSensor_ResetState();
#if AUTO_BALANCE_MID
            mid_cal_n = 0;
            mid_cal_sum = 0.0f;
            mid_cal_wait = 0;
#endif
        }
        /* not enabled: coast both motors instead of holding last PWM */
        Motor_L_Write(0);
        Motor_R_Write(0);
    }

    s_last_key = Key_Flag;//将按下按键标志保存供复位用
}
