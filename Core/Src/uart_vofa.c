/*******************************************************************************
 * @file        VOFA+
 * @author      eternal_fu
 * @version     V0.0.1
 * @brief       
 * @Date 2024-12-11 01:05:50
 * @LastEditTime 2024-12-11 04:28:17
 * @**************************************************************************** 
 * @attention   
 * @    ��                            
 *      0.0.1       20241211      fu      FireWater��
 *******************************************************************************/
#include "uart_vofa.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // 
#include "pid_demo.h"
#include "motor.h"
#include "tracking.h"
#include "usart.h"
//通道数组
float fdata[CH_COUNT];
//帧尾
unsigned char tail[4]={0x00, 0x00, 0x80, 0x7f};

uint8_t tx_buf[VOFA_TX_BUF_SIZE];
/* command map: add one line here when a new tunable parameter is needed */
CmdMap_t cmd_table[] = {
	//直立环
    {"PID_Upright_Kp", &g_tAnglePID.kp},
    {"PID_Upright_Kd", &g_tAnglePID.kd},
	//速度环
    {"L_PID_Speed_Kp",   &g_tLeftMotorPID.kp},  /* currently syncs left wheel only, see note below */
    {"L_PID_Speed_Ki",   &g_tLeftMotorPID.ki},
	{"L_PID_Speed_limit_i",   &g_tLeftMotorPID.integral_limit},
	//{"L_PID_Speed_limit_p",   &g_tLeftMotorPID.limit},
	{"R_PID_Speed_Kp",   &g_tRightMotorPID.kp},  /* currently syncs left wheel only, see note below */
    {"R_PID_Speed_Ki",   &g_tRightMotorPID.ki},
	{"R_PID_Speed_limit_i",   &g_tRightMotorPID.integral_limit},
	//{"R_PID_Speed_limit_p",   &g_tRightMotorPID.limit},
	//转向环
    {"PID_Turn_Kp",    &g_tSteeringPID.kp},
    //循迹环
    {"LinePID_Kp",    &g_tLinePID.kp},
    //前进速度
    {"Speed",    &Line_Follow_Speed},
    //机械中值
    {"Blance_Target",    &Blance_Target},
};

#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(cmd_table[0]))

/* entry count as a real object, so other files can loop the table
   (sizeof does not work on the extern array declared in the header) */
const uint8_t cmd_table_count = CMD_TABLE_SIZE;
/*串口解析函数，返回消耗的字节数，决定数据是等待拼接还是直接结束，也就是tail是否更新
若发送数据如kp=100不加换行函数返回0表示继续接收后拼接数据，环形缓冲区的tail不改变，RingBuffer_GetCount的返回值会累积
若发送数据带换行如kp=100\n，函数返回字节数表示一帧发完，环形缓冲区的tail由RingBuffer_ReadSkip函数改变。RingBuffer_GetCount的返回值不会累积
*/
uint16_t Command_Parser(uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        if (buf[i] == '\n') {
            buf[i] = '\0'; // 截断换行符
            
            char cmd_name[32]; // 存放提取出来的命令名
            float temp_val;
            
            // 提取 "等号前面的字符串" 和 "等号后面的浮点数"
            // %31[^=] 意思是：最多读31个字符，直到遇到 '=' 停止
            if (sscanf((char *)buf, "%31[^=]=%f", cmd_name, &temp_val) == 2) {        
                // 遍历映射表，寻找匹配的命令
                for (uint8_t j = 0; j < CMD_TABLE_SIZE; j++) {
                    if (strcmp(cmd_name, cmd_table[j].cmd_name) == 0) {
                        *(cmd_table[j].target_ptr) = temp_val; // 通过指针直接修改目标变量
                        break; // 找到就退出循环，不用再往后找了
                    }
                }
            }
            return i + 1; // 消耗掉这一帧（包含换行符）
        }
    }
    return 0; // 没找到换行符，等下一次数据
}
//串口解析函数
void VOFA_T_Just_Float(void){
    if(tx_busy)return;
//  拼接成 20 字节的发送缓冲区
	uint8_t charnum=CH_COUNT*4;//所有通道组成多少字节
	uint8_t total_charnum=charnum+4;//加上帧尾总共字节
	VOFA_Fill();
    memcpy(tx_buf, fdata, charnum);       // 4个float共16字节
    memcpy(&tx_buf[charnum], tail, 4);   // 拼接4字节帧尾
	//  通过串口发送
    tx_busy=1;
    if (HAL_UART_Transmit_IT(&huart2, tx_buf, total_charnum) != HAL_OK) {
        tx_busy = 0; /* 启动失败立刻放锁，否则后续 ASCII/VOFA 全被永久挡住 */
    }
}

static const char *VOFA_StateName(void)
{
    switch (current_state) {
    case STATE_STAND:
        return "STATE_STAND";
    case STATE_TRACK:
        return "STATE_TRACK";
    case STATE_LOST:
        return "STATE_LOST";
    case STATE_ESTOP:
        return "STATE_ESTOP";
    default:
        return "STATE_UNKNOWN";
    }
}

void VOFA_T_Fire_Water(void){
    if(tx_busy)return;

    static const float scale = 1000.0f;
    static const float max_value = 1000000.0f;
    char *p = (char *)tx_buf;
    const char *state_name = VOFA_StateName();
    uint8_t channel;

    VOFA_Fill();

    /* FireWater 允许 "<any>:ch0,ch1,...\n"，状态名放在前缀中，
       不占用数值通道，也不会破坏 CSV 数值解析。 */
    while (*state_name != '\0') {
        *p++ = *state_name++;
    }
    *p++ = ':';

    for (channel = 0U; channel < CH_COUNT; channel++) {
        float value = fdata[channel];
        uint32_t scaled;
        uint32_t whole;
        uint32_t fraction;
        uint32_t divisor;
        uint8_t negative = 0U;

        if (channel > 0U) {
            *p++ = ',';
        }

        /* NaN 输出 0；正负无穷和超大值限幅，避免浮点转整数未定义。 */
        if (value != value) {
            value = 0.0f;
        }
        if (value > max_value) {
            value = max_value;
        } else if (value < -max_value) {
            value = -max_value;
        }

        if (value < 0.0f) {
            negative = 1U;
            value = -value;
        }

        scaled = (uint32_t)(value * scale + 0.5f);
        if ((negative != 0U) && (scaled != 0U)) {
            *p++ = '-';
        }
        whole = scaled / 1000U;
        fraction = scaled % 1000U;

        if (whole == 0U) {
            *p++ = '0';
        } else {
            divisor = 1000000000U;
            while ((whole / divisor) == 0U) {
                divisor /= 10U;
            }
            while (divisor > 0U) {
                *p++ = (char)('0' + ((whole / divisor) % 10U));
                divisor /= 10U;
            }
        }

        *p++ = '.';
        *p++ = (char)('0' + ((fraction / 100U) % 10U));
        *p++ = (char)('0' + ((fraction / 10U) % 10U));
        *p++ = (char)('0' + (fraction % 10U));
    }

    *p++ = '\r';
    *p++ = '\n';

    tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart2, tx_buf, (uint16_t)(p - (char *)tx_buf)) != HAL_OK) {
        tx_busy = 0;
    }
}
void VOFA_Fill(void){
    fdata[0] = tilt_offset;/* VOFA通道1：速度环输出的平衡点偏移 */
    fdata[1]=s_line_pos;/* VOFA通道2：循迹位置 -7~+7 */
    fdata[2] = (float)LineSensor_IsLost(); /* VOFA通道3：1=丢线 */
    fdata[3] = s_speed_cmd;                  /* VOFA通道4：速度规划输出 */
    fdata[4] = pwm_L;                  /* VOFA通道4：速度规划输出 */
    fdata[5] = pwm_R;                  /* VOFA通道4：速度规划输出 */
}



