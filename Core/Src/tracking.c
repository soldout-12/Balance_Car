#include "tracking.h"
#include "iic.h"
/* 循迹传感器（GW_GRAY）挂第二路软件 IIC：PD3(SDA)/PD4(SCL)，函数为 IIC2_ 前缀。
   返回值语义与 IIC 一致：0=成功，非 0=失败。 */
#include "gw_grayscale_sensor.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
unsigned char Digtal;
unsigned char Anolog[8]={0};
unsigned char rx_buff[256]={0};
unsigned char Normal[8]={0};
uint8_t tracking_online = 0;    /* 循迹传感器在线标志：1=Ping通过，0=超时(未接线/IIC2不通) */
 unsigned char IIC_Anolog_Normalize(uint8_t Normalize_channel);
/* 前置声明：定义在本文件下方 */
unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len);
int32_t sum = 0;
/* ———————————— 供控制环调用的轻量循迹接口 ————————————
   与下面的 Tracking_Task（1秒级串口调试任务）完全独立：
   - 归一化在 Tracking_Task_Init 里常开，这里每 15ms 只读 8 字节；
   - 调用方是 200Hz 的 PID 中断路径，内部抽稀避免软件IIC拖垮控制时序。 */

#define LINE_READ_DIVIDE   3      /* 每3次调用(15ms)真读一次，其余用上次值 */
/* 判线改用"最黑通道黑度"，不再用8路黑度求和。原因(实测日志)：
   归一化白底在 105~255 之间漂，白底单通道仍有 75~150 的"黑度"，
   8路背景累加可达 500+，与真黑线(≈255)混在一起，sum 根本分不清——
   单通道压线时 sum 恰卡在丢线/找回两个门槛之间，状态机 50ms 级横跳。
   最黑通道不受背景总量影响：白底时它 <=150，压线时 ≈255，裕量极大。 */
#define LINE_BLACK_MIN     170U   /* 单通道黑度(255-norm)达到此值才算真黑线；
                                     低于此值视为白底背景，清零不参与加权/判线。
                                     实测白底黑度最高约150，真黑线≈255 */
#define LINE_LOST_DECAY_NUM 7     /* 丢线记忆每读周期乘 7/8：约7周期(105ms)衰减到0。
                                     调小(如5/8)记忆更短更保守，调大(如15/16)方向保持更久 */

 int16_t s_line_pos = 0;    /* 当前使用的位置，-7~+7 */
static uint8_t s_line_lost = 1;      /* 1=丢线搜线中，motor.c 据此减速 */
static uint16_t s_lost_cycles = 0;   /* 连续丢线的读周期数(每周期15ms)，供恢复状态机判超时 */

/*******************************************************************************
 * @brief   循迹位置解算：归一化模拟值里只挑"真黑"通道做加权平均
 * @return  -7~+7：线相对车中心的位置；>0=线在右侧(需右转)，<0=在左侧
 * @note    首调先验证极性：手持传感器让黑线在右，返回值应为正；若为负说明
 *          通道顺序装反，把 kW[] 全部取反即可。
 *          判线用最黑通道、不用总黑度求和(白底背景会污染 sum，见 LINE_BLACK_MIN)。
 *          丢线(最黑通道不够黑/连续IIC失败)时保留最后一次位置并逐周期(15ms)衰减，
 *          让急弯滑出视野的那一刻车还在朝线消失的方向修，而不是突然回正。
 *******************************************************************************/
int16_t LineSensor_ReadPos(void)
{
    static uint8_t div_cnt = 0;
    static uint8_t iic_fail_cnt = 0;   /* 连续IIC读失败次数：单次失败不能当拍判丢线 */
    static const int8_t kW[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    uint8_t norm[8];
    int32_t  wsum = 0;
    uint8_t line_valid = 0;
    uint8_t i;

    if (!tracking_online) return 0;   /* 离线：不碰总线，保持平衡优先 */

    if (++div_cnt >= LINE_READ_DIVIDE) {
        div_cnt = 0;
        if (IIC_Get_Anolog(norm, 8) == 0) {
            uint8_t darkest = 255;

            iic_fail_cnt = 0;
            sum = 0; /* 每次真实采样重新累计，不能跨 15ms 周期累加 */

            /* 判线看最黑通道，不看总黑度：白底背景累加会把 sum 顶到假高，
               单通道压线时又卡在门槛边缘(详见 LINE_BLACK_MIN 注释) */
            for (i = 0; i < 8; i++) {
                if (norm[i] < darkest) darkest = norm[i];
            }
            line_valid = ((uint8_t)(255 - darkest) >= LINE_BLACK_MIN) ? 1U : 0U;

            if (line_valid) {
                for (i = 0; i < 8; i++) {
                    int32_t black = 255 - (int32_t)norm[i];
                    /* 白底背景清零：只让真黑线的通道参与加权，否则背景黑度
                       会把 pos 往中间拽，弯道算不出真实偏移 */
                    if (black < (int32_t)LINE_BLACK_MIN) black = 0;
                    sum  += black;
                    wsum += black * kW[i];
                }
                if (sum > 0) {
                    s_line_pos = (int16_t)(wsum / sum);
                }
            }
        } else {
            /* 软件IIC在定时器/主循环高频调用下偶发读失败。若当拍就判丢线，
               状态机每失败一次抖一下(实测 TRACK/LOST 50ms 级横跳)。
               连续 3 次失败才认定总线真断，单次失败沿用上次位置和状态。 */
            if (iic_fail_cnt < 3U) {
                iic_fail_cnt++;
                return s_line_pos;
            }
            line_valid = 0U;
        }

        if (line_valid) {
            s_line_lost = 0;
            s_lost_cycles = 0;
        } else {
            /* 丢线：位置不能直接归零。急弯时线会先从视野(±7≈7cm)里
               滑出去，归零等于在车头最该打方向的一刻松方向盘，车沿切线冲出赛道。
               保留最后一次方向并按 LINE_LOST_DECAY_NUM/8 逐周期衰减；
               车速由 motor.c 的丢线分支压到 0.5 倍，方向靠这段记忆继续修。 */
            if (s_lost_cycles < 0xFFFFU) s_lost_cycles++;
            s_line_pos = (int16_t)((s_line_pos * LINE_LOST_DECAY_NUM) / 8);
            s_line_lost = 1;
        }
    }

    return s_line_pos;
}

/* 1-->丢线 */
uint8_t LineSensor_IsLost(void)
{
    return (tracking_online && s_line_lost) ? 1U : 0U;//1-->丢线
}

/* 连续丢线多久了(单位：读周期，每周期 15ms)。恢复状态机用它判超时 */
uint16_t LineSensor_LostCycles(void)
{
    return s_lost_cycles;
}

/* 停车/重新启动时清掉丢线状态：
   - s_lost_cycles 归零，避免带着上次的计数直接进倒车恢复；
   - s_line_lost 强制置1、s_line_pos 归零。只清计数是不够的：若上次是在线停车，
     残留的 s_line_lost=0 会让重启后头一个读周期(15ms)误判"线还在"，
     此时若车其实没在赛道上，速度环会立刻给满速目标窜出去。
     置1后下一个读周期会按真实数据重新判定，在线则15ms内即恢复正常。 */
void LineSensor_ResetState(void)
{
    s_lost_cycles = 0;
    s_line_lost = 1;
    s_line_pos = 0;
}

 unsigned char Ping(void)
{
	unsigned char dat;
	IIC2_ReadRegs(GW_GRAY_ADDR_DEF<<1,GW_GRAY_PING,&dat,1);
	if(dat==GW_GRAY_PING_OK)
	{
			return 0;
	}	
	else return 1;
}
void Tracking_Task_Init()
{
sprintf((char *)rx_buff,"hello_world!\r\n");
	uart2_send_string((char *)rx_buff);
	memset(rx_buff,0,256);
	uint16_t retry = 0;
	while(Ping())
	{
		if (++retry >= 1000U) {   /* 约1~2秒应答不上就放弃：循迹是附加功能，绝不能堵死整车启动 */
			tracking_online = 0;
			sprintf((char *)rx_buff,"Ping timeout, tracking offline!\r\n");
			uart2_send_string((char *)rx_buff);
			return;
		}
		HAL_Delay(1);
		sprintf((char *)rx_buff,"Ping Faild Try Again!\r\n");
		uart2_send_string((char *)rx_buff);
		memset(rx_buff,0,256);
	}
	tracking_online = 1;//Pin通过
	sprintf((char *)rx_buff,"Ping Succseful!\r\n");
	uart2_send_string((char *)rx_buff);
	memset(rx_buff,0,256);

	/* 归一化常开：控制环每15ms直接读8字节即可。
	   若在循环里反复开关，一次就要 10ms+，200Hz 控制环耗不起。 */
	IIC_Anolog_Normalize(0xFF);
	HAL_Delay(10);
}
unsigned char IIC_Get_Digtal(void)
{
	unsigned char dat;
	IIC2_ReadRegs(GW_GRAY_ADDR_DEF<<1,GW_GRAY_DIGITAL_MODE,&dat,1);
	return dat;
}
unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len)
{
	if(IIC2_ReadRegs(GW_GRAY_ADDR_DEF<<1,GW_GRAY_ANALOG_BASE_,Result,len))return 1;
	else return 0;
}
unsigned char IIC_Get_Single_Anolog(unsigned char Channel)
{
	unsigned char dat;
	IIC2_ReadRegs(GW_GRAY_ADDR_DEF<<1,GW_GRAY_ANALOG(Channel),&dat,1);
	return dat;
}
unsigned char IIC_Anolog_Normalize(uint8_t Normalize_channel)
{
	return IIC2_WriteRegs(GW_GRAY_ADDR_DEF<<1,0xCF,&Normalize_channel,1);
}
unsigned short IIC_Get_Offset(void )
{
	unsigned char dat;
	IIC2_ReadRegs(GW_GRAY_ADDR_DEF<<1,Offset,&dat,1);
	return dat;
}
void Tracking_Task()
{
	Digtal=IIC_Get_Digtal();
	sprintf((char *)rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
	uart2_send_string((char *)rx_buff);
	memset(rx_buff,0,256);
	
	//获取传感器模拟量结果
	if(IIC_Get_Anolog(Anolog,8)==0){
	sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
	uart2_send_string((char *)rx_buff);
	memset(rx_buff,0,256);
	}
	
	//获取传感器归一化结果（归一化已在 Init 常开，直接读即可）

	if(IIC_Get_Anolog(Normal,8)==0){
	sprintf((char *)rx_buff,"Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
	uart2_send_string((char *)rx_buff);
	memset(rx_buff,0,256);
	}
	sprintf((char *)rx_buff,"LinePos %d\r\n",s_line_pos);
	uart2_send_string((char *)rx_buff);
	memset(rx_buff,0,256);
	//HAL_Delay(1000);
}
