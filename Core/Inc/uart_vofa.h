#ifndef __UART_VOFA_H__
#define __UART_VOFA_H__
#include <main.h>
#include "pid_demo.h"

#define CH_COUNT 6                         //串口通道数量
#define VOFA_TX_BUF_SIZE 128U              /* FireWater 是 ASCII，缓冲必须大于 JustFloat */
typedef struct {
    const char *cmd_name;   /* command name, e.g. "PID_Upright_Kp" */
    float *target_ptr;      /* pointer to the target PID parameter */
} CmdMap_t;

/* cmd_table is DEFINED in uart_vofa.c; every .c that includes this
   header shares this single copy via extern */
extern CmdMap_t cmd_table[];
extern const uint8_t cmd_table_count;
extern float fdata[CH_COUNT];
extern unsigned char tail[4];
extern uint8_t tx_buf[VOFA_TX_BUF_SIZE];

uint16_t Command_Parser(uint8_t *buf, uint16_t len);
void VOFA_T_Just_Float(void);
void VOFA_T_Fire_Water(void);
void VOFA_Fill(void);
#endif

