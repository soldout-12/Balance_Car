
#include "spi.h"
#include "lcd.h" 
#include "pic.h"
#include "ILI9341.h" 
#include "lcdfont.h" 
#include "motor.h"
#include "key.h"
#include "mpu6050.h"
#include "usart.h"
#include "pid_demo.h"
#include "uart_vofa.h"
#include "motor.h"
#include "tracking.h"

#define LCD_DMA_BLOCK_SIZE 1024U /* 每次 DMA 发送的最大字节数，必须为偶数。 */

typedef struct { /* 保存一个异步填充事务的全部状态。 */
    volatile bool busy; /* true 表示 SPI1 和 LCD 事务仍被 DMA 占用。 */
    volatile bool error; /* true 表示 DMA 或 SPI 发送失败。 */
    volatile bool faulted; /* true 表示总线恢复失败，重新初始化前禁止新事务。 */
    volatile bool finalize_pending; /* true 表示主循环需要等待 BSY 并释放 CS。 */
    uint32_t bytes_remaining; /* 尚未提交给 DMA 的像素字节数。 */
    const uint8_t *source; /* 非 NULL 时 DMA 源为图片数据；NULL 时重复发送 lcd_dma_block。 */
} LCD_DmaState; /* DMA 状态类型定义结束。 */

static uint8_t lcd_dma_block[LCD_DMA_BLOCK_SIZE]; /* DMA 源缓冲必须为静态或全局对象。 */
static LCD_DmaState lcd_dma_state; /* 保存当前唯一 LCD DMA 事务。 */

 void LCD_FillOrFail(uint16_t color) /* 填充全屏并在 SPI/LCD 事务失败时停止测试。 */
{ /* 纯色测试辅助函数体开始。 */
    if (ILI9341_FillScreen(color) != HAL_OK) { /* 检查窗口、像素发送和 BSY 等待结果。 */
        Error_Handler(); /* 保存失败现场，避免继续发送掩盖首个错误。 */
    } /* 完成填充结果检查。 */
} /* 纯色测试辅助函数体结束。 */


static HAL_StatusTypeDef LCD_DmaStartNextBlock(void) /* 提交下一块 DMA 数据。 */
{ /* DMA 分块提交函数体开始。 */
    uint16_t chunk; /* 保存本次 DMA 字节数。 */
    const uint8_t *src; /* 保存本次 DMA 的源地址。 */
    if (lcd_dma_state.bytes_remaining == 0U) { /* 所有数据均已交给 DMA。 */
        return HAL_OK; /* 等待最后一次完成回调结束事务。 */
    } /* 完成剩余量检查。 */
    chunk = lcd_dma_state.bytes_remaining > LCD_DMA_BLOCK_SIZE ? LCD_DMA_BLOCK_SIZE : (uint16_t)lcd_dma_state.bytes_remaining; /* 选择当前块长度。 */
    src = lcd_dma_state.source; /* 图片事务直接使用图片数据指针。 */
    if (src != NULL) { /* 图片模式下源指针随分块前移。 */
        lcd_dma_state.source = src + chunk; /* 提前推进源指针，回调中继续使用。 */
    } else { /* 纯色模式下始终重复发送同一个静态色块。 */
        src = lcd_dma_block; /* 纯色事务的 DMA 源地址固定。 */
    } /* 完成源地址选择。 */
    lcd_dma_state.bytes_remaining -= chunk; /* 提交前登记剩余量，回调再决定是否继续。 */
    return HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)src, chunk); /* 异步启动 SPI1_TX DMA。 */
} /* DMA 分块提交函数体结束。 */


static HAL_StatusTypeDef ILI9341_FillRectDmaStart(uint16_t x, /* 矩形左上角 X。 */
                                                 uint16_t y, /* 矩形左上角 Y。 */
                                                 uint16_t width, /* 矩形宽度，本示例要求已在屏幕范围内。 */
                                                 uint16_t height, /* 矩形高度，本示例要求已在屏幕范围内。 */
                                                 uint16_t color) /* RGB565 填充颜色。 */
{ /* DMA 矩形启动函数体开始。 */
    HAL_StatusTypeDef status; /* 保存窗口设置和 DMA 启动结果。 */
    if (lcd_dma_state.busy) { /* 同一 SPI/LCD 事务不能重入。 */
        return HAL_BUSY; /* 让调用者稍后重试或排队。 */
    } /* 完成 busy 检查。 */
    if (lcd_dma_state.faulted) { /* 上一次错误尚未完成 SPI 和 LCD 恢复。 */
        return HAL_ERROR; /* 禁止在不确定的外设状态上开始新事务。 */
    } /* 完成锁存故障检查。 */
    if ((width == 0U) || (height == 0U) || ((uint32_t)x + width > LCD_WIDTH) || ((uint32_t)y + height > LCD_HEIGHT)) { /* 简化版要求矩形完整位于屏幕内。 */
        return HAL_ERROR; /* 非法参数交给调用者修正。 */
    } /* 完成矩形边界检查。 */
    lcd_dma_state.busy = true; /* 在任何 SPI/GPIO 操作前取得事务所有权。 */
    lcd_dma_state.error = false; /* 清除上一笔事务的错误。 */
    lcd_dma_state.finalize_pending = false; /* 清除上一笔事务的结束请求。 */
    lcd_dma_state.source = NULL; /* 纯色事务复用静态色块，不使用图片指针。 */


    for (uint16_t i = 0U; i < LCD_DMA_BLOCK_SIZE; i += 2U) { /* 生成重复颜色 DMA 块。 */
        lcd_dma_block[i] = (uint8_t)(color >> 8U); /* 像素高字节先发。 */
        lcd_dma_block[i + 1U] = (uint8_t)color; /* 像素低字节后发。 */
    } /* 完成 DMA 块生成。 */
    status = ILI9341_SetAddressWindow(x, y, (uint16_t)(x + width - 1U), (uint16_t)(y + height - 1U)); /* 发送窗口并保持 CS 低、DC 高。 */
    if (status != HAL_OK) { /* 窗口或 RAMWR 发送失败。 */
        lcd_dma_state.error = true; /* 保存窗口事务失败。 */
        lcd_dma_state.faulted = true; /* HAL/外设状态可能不确定，要求显式恢复。 */
        lcd_dma_state.busy = false; /* 窗口函数已释放 CS，可以归还事务所有权。 */
        return status; /* 不进入 DMA 状态。 */
    } /* 完成窗口设置检查。 */
    lcd_dma_state.bytes_remaining = (uint32_t)width * height * 2U; /* 记录整个矩形的像素字节数。 */
    status = LCD_DmaStartNextBlock(); /* 启动第一块异步传输。 */
    if (status != HAL_OK) { /* 首块 DMA 启动失败时必须释放事务。 */
        lcd_dma_state.error = true; /* 保留错误供诊断。 */
        lcd_dma_state.faulted = true; /* DMA/HAL 状态可能不确定，禁止直接重试。 */
        if (LCD_EndPixels() != HAL_OK) { /* 先等待 SPI 并结束已经开始的 LCD 像素事务。 */
            lcd_dma_state.faulted = true; /* 无法安全结束时锁存故障，禁止自动重试。 */
        } /* 完成错误事务结束检查。 */
        lcd_dma_state.busy = false; /* CS 已释放后再归还事务所有权。 */
    } /* 完成首块启动结果处理。 */
    return status; /* 返回异步操作是否成功启动。 */
} /* DMA 矩形启动函数体结束。 */


static HAL_StatusTypeDef ILI9341_DrawImageDmaStart(uint16_t x, /* 图片左上角 X。 */
                                                   uint16_t y, /* 图片左上角 Y。 */
                                                   uint16_t width, /* 图片宽度，要求在屏幕范围内。 */
                                                   uint16_t height, /* 图片高度，要求在屏幕范围内。 */
                                                   const uint8_t *pic) /* RGB565 图片数据，高字节在前。 */
{ /* DMA 图片启动函数体开始。 */
    HAL_StatusTypeDef status; /* 保存窗口设置和 DMA 启动结果。 */
    if ((pic == NULL) || (width == 0U) || (height == 0U)) { /* 空指针或空图片无法绘制。 */
        return HAL_ERROR; /* 参数错误交给调用者修正。 */
    } /* 完成图片参数检查。 */
    if ((uint32_t)x + width > LCD_WIDTH) { /* 右侧越界的图片不按裁剪处理，直接拒绝。 */
        return HAL_ERROR; /* 与填充函数保持一致，要求完整位于屏幕内。 */
    } /* 完成右边界检查。 */
    if ((uint32_t)y + height > LCD_HEIGHT) { /* 下侧越界同样拒绝。 */
        return HAL_ERROR; /* 避免发送不完整的像素流导致窗口错位。 */
    } /* 完成下边界检查。 */
    if (lcd_dma_state.busy) { /* 同一 SPI/LCD 事务不能重入。 */
        return HAL_BUSY; /* 让调用者稍后重试或排队。 */
    } /* 完成 busy 检查。 */
    if (lcd_dma_state.faulted) { /* 上一次错误尚未完成 SPI 和 LCD 恢复。 */
        return HAL_ERROR; /* 禁止在不确定的外设状态上开始新事务。 */
    } /* 完成锁存故障检查。 */
    lcd_dma_state.busy = true; /* 在任何 SPI/GPIO 操作前取得事务所有权。 */
    lcd_dma_state.error = false; /* 清除上一笔事务的错误。 */
    lcd_dma_state.finalize_pending = false; /* 清除上一笔事务的结束请求。 */
    lcd_dma_state.source = pic; /* 记录图片源指针，DMA 直接读 flash 中的像素数据。 */
    status = ILI9341_SetAddressWindow(x, y, (uint16_t)(x + width - 1U), (uint16_t)(y + height - 1U)); /* 发送窗口并保持 CS 低、DC 高。 */
    if (status != HAL_OK) { /* 窗口或 RAMWR 发送失败。 */
        lcd_dma_state.error = true; /* 保存窗口事务失败。 */
        lcd_dma_state.faulted = true; /* HAL/外设状态可能不确定，要求显式恢复。 */
        lcd_dma_state.busy = false; /* 窗口函数已释放 CS，可以归还事务所有权。 */
        return status; /* 不进入 DMA 状态。 */
    } /* 完成窗口设置检查。 */
    lcd_dma_state.bytes_remaining = (uint32_t)width * height * 2U; /* RGB565 一个像素占两个字节。 */
    status = LCD_DmaStartNextBlock(); /* 启动第一块异步传输。 */
    if (status != HAL_OK) { /* 首块 DMA 启动失败时必须释放事务。 */
        lcd_dma_state.error = true; /* 保留错误供诊断。 */
        lcd_dma_state.faulted = true; /* DMA/HAL 状态可能不确定，禁止直接重试。 */
        if (LCD_EndPixels() != HAL_OK) { /* 先等待 SPI 并结束已经开始的 LCD 像素事务。 */
            lcd_dma_state.faulted = true; /* 无法安全结束时锁存故障，禁止自动重试。 */
        } /* 完成错误事务结束检查。 */
        lcd_dma_state.busy = false; /* CS 已释放后再归还事务所有权。 */
    } /* 完成首块启动结果处理。 */
    return status; /* 返回异步操作是否成功启动。 */
} /* DMA 图片启动函数体结束。 */


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) /* HAL 在一块 SPI DMA 完成后调用。 */
{ /* SPI DMA 完成回调函数体开始。 */
    if (hspi != &hspi1 || !lcd_dma_state.busy) { /* 过滤其他 SPI 或非 LCD 事务。 */
        return; /* 不处理不属于当前 LCD DMA 的回调。 */
    } /* 完成回调来源检查。 */
    if (lcd_dma_state.bytes_remaining > 0U) { /* 仍有像素尚未提交。 */
        if (LCD_DmaStartNextBlock() != HAL_OK) { /* 在回调中继续下一块。 */
            lcd_dma_state.error = true; /* 记录后续块启动失败。 */
            lcd_dma_state.bytes_remaining = 0U; /* 停止继续提交。 */
            lcd_dma_state.finalize_pending = true; /* 通知主循环结束损坏的事务。 */
        } /* 完成下一块启动检查。 */
        return; /* 下一块运行中，当前回调结束。 */
    } /* 完成剩余量判断。 */
    lcd_dma_state.finalize_pending = true; /* 让主循环等待 BSY、释放 CS 并发布完成。 */
} /* SPI DMA 完成回调函数体结束。 */

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) /* HAL 在 SPI 或 DMA 错误时调用。 */
{ /* SPI 错误回调函数体开始。 */
    if (hspi == &hspi1 && lcd_dma_state.busy) { /* 只恢复当前 LCD 事务。 */
        lcd_dma_state.error = true; /* 保存错误状态供主循环诊断。 */
        lcd_dma_state.bytes_remaining = 0U; /* 禁止继续提交分块。 */
        lcd_dma_state.finalize_pending = true; /* 让主循环中止 SPI、释放 CS 并恢复总线。 */
    } /* 完成错误来源检查。 */
} /* SPI 错误回调函数体结束。 */

 HAL_StatusTypeDef LCD_WaitNotBusy(uint32_t timeout_ms) /* 等待 SPI 移位寄存器完成最后一位发送。 */
{ /* SPI 空闲等待函数体开始。 */
    uint32_t start = HAL_GetTick(); /* 记录开始等待的系统 tick。 */
    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET) { /* BSY 为 1 表示线路仍在传输。 */
        if ((HAL_GetTick() - start) >= timeout_ms) { /* 防止外设异常时永久死循环。 */
            return HAL_TIMEOUT; /* 把超时交给上层处理。 */
        } /* 完成超时判断。 */
    } /* BSY 清零后最后一个时钟脉冲已经结束。 */
    return HAL_OK; /* 返回 SPI 已空闲。 */
} /* SPI 空闲等待函数体结束。 */

bool LCD_DmaIsBusy(void) /* 查询 DMA 是否仍在占用 SPI 总线。 */
{ /* LCD DMA 占用查询函数体开始。 */
    return lcd_dma_state.busy; /* 供阻塞式渲染在发送前做互斥检查。 */
} /* LCD DMA 占用查询函数体结束。 */

void LCD_Poll(void) /* 在主循环中完成 DMA 事务收尾，避免在中断中阻塞等待。 */
{ /* LCD DMA 轮询函数体开始。 */
    if (lcd_dma_state.busy && lcd_dma_state.finalize_pending) { /* DMA 已结束但 LCD 事务尚未释放。 */
        if (lcd_dma_state.error) { /* 错误事务需要先停止并恢复 SPI 外设。 */
            HAL_StatusTypeDef abort_status = HAL_SPI_Abort(&hspi1); /* 在主循环中止残留的 IT/DMA/SPI 操作。 */
            HAL_StatusTypeDef idle_status = LCD_WaitNotBusy(10U); /* 检查硬件是否真正停止。 */
            if ((abort_status != HAL_OK) || (idle_status != HAL_OK)) { /* 任一恢复步骤失败都不能复用总线。 */
                lcd_dma_state.faulted = true; /* 锁存故障，要求显式重新初始化。 */
            } /* 完成 SPI 中止结果检查。 */
            HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); /* SPI 停止后释放被破坏的 LCD 事务。 */
        } else if (LCD_EndPixels() != HAL_OK) { /* 正常 DMA 完成后等待最后一位并释放 CS。 */
            lcd_dma_state.error = true; /* 记录 BSY 等待超时。 */
            lcd_dma_state.faulted = true; /* 无法确认总线结束，重新初始化前禁止发送。 */
        } /* 完成正常或错误收尾分支。 */
        lcd_dma_state.finalize_pending = false; /* 清除结束请求。 */
        lcd_dma_state.busy = false; /* 向应用层发布事务已经结束。 */
    } /* 没有待结束事务时立即返回。 */
} /* LCD DMA 轮询函数体结束。 */

static HAL_StatusTypeDef LCD_DmaRecover(void) /* 在主循环显式恢复锁存的 SPI/LCD 故障。 */
{ /* LCD DMA 恢复函数体开始。 */
    HAL_StatusTypeDef status; /* 保存 SPI 和 LCD 重新初始化结果。 */
    if (lcd_dma_state.busy) { /* 活动事务尚未由 LCD_Poll 收尾。 */
        return HAL_BUSY; /* 避免恢复过程与 DMA 并发。 */
    } /* 完成活动事务检查。 */
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); /* 确保重新初始化期间 LCD 未选中。 */
    status = HAL_SPI_DeInit(&hspi1); /* 清理 SPI 和 DMA 关联状态。 */
    if (status == HAL_OK) { /* 反初始化成功后重新应用 CubeMX 参数。 */
        status = HAL_SPI_Init(&hspi1); /* 恢复 SPI1 Mode 0 和分频配置。 */
    } /* 完成 SPI 重新初始化判断。 */
    if (status == HAL_OK) { /* SPI 恢复后重新建立 LCD 控制器状态。 */
        status = ILI9341_Init(); /* 硬复位并重新发送面板初始化表。 */
    } /* 完成 LCD 重新初始化判断。 */
    if (status == HAL_OK) { /* 全部恢复步骤成功后才解除故障锁存。 */
        lcd_dma_state.error = false; /* 清除上一事务错误。 */
        lcd_dma_state.faulted = false; /* 允许开始新的 LCD 事务。 */
    } else { /* 任一恢复步骤失败。 */
        lcd_dma_state.faulted = true; /* 保持锁存，防止继续破坏总线。 */
    } /* 完成恢复结果发布。 */
    return status; /* 让应用决定重试次数或进入永久故障模式。 */
} /* LCD DMA 恢复函数体结束。 */


/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
static HAL_StatusTypeDef LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color) /* 在指定坐标绘制一个 RGB565 像素。 */
{ /* 单像素绘制函数体开始。 */
    if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) { /* 屏幕外坐标不能发送给当前方向的窗口。 */
        return HAL_ERROR; /* 返回参数错误。 */
    } /* 完成坐标检查。 */
    return ILI9341_FillRect(x, y, 1U, 1U, color); /* 复用完整的窗口和像素流实现。 */
} /* 单像素绘制函数体结束。 */
/******************************************************************************
      函数说明：发像素
      入口数据：color 点的颜色
      返回值：  无
******************************************************************************/
static HAL_StatusTypeDef LCD_WritePixel(uint16_t color) /* 发送一个 RGB565 像素，字节序与填充函数一致。 */
{ /* 单像素发送函数体开始。 */
    uint8_t pixel[2]; /* 保存高字节先发的两个像素字节。 */
    pixel[0] = (uint8_t)(color >> 8U); /* 像素高字节先发，与 ILI9341 大端约定一致。 */
    pixel[1] = (uint8_t)color; /* 像素低字节后发。 */
    return HAL_SPI_Transmit(&hspi1, pixel, 2U, 100U); /* 阻塞发送一个像素。 */
} /* 单像素发送函数体结束。 */

/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if (delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
    HAL_StatusTypeDef status; /* 保存 HAL 发送或等待结果。 */
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //得到偏移后的值
	status=ILI9341_SetAddressWindow(x,y,x+sizex-1,y+sizey-1);  //设置光标位置 
    if(status==HAL_OK){
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
		else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
		else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
		else{LCD_EndPixels();return;}
		for(t=0;t<8;t++)
		{
			if(!mode)//非叠加模式
			{
				if(temp&(0x01<<t))
				status=LCD_WritePixel(fc);
				else status=LCD_WritePixel(bc);
				if(status!=HAL_OK){LCD_EndPixels();return;}
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//叠加模式
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}  
	LCD_EndPixels(); /* 字符像素流结束，释放 CS。 */
} 			  
}

/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowIntNum(uint16_t x,uint16_t y,int32_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	if(num<0) /* 负数：先画减号，再从下一位开始画数字。num 取正值。 */
	{
		LCD_ShowChar(x,y,'-',fc,bc,sizey,0);
		num=-num;
		x+=sizex;
	}
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 


/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp,sizex;
	int32_t num1;
	sizex=sizey/2;
	num1=num*100;
	if(num1<0) /* 负数：先画负号，再对绝对值取位。 */
	{
		LCD_ShowChar(x,y,'-',fc,bc,sizey,0);
		num1=-num1;
		x+=sizex;
	}
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  无
******************************************************************************/
HAL_StatusTypeDef LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pic)
{
    return ILI9341_DrawImageDmaStart(x, y, width, height, pic); /* 复用带 busy 检查和字节序约定的图片 DMA 函数。 */
}

/* 启动一次纯色矩形 DMA 填充，主循环调用 LCD_Poll 等待其结束。 */
HAL_StatusTypeDef LCD_FillRectAsync(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint16_t color)
{
    return ILI9341_FillRectDmaStart(x, y, width, height, color); /* 复用原有重复色块 DMA 路径。 */
}

/* print a 0-terminated string with the built-in 8x16 ascii font */
 void LCD_ShowString(uint16_t x, uint16_t y, const char *str, uint16_t fc, uint16_t bc,uint16_t len)
{
    while (*str) {
        LCD_ShowChar(x, y, (uint8_t)*str, fc, bc, len, 0);
        x += len/2;
        str++;
    }
}

void LCD_Task(void){
   
    static uint32_t last_tick = 0;
    /* DMA 收尾必须最早执行：CS 还低着时不能启动任何阻塞式 SPI 发送。 */
    LCD_Poll();
    #if LCD_IMAGE
    /* 图片异步发送，本循环不等它发完，下一轮 LCD_Poll 负责收尾。 */
    (void)LCD_ShowPicture(0, 40, 240, 240, gImage_2);
    #endif
    
    if (LCD_DmaIsBusy()) { /* 图片仍在发送：阻塞式文字会打断像素流。 */
        return; /* 等下一轮 LCD_Poll 收尾后再渲染文字。 */
    } /* 完成 DMA 占用检查。 */
     #if LCD_TEST
    // 判断是否距离上次刷新超过 50 毫秒 (即 20Hz)
    if (HAL_GetTick() - last_tick < 50) {
        return; // 时间没到，直接退出，不占用 CPU 时间！
    }

    last_tick = HAL_GetTick();
    /* refresh area: wheel speeds on top, PID params from cmd_table below */
    LCD_ShowString(0,  0, "Speed1:", LCD_COLOR_GREEN, LCD_COLOR_BLACK,16);
    LCD_ShowIntNum(72, 0, speed1, 6, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 16);
    LCD_ShowString(0, 20, "Speed2:", LCD_COLOR_GREEN, LCD_COLOR_BLACK,16);
    LCD_ShowIntNum(72, 20, speed2, 6, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 16);
    
    for (uint8_t i = 0; i < cmd_table_count; i++) {
        uint16_t y = 48 + (uint16_t)i * 20;
        LCD_ShowString(0, y, cmd_table[i].cmd_name, LCD_COLOR_GREEN, LCD_COLOR_BLACK,16);
        LCD_ShowFloatNum1(160, y, *(cmd_table[i].target_ptr), 6, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 16);
    } // 更新时间戳
    LCD_ShowString(0,  280, "pitch:", LCD_COLOR_GREEN, LCD_COLOR_BLACK,16);
    LCD_ShowFloatNum1(72, 280,sum , 6, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 16);
    LCD_ShowString(0,  300, "pos:", LCD_COLOR_GREEN, LCD_COLOR_BLACK,16);
    LCD_ShowFloatNum1(72, 300, s_line_pos, 6, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 16);
//    LCD_ShowString(0,  288, "tilt:", LCD_COLOR_GREEN, LCD_COLOR_BLACK,16);
//    LCD_ShowFloatNum1(72, 288, tilt_offset, 6, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 16);
#endif
}
