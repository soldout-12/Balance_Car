#ifndef __LCD_H__
#define __LCD_H__

#include "main.h" /* 提供 hspi1、GPIO 宏和 HAL API。 */
#include <stdbool.h> /* 提供 bool 类型。 */
#include <stdint.h> /* 提供固定宽度整数类型。 */

#define LCD_IMAGE 1
#define LCD_TEST  0
#define LCD_WIDTH 240U /* 竖屏逻辑宽度。 */
#define LCD_HEIGHT 320U /* 竖屏逻辑高度。 */
#define LCD_COLOR_BLACK 0x0000U /* RGB565 黑色。 */
#define LCD_COLOR_WHITE 0xFFFFU /* RGB565 白色。 */
#define LCD_COLOR_RED 0xF800U /* RGB565 红色。 */
#define LCD_COLOR_GREEN 0x07E0U /* RGB565 绿色。 */
#define LCD_COLOR_BLUE 0x001FU /* RGB565 蓝色。 */
extern volatile uint16_t Second;
extern SPI_HandleTypeDef hspi1; /* 引用 CubeMX 在 spi.c 中创建的 SPI1 句柄。 */
void LCD_FillOrFail(uint16_t color);
HAL_StatusTypeDef ILI9341_Init(void);
HAL_StatusTypeDef LCD_WaitNotBusy(uint32_t timeout_ms);
void LCD_Poll(void); /* 在主循环调用，完成 DMA 事务收尾。 */
bool LCD_DmaIsBusy(void); /* 查询 SPI 是否仍被 DMA 占用，阻塞式渲染前需检查。 */
HAL_StatusTypeDef LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *pic);
HAL_StatusTypeDef LCD_FillRectAsync(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void LCD_Task(void);
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);
void LCD_ShowIntNum(uint16_t x,uint16_t y,int32_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);
void LCD_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);
 void LCD_ShowString(uint16_t x, uint16_t y, const char *str, uint16_t fc, uint16_t bc,uint16_t len);

#endif 
