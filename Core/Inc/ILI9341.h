#ifndef __ILI9341_H__
#define __ILI9341_H__

#include "main.h" /* 提供 hspi1、GPIO 宏和 HAL API。 */
 HAL_StatusTypeDef ILI9341_Init(void);
 HAL_StatusTypeDef LCD_EndPixels(void);
HAL_StatusTypeDef ILI9341_FillScreen(uint16_t color);
//无DMA，spi阻塞发送，因为地址字节不多		
HAL_StatusTypeDef ILI9341_SetAddressWindow(uint16_t x0, /* 窗口左边界。 */
                                                  uint16_t y0, /* 窗口上边界。 */
                                                  uint16_t x1, /* 窗口右边界，包含该坐标。 */
                                                  uint16_t y1); /* 窗口下边界，包含该坐标。 */
												  
//无DMA，spi阻塞发送												  
HAL_StatusTypeDef ILI9341_FillRect(uint16_t x, /* 矩形左上角 X。 */
                                         uint16_t y, /* 矩形左上角 Y。 */
                                         uint16_t width, /* 矩形宽度。 */
                                         uint16_t height, /* 矩形高度。 */
                                         uint16_t color); /* RGB565 填充颜色。 */
#endif
