#include "ILI9341.h" 
#include "spi.h"
#include "lcd.h" 
static uint8_t lcd_color_block[128]; /* 静态缓冲区在函数返回前后都存在，并避免占用大块栈。 */
typedef struct { /* 定义一条 LCD 初始化命令。 */
    uint8_t command; /* ILI9341 命令字节。 */
    uint8_t length; /* 有效参数字节数。 */
    uint16_t delay_ms; /* 命令发送后需要等待的时间。 */
    uint8_t data[15]; /* 保存本示例最长 15 字节的命令参数。 */
} LCD_InitItem; /* 初始化表项类型定义结束。 */

static const LCD_InitItem ili9341_init_table[] = { /* 常见 ILI9341 模组初始化序列。 */
    {0xEFU, 3U, 0U, {0x03U, 0x80U, 0x02U}}, /* 厂商扩展控制。 */
    {0xCFU, 3U, 0U, {0x00U, 0xC1U, 0x30U}}, /* 电源控制 B。 */
    {0xEDU, 4U, 0U, {0x64U, 0x03U, 0x12U, 0x81U}}, /* 上电时序控制 A。 */
    {0xE8U, 3U, 0U, {0x85U, 0x00U, 0x78U}}, /* 驱动时序控制 A。 */
    {0xCBU, 5U, 0U, {0x39U, 0x2CU, 0x00U, 0x34U, 0x02U}}, /* 上电控制 A。 */
    {0xF7U, 1U, 0U, {0x20U}}, /* 泵比控制。 */
    {0xEAU, 2U, 0U, {0x00U, 0x00U}}, /* 驱动时序控制 B。 */
    {0xC0U, 1U, 0U, {0x23U}}, /* Power Control 1。 */
    {0xC1U, 1U, 0U, {0x10U}}, /* Power Control 2。 */
    {0xC5U, 2U, 0U, {0x3EU, 0x28U}}, /* VCOM Control 1。 */
    {0xC7U, 1U, 0U, {0x86U}}, /* VCOM Control 2。 */
    {0x36U, 1U, 0U, {0x48U}}, /* MADCTL：示例竖屏方向并启用 BGR。 */
    {0x3AU, 1U, 0U, {0x55U}}, /* COLMOD：选择 16 bit/pixel RGB565。 */
    {0xB1U, 2U, 0U, {0x00U, 0x18U}}, /* Normal Mode Frame Rate。 */
    {0xB6U, 3U, 0U, {0x08U, 0x82U, 0x27U}}, /* Display Function Control。 */
    {0xF2U, 1U, 0U, {0x00U}}, /* 禁用 3Gamma。 */
    {0x26U, 1U, 0U, {0x01U}}, /* 选择 Gamma 曲线 1。 */
    {0xE0U, 15U, 0U, {0x0FU, 0x31U, 0x2BU, 0x0CU, 0x0EU, 0x08U, 0x4EU, 0xF1U, 0x37U, 0x07U, 0x10U, 0x03U, 0x0EU, 0x09U, 0x00U}}, /* 正 Gamma。 */
    {0xE1U, 15U, 0U, {0x00U, 0x0EU, 0x14U, 0x03U, 0x11U, 0x07U, 0x31U, 0xC1U, 0x48U, 0x08U, 0x0FU, 0x0CU, 0x31U, 0x36U, 0x0FU}}, /* 负 Gamma。 */
    {0x11U, 0U, 120U, {0U}}, /* SLPOUT：退出睡眠后至少等待约 120 ms。 */
    {0x29U, 0U, 20U, {0U}} /* DISPON：打开显示并等待面板稳定。 */
}; /* 初始化表结束。 */

static void ILI9341_HardwareReset(void) /* 按控制器要求产生硬件复位脉冲。 */
{ /* LCD 硬复位函数体开始。 */
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); /* 复位期间保持 LCD 未选中。 */
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET); /* 先建立确定的高电平。 */
    HAL_Delay(5U); /* 等待电源和 GPIO 电平稳定。 */
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET); /* 拉低 RESET 进入硬复位。 */
    HAL_Delay(20U); /* 保持低电平，满足模块复位脉宽并留裕量。 */
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET); /* 释放硬复位。 */
    HAL_Delay(120U); /* 等待控制器内部电源和寄存器恢复。 */
} /* LCD 硬复位函数体结束。 */



static HAL_StatusTypeDef LCD_WriteRegister(uint8_t command, /* 要写入 LCD 的命令字节。 */
                                           const uint8_t *data, /* 命令参数数组，允许在 length=0 时为 NULL。 */
                                           uint16_t length) /* 参数字节数。 */
{ /* LCD 寄存器写入函数体开始。 */
    HAL_StatusTypeDef status; /* 保存 HAL 发送或等待结果。 */
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); /* 拉低 CS，选择 LCD。 */
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET); /* DC=0，下一字节解释为命令。 */
    status = HAL_SPI_Transmit(&hspi1, &command, 1U, 100U); /* 阻塞发送一个命令字节。 */
    if ((status == HAL_OK) && (length > 0U)) { /* 命令成功且确实存在参数时继续。 */
        HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET); /* DC=1，后续字节解释为参数。 */
        status = HAL_SPI_Transmit(&hspi1, (uint8_t *)data, length, 100U); /* 发送参数，显式适配 HAL 的非 const 原型。 */
    } /* 完成参数发送判断。 */
    if (status == HAL_OK) { /* 只有发送 API 成功时才继续等待总线结束。 */
        status = LCD_WaitNotBusy(10U); /* 等待最后一位真正离开 SPI 引脚。 */
    } /* 完成 BSY 等待条件判断。 */
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); /* 无论成功或失败都释放 CS。 */
    return status; /* 返回成功、超时或 HAL 错误。 */
} /* LCD 寄存器写入函数体结束。 */

 HAL_StatusTypeDef ILI9341_Init(void) /* 执行硬复位和表驱动初始化。 */
{ /* ILI9341 初始化函数体开始。 */
    ILI9341_HardwareReset(); /* 先让控制器回到确定状态。 */
    for (uint16_t i = 0U; i < (uint16_t)(sizeof(ili9341_init_table) / sizeof(ili9341_init_table[0])); ++i) { /* 顺序执行全部命令。 */
        const LCD_InitItem *item = &ili9341_init_table[i]; /* 取得当前初始化表项。 */
        HAL_StatusTypeDef status = LCD_WriteRegister(item->command, item->data, item->length); /* 发送命令及参数。 */
        if (status != HAL_OK) { /* 任意传输失败都不能继续假装初始化成功。 */
            return status; /* 把失败状态交给 main 或故障处理逻辑。 */
        } /* 完成发送结果检查。 */
        if (item->delay_ms > 0U) { /* 仅对要求等待的命令执行延时。 */
            HAL_Delay(item->delay_ms); /* 等待控制器内部操作完成。 */
        } /* 完成延时判断。 */
    } /* 完成初始化表遍历。 */
    return HAL_OK; /* 所有命令均成功发送。 */
} /* ILI9341 初始化函数体结束。 */

static HAL_StatusTypeDef LCD_BeginPixels(void) /* 发送 RAMWR 并保持 CS 为低，准备连续像素流。 */
{ /* 像素流开始函数体开始。 */
    uint8_t command = 0x2CU; /* ILI9341 Memory Write 命令。 */
    HAL_StatusTypeDef status; /* 保存命令发送和空闲等待结果。 */
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); /* 选择 LCD 并开始像素事务。 */
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET); /* DC=0，先发送 RAMWR 命令。 */
    status = HAL_SPI_Transmit(&hspi1, &command, 1U, 100U); /* 阻塞发送一个 RAMWR 字节。 */
    if (status == HAL_OK) { /* 命令成功后等待最后一位离开线路。 */
        status = LCD_WaitNotBusy(10U); /* 避免切换 DC 时命令仍在发送。 */
    } /* 完成命令结果检查。 */
    if (status == HAL_OK) { /* 只有总线空闲时才能进入像素数据阶段。 */
        HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET); /* DC=1，后续字节解释为 RGB565 像素。 */
    } else { /* 命令或等待失败时不能保持 LCD 被选中。 */
        HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); /* 释放 CS 结束错误事务。 */
    } /* 完成像素阶段切换。 */
    return status; /* 返回像素流是否成功开始。 */
} /* 像素流开始函数体结束。 */

 HAL_StatusTypeDef LCD_EndPixels(void) /* 等待像素流完成并释放 CS。 */
{ /* 像素流结束函数体开始。 */
    HAL_StatusTypeDef status = LCD_WaitNotBusy(10U); /* 确认最后一个像素字节已经发送。 */
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); /* 无论结果如何都结束当前 LCD 事务。 */
    return status; /* 返回总线结束状态。 */
} /* 像素流结束函数体结束。 */

/* 设置起始地址和借结束地址，同时启动像素流*/ 
 HAL_StatusTypeDef ILI9341_SetAddressWindow(uint16_t x0, /* 窗口左边界。 *//* 设置起始地址和借结束地址，同时启动像素流*/ 
                                                  uint16_t y0, /* 窗口上边界。 */
                                                  uint16_t x1, /* 窗口右边界，包含该坐标。 */
                                                  uint16_t y1) /* 窗口下边界，包含该坐标。 */
{ /* 地址窗口函数体开始。 */
    uint8_t column[4] = {(uint8_t)(x0 >> 8U), (uint8_t)x0, (uint8_t)(x1 >> 8U), (uint8_t)x1}; /* 按大端序打包列范围。 */
    uint8_t page[4] = {(uint8_t)(y0 >> 8U), (uint8_t)y0, (uint8_t)(y1 >> 8U), (uint8_t)y1}; /* 按大端序打包行范围。 */
    HAL_StatusTypeDef status = LCD_WriteRegister(0x2AU, column, 4U); /* 发送 CASET 列地址范围。 */
    if (status == HAL_OK) { /* 列范围成功后再发送行范围。 */
        status = LCD_WriteRegister(0x2BU, page, 4U); /* 发送 PASET 行地址范围。 */
    } /* 完成行范围发送条件判断。 */
    if (status == HAL_OK) { /* 坐标均设置成功后才开始写显存。 */
        status = LCD_BeginPixels(); /* 发送 RAMWR，并让 CS 保持低。 */
    } /* 完成像素流启动判断。 */
    return status; /* 返回窗口和像素流是否准备成功。 */
} /* 地址窗口函数体结束。 */

 HAL_StatusTypeDef ILI9341_FillRect(uint16_t x, /* 矩形左上角 X。 */
                                         uint16_t y, /* 矩形左上角 Y。 */
                                         uint16_t width, /* 矩形宽度。 */
                                         uint16_t height, /* 矩形高度。 */
                                         uint16_t color) /* RGB565 填充颜色。 */
{ /* 矩形填充函数体开始。 */
    uint32_t x_end; /* 使用 32 位计算防止 16 位加法回绕。 */
    uint32_t y_end; /* 使用 32 位计算防止 16 位加法回绕。 */
    uint32_t bytes_remaining; /* 保存尚未发送的像素字节数。 */
    HAL_StatusTypeDef status; /* 保存 SPI 操作结果。 */
    if ((width == 0U) || (height == 0U) || (x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) { /* 空矩形或起点越界无需绘制。 */
        return HAL_OK; /* 把不可见区域视为无操作成功。 */
    } /* 完成基本参数检查。 */
    x_end = (uint32_t)x + width; /* 计算不包含在内的 X 终点。 */
    y_end = (uint32_t)y + height; /* 计算不包含在内的 Y 终点。 */
    if (x_end > LCD_WIDTH) { /* 右侧超出屏幕时执行裁剪。 */
        x_end = LCD_WIDTH; /* 把终点限制到屏幕宽度。 */
    } /* 完成 X 裁剪。 */
    if (y_end > LCD_HEIGHT) { /* 下侧超出屏幕时执行裁剪。 */
        y_end = LCD_HEIGHT; /* 把终点限制到屏幕高度。 */
    } /* 完成 Y 裁剪。 */
    width = (uint16_t)(x_end - x); /* 得到裁剪后的实际宽度。 */
    height = (uint16_t)(y_end - y); /* 得到裁剪后的实际高度。 */
    for (uint16_t i = 0U; i < (uint16_t)sizeof(lcd_color_block); i += 2U) { /* 预生成 64 个相同像素。 */
        lcd_color_block[i] = (uint8_t)(color >> 8U); /* 每个像素先放高字节。 */
        lcd_color_block[i + 1U] = (uint8_t)(color & 0xFFU); /* 每个像素再放低字节。 */
    } /* 完成重复色块生成。 */
    status = ILI9341_SetAddressWindow(x, y, (uint16_t)(x_end - 1U), (uint16_t)(y_end - 1U)); /* 设置裁剪后的闭区间窗口。 */
    bytes_remaining = (uint32_t)width * height * 2U; /* 一个 RGB565 像素占两个字节。 */
    while ((status == HAL_OK) && (bytes_remaining > 0U)) { /* 分块发送直到矩形填满或发生错误。 */
        uint16_t chunk = bytes_remaining > sizeof(lcd_color_block) ? (uint16_t)sizeof(lcd_color_block) : (uint16_t)bytes_remaining; /* 选择当前块长度。 */
        status = HAL_SPI_Transmit(&hspi1, lcd_color_block, chunk, 100U); /* 阻塞发送当前重复颜色块。 */
        bytes_remaining -= chunk; /* 更新尚未发送的字节数。 */
    } /* 完成全部像素分块发送。 */
    if (LCD_EndPixels() != HAL_OK) { /* 始终结束事务，并检查最后一位发送状态。 */
        status = HAL_TIMEOUT; /* 用超时表示总线未正常结束。 */
    } /* 完成事务结束检查。 */
    return status; /* 返回填充结果。 */
} /* 矩形填充函数体结束。 */

HAL_StatusTypeDef ILI9341_FillScreen(uint16_t color) /* 用单一 RGB565 颜色填满全屏。 */
{ /* 全屏填充函数体开始。 */
    return ILI9341_FillRect(0U, 0U, LCD_WIDTH, LCD_HEIGHT, color); /* 复用经过裁剪和字节序验证的矩形函数。 */
} /* 全屏填充函数体结束。 */
