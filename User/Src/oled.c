#include "oled.h"
#include "oledfont.h"
#include "i2c.h" // 引用 hi2c1
#include <math.h>
#include <stdlib.h>

// 向OLED写入命令
static void OLED_WriteCmd(uint8_t cmd) {
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
}

// 向OLED写入数据
static void OLED_WriteData(uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

// 设置光标坐标 (x: 0~127, y: 0~7)
void OLED_SetPos(uint8_t x, uint8_t y) {
    OLED_WriteCmd(0xB0 + y);
    OLED_WriteCmd(((x & 0xF0) >> 4) | 0x10);
    OLED_WriteCmd((x & 0x0F) | 0x01);
}

// 清除OLED全屏内容
void OLED_Clear(void) {
    uint8_t i, n;
    for (i = 0; i < 8; i++) {
        OLED_WriteCmd(0xB0 + i);
        OLED_WriteCmd(0x00);
        OLED_WriteCmd(0x10);
        for (n = 0; n < 128; n++) {
            OLED_WriteData(0x00);
        }
    }
}

// 清除特定的一行 (y: 0~7)
void OLED_ClearLine(uint8_t y) {
    uint8_t n;
    if (y > 7) return;
    OLED_WriteCmd(0xB0 + y);
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x10);
    for (n = 0; n < 128; n++) {
        OLED_WriteData(0x00);
    }
}

// OLED 初始化函数
void OLED_Init(void) {
    HAL_Delay(100); // 等待OLED上电稳定

    OLED_WriteCmd(0xAE); // 关闭显示
    OLED_WriteCmd(0x00); // 设置低列地址
    OLED_WriteCmd(0x10); // 设置高列地址
    OLED_WriteCmd(0x40); // 设置起始行地址
    OLED_WriteCmd(0x81); // 设置对比度控制寄存器
    OLED_WriteCmd(0xCF); // 设置SEG输出电流亮度
    OLED_WriteCmd(0xA1); // 设置SEG/列映射
    OLED_WriteCmd(0xC8); // 设置COM/行扫描方向
    OLED_WriteCmd(0xA6); // 正常显示
    OLED_WriteCmd(0xA8); // 设置多路复用比
    OLED_WriteCmd(0x3f); // 1/64 duty
    OLED_WriteCmd(0xD3); // 设置显示偏移
    OLED_WriteCmd(0x00); // 不偏移
    OLED_WriteCmd(0xD5); // 设置显示时钟分频比/振荡频率
    OLED_WriteCmd(0x80); // 设置时钟为 100 Frames/sec
    OLED_WriteCmd(0xD9); // 设置预充电周期
    OLED_WriteCmd(0xF1); // 设置预充电为15个时钟，放电为1个时钟
    OLED_WriteCmd(0xDA); // 设置com引脚硬件配置
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(0xDB); // 设置VCOM取消选择电平
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(0x20); // 设置页面寻址模式 (0x00/0x01/0x02)
    OLED_WriteCmd(0x02);
    OLED_WriteCmd(0x8D); // 电荷泵设置
    OLED_WriteCmd(0x14); // 启用电荷泵
    OLED_WriteCmd(0xA4); // 禁用全屏点亮
    OLED_WriteCmd(0xA6); // 禁用反显
    OLED_WriteCmd(0xAF); // 开启显示

    OLED_Clear(); // 初始清屏
}

// 显示单个字符
void OLED_ShowChar(uint8_t x, uint8_t y, char chr) {
    uint8_t c = 0, i = 0;
    c = chr - ' '; // 获取字符偏移量
    if (x > 128 - 6) { x = 0; y++; } // 若一行放不下，则自动换行
    OLED_SetPos(x, y);
    for (i = 0; i < 6; i++) {
        OLED_WriteData(F6x8[c][i]);
    }
}

// 显示单行字符串
void OLED_ShowString(uint8_t x, uint8_t y, char *str) {
    while (*str != '\0') {
        OLED_ShowChar(x, y, *str);
        x += 6;
        if (x > 120) {
            x = 0;
            y++;
        }
        str++;
    }
}

// 显示多行字符串 (支持 \n 换行)
void OLED_ShowMultiString(uint8_t x, uint8_t y, char *str) {
    uint8_t start_x = x;
    while (*str != '\0') {
        if (*str == '\n') {
            x = start_x;
            y++;
        } else {
            OLED_ShowChar(x, y, *str);
            x += 6;
            if (x > 120) { // 越界自动换行到起始X坐标
                x = start_x;
                y++;
            }
        }
        str++;
    }
}

// 绘制雷达扫描动画 (在第2~7页，也就是屏幕下方的 128x48 区域)
void OLED_DrawRadarWave(uint16_t tick) {
    uint8_t buf[6][128] = {0}; // 6页，每页128像素
    int cx = 64;
    int cy = 47; // 相对坐标的底部中心
    
    #define DRAW_PIXEL(px, py) do { \
        if ((px) >= 0 && (px) < 128 && (py) >= 0 && (py) < 48) \
            buf[(py)/8][px] |= (1 << ((py)%8)); \
    } while(0)
    
    // 1. 画三个同心半圆弧
    int radii[3] = {15, 30, 45};
    for (int i=0; i<3; i++) {
        int r = radii[i];
        int x = 0;
        int y = r;
        int d = 3 - 2*r;
        while (y >= x) {
            DRAW_PIXEL(cx + x, cy - y);
            DRAW_PIXEL(cx - x, cy - y);
            DRAW_PIXEL(cx + y, cy - x);
            DRAW_PIXEL(cx - y, cy - x);
            
            x++;
            if (d > 0) {
                y--;
                d = d + 4 * (x - y) + 10;
            } else {
                d = d + 4 * x + 6;
            }
        }
    }
    
    // 2. 画十字参考线
    for(int x=0; x<128; x++) DRAW_PIXEL(x, cy);
    for(int y=0; y<=cy; y++) DRAW_PIXEL(cx, y);
    
    // 3. 画扫描线
    // tick 是 20ms 一次。让雷达每 1.5 秒转一圈 (75 tick)
    float angle = (tick % 75) * 3.14159f / 37.5f; 
    int end_x = cx + (int)(47.0f * cosf(angle));
    int end_y = cy - (int)(47.0f * sinf(angle));
    
    // Bresenham 画线
    int dx = abs(end_x - cx), sx = cx < end_x ? 1 : -1;
    int dy = -abs(end_y - cy), sy = cy < end_y ? 1 : -1;
    int err = dx + dy, e2; 
    
    int lx = cx, ly = cy;
    while(1) {
        DRAW_PIXEL(lx, ly);
        if (lx == end_x && ly == end_y) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; lx += sx; }
        if (e2 <= dx) { err += dx; ly += sy; }
    }
    
    #undef DRAW_PIXEL
    
    // 4. 将 buf 写入 OLED
    for (int p = 0; p < 6; p++) {
        OLED_WriteCmd(0xB0 + 2 + p);
        OLED_WriteCmd(0x00);
        OLED_WriteCmd(0x10);
        for (int i = 0; i < 128; i++) {
            OLED_WriteData(buf[p][i]);
        }
    }
}
