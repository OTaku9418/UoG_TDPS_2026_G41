#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_ADDR 0x78 // OLED I2C 默认通信地址

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearLine(uint8_t y);
void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_ShowChar(uint8_t x, uint8_t y, char chr);
void OLED_ShowString(uint8_t x, uint8_t y, char *str);
void OLED_ShowMultiString(uint8_t x, uint8_t y, char *str);
void OLED_DrawRadarWave(uint16_t tick);

#endif
