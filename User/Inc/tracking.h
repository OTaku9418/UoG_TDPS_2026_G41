#ifndef __TRACKING_H
#define __TRACKING_H

#include "main.h"

/*
 * IR sensor read macros - 4-way tracking:
 *   IR_X4 (Left-Out):   PC5
 *   IR_X3 (Left-In):    PB1
 *   IR_X2 (Right-In):   PE8
 *   IR_X1 (Right-Out):  PE10
 *
 * Return: Assuming 1 = line detected (black), 0 = ground (white)
 */
#define IR_X4_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)
#define IR_X3_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)
#define IR_X2_READ()  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8)
#define IR_X1_READ()  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)

/* API */
void Tracking_Init(void);
void Tracking_Pro_Run(void);
void Tracking_Pro_Reset(void);

#endif /* __TRACKING_H */
