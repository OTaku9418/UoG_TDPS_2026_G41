#ifndef __LORA_H__
#define __LORA_H__

#include "main.h"
#include <stdint.h>

/**
 * @brief 使用LoRa模块以定点传输模式发送信息
 * @param addr    目标设备地址（范围 0x0000 - 0xFFFF）
 * @param channel 目标信道（范围 0x00 - 0x63, 对应 850.15MHz - 949.15MHz）
 * @param data    消息内容指针
 * @param len     消息内容长度（最大不超过227字节，因为分包最大230字节）
 */
void LoRa_Init(void);
void LoRa_Send_FixedPoint(uint16_t addr, uint8_t channel, uint8_t *data, uint16_t len);

void TOF_Init(void);
uint8_t TOF_CheckAndSend(void);

#endif /* __LORA_H__ */
