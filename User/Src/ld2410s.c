#include "ld2410s.h"
#include "usart.h"
#include "oled.h"
#include <stdio.h>

volatile uint16_t ld2410s_distance = 0;
uint8_t ld_rx_buf[1];
uint8_t ld_frame_buf[10];
uint8_t ld_frame_len = 0;

void LD2410S_Init(void)
{
    // 使能配置命令 (Enable configuration)
    uint8_t cmd_enable[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    HAL_UART_Transmit(&huart4, cmd_enable, sizeof(cmd_enable), 100);
    HAL_Delay(50);
    
    // 设置雷达上报数据频率为1Hz (Set frequency to 1Hz)
    // 0x0070命令字, 02 00 (状态频率), 0A 00 00 00 (1Hz), 0C 00 (距离频率), 0A 00 00 00 (1Hz)
    uint8_t cmd_freq[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x0E, 0x00, 0x70, 0x00, 
                          0x02, 0x00, 0x0A, 0x00, 0x00, 0x00, 
                          0x0C, 0x00, 0x0A, 0x00, 0x00, 0x00, 
                          0x04, 0x03, 0x02, 0x01};
    HAL_UART_Transmit(&huart4, cmd_freq, sizeof(cmd_freq), 100);
    HAL_Delay(50);
    
    // 切换为极简数据上报模式 (Switch to minimalist reporting mode)
    uint8_t cmd_minimalist[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x7A, 0x00, 
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                0x04, 0x03, 0x02, 0x01};
    HAL_UART_Transmit(&huart4, cmd_minimalist, sizeof(cmd_minimalist), 100);
    HAL_Delay(50);
    
    // 结束配置命令 (Disable configuration)
    uint8_t cmd_disable[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    HAL_UART_Transmit(&huart4, cmd_disable, sizeof(cmd_disable), 100);
    HAL_Delay(50);
    
    // 清除初始化期间由于未及时读取ACK而产生的溢出错误(ORE)标志
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    // 清空可能残留在数据寄存器中的无效字节
    volatile uint32_t tmpreg = huart4.Instance->DR;
    (void)tmpreg;
}

void LD2410S_SoftRestart(void)
{
    // 使能配置命令 (进入配置模式，打断雷达当前状态)
    uint8_t cmd_enable[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    HAL_UART_Transmit(&huart4, cmd_enable, sizeof(cmd_enable), 100);
    HAL_Delay(50);
    
    // 结束配置命令 (退出配置模式，让雷达基于当前环境重新开始检测)
    uint8_t cmd_disable[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    HAL_UART_Transmit(&huart4, cmd_disable, sizeof(cmd_disable), 100);
    HAL_Delay(50);
}
