#include "lora.h"
#include "usart.h"
#include "motor.h"
#include "oled.h"
#include <string.h>
#include <stdio.h>

static uint32_t tof_start_tick = 0;
HAL_StatusTypeDef rx_status;
uint8_t rx_buf[7] = {0};
static uint32_t last_tof_send_tick = 0;
static uint8_t is_first_tof_send = 1;

/**
 * @brief 初始化 E22-T LoRa 模块
 * @note 新模块已通过配置工具预先设置为定点传输模式，因此只需配置 M0, M1 引脚进入模式0（传输模式）
 */
void LoRa_Init(void)
{
    // E22-T模块在 M0=0, M1=0 时进入模式0 (传输模式)
    HAL_GPIO_WritePin(lora_m0_GPIO_Port, lora_m0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lora_m1_GPIO_Port, lora_m1_Pin, GPIO_PIN_RESET);
    
    // 延时等待模块状态切换并启动稳定
    HAL_Delay(100); 
}

/**
 * @brief 使用 E22-T LoRa 模块以定点传输模式发送信息
 * @param addr    目标设备地址（范围 0x0000 - 0xFFFF）
 * @param channel 目标信道
 * @param data    消息内容指针
 * @param len     消息内容长度
 */
void LoRa_Send_FixedPoint(uint16_t addr, uint8_t channel, uint8_t *data, uint16_t len)
{
    // E22-T 定点传输数据格式：目标地址高字节 + 目标地址低字节 + 目标信道 + 数据
    // E22-T 单次最大缓存为 240 字节，减去 3 个字节的头部，数据最大 237 字节
    uint8_t buffer[240];
    
    // 如果数据长度超过237，截断处理以防止溢出
    if (len > 237)
    {
        len = 237;
    }

    // 检查 E22-T 的 AUX 引脚状态，等待模块空闲
    // E22-T 模块正常工作时：低电平表示忙状态，高电平表示空闲状态
    while (HAL_GPIO_ReadPin(lora_aux_GPIO_Port, lora_aux_Pin) == GPIO_PIN_RESET)
    {
        // 阻塞等待模块空闲
    }

    // 拼装定点传输数据包
    buffer[0] = (addr >> 8) & 0xFF; // 目标地址高字节
    buffer[1] = addr & 0xFF;        // 目标地址低字节
    buffer[2] = channel;            // 目标信道
    
    // 拷贝消息内容
    if (len > 0 && data != NULL)
    {
        memcpy(&buffer[3], data, len);
    }

    // 通过 UART5 发送给 LoRa 模块
    HAL_UART_Transmit(&huart5, buffer, len + 3, HAL_MAX_DELAY);
}

/**
 * @brief 初始化TOF传感器，配置为高精度模式，并记录开机时间
 */
void TOF_Init(void)
{
    uint8_t dummy_buf[8];
	
    // TOF设置高精度模式指令: 01 06 00 04 00 01 09 CB
    uint8_t tof_high_precision_cmd[] = {0x01, 0x06, 0x00, 0x04, 0x00, 0x01, 0x09, 0xCB};
    HAL_UART_Transmit(&huart3, tof_high_precision_cmd, sizeof(tof_high_precision_cmd), HAL_MAX_DELAY);
    // 丢弃模块返回的应答
    HAL_UART_Receive(&huart3, dummy_buf, 8, 100);
    HAL_Delay(50);
    
    // 记录开机时刻（毫秒级）
    tof_start_tick = HAL_GetTick();
}

/**
 * @brief CRC-16/MODBUS 校验计算
 * @param data 待计算数据指针
 * @param len  数据长度
 * @return 16位CRC校验值
 */
static uint16_t CRC16_Modbus(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief TOF 偏移量校准函数
 * @note  使用方法：将障碍物放置在传感器正前方固定距离处（建议50mm），
 *        将该实际距离（单位mm）作为参数传入本函数，模块将自动计算并存储偏移量。
 *        校准成功后偏移量永久保存在模块内，断电不丢失，正常使用时无需重复校准。
 * @param actual_mm 障碍物到传感器的实际距离（单位：mm，建议使用50mm）
 */
void TOF_Calibrate(uint16_t actual_mm)
{
    uint8_t cmd[8];
    uint8_t resp[8] = {0};
    uint16_t crc;

    // 构建写寄存器 0x0020（偏移校准触发寄存器）的 Modbus RTU 指令
    cmd[0] = 0x01;                       // 从机地址
    cmd[1] = 0x06;                       // 功能码：写单个寄存器
    cmd[2] = 0x00;                       // 寄存器地址高字节
    cmd[3] = 0x20;                       // 寄存器地址低字节（0x0020 = 偏移校准寄存器）
    cmd[4] = (actual_mm >> 8) & 0xFF;    // 实际距离高字节
    cmd[5] = actual_mm & 0xFF;           // 实际距离低字节

    // 计算并附加 CRC 校验码
    crc = CRC16_Modbus(cmd, 6);
    cmd[6] = crc & 0xFF;                 // CRC 低字节
    cmd[7] = (crc >> 8) & 0xFF;         // CRC 高字节

    // 发送校准指令
    HAL_UART_Transmit(&huart3, cmd, 8, HAL_MAX_DELAY);

    // 等待模块应答（校准完成后会回显相同的指令）
    HAL_UART_Receive(&huart3, resp, 8, 500);

    // 等待模块完成内部校准写入
    HAL_Delay(200);
}



/**
 * @brief 读取TOF传感器数据，如果小于500mm则通过LoRa发送信息
 * @return 1如果成功发送了信息，0则未发送
 */
uint8_t TOF_CheckAndSend(void)
{
    // TOF读取测距值指令: 01 03 00 10 00 01 85 CF
    uint8_t read_cmd[] = {0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF};
    
    // 强行复位串口的状态机，防止由于之前异常或中断嵌套导致的永久性 HAL_BUSY
    huart3.RxState = HAL_UART_STATE_READY;
    __HAL_UNLOCK(&huart3);

    // 清除溢出错误(ORE)标志和接收寄存器，防止接收卡死
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_FLUSH_DRREGISTER(&huart3);
    
    // 发送读取指令
    HAL_UART_Transmit(&huart3, read_cmd, sizeof(read_cmd), HAL_MAX_DELAY);
    
    // 接收返回值
    rx_status = HAL_UART_Receive(&huart3, rx_buf, 7, 100);
    if (rx_status == HAL_OK)
    {
        // 校验头部是否正确 (从机地址0x01, 功能码0x03, 数据长度0x02)
        if (rx_buf[0] == 0x01 && rx_buf[1] == 0x03 && rx_buf[2] == 0x02)
        {
            // 解析距离数据 (单位: mm)
            uint16_t distance = (rx_buf[3] << 8) | rx_buf[4];
            
            if (distance < 500)
            {
                uint32_t current_tick = HAL_GetTick();

                // 检查是否为首次发送，或者距离上次发送已经超过 15 秒 (15000 毫秒)
                if (is_first_tof_send || (current_tick - last_tof_send_tick >= 15000))
                {
										
                    // 计算从初始化到现在的运行时间(秒)
                    uint32_t duration_sec = (current_tick - tof_start_tick) / 1000;
                    uint32_t mm = duration_sec / 60;
                    uint32_t ss = duration_sec % 60;
                    
                    char msg[64];
                    sprintf(msg, "Team41+Duration time:%02lu:%02lu", mm, ss);
                    
                    // 发送前在 OLED 第一行显示提示信息
                    OLED_ClearLine(0);
                    OLED_ShowString(0, 0, "LoRa Sending");

                    // 调用LoRa向0x0003地址，30信道发送信息
                    LoRa_Send_FixedPoint(0x0003, 0x1E, (uint8_t *)msg, strlen(msg));

                    // 停车3秒：设置目标速度为0，并直接输出0 PWM让底盘立刻停止
                    motor_speed_set(0, 0, 0, 0);
                    MOTOR_A_SetSpeed(0);
                    MOTOR_B_SetSpeed(0);
                    MOTOR_C_SetSpeed(0);
                    MOTOR_D_SetSpeed(0);
                    HAL_Delay(3000);

                    // 停车结束，清除 OLED 上的提示信息
                    OLED_ClearLine(0);

                    // 更新发送时间和状态标志位
                    last_tof_send_tick = HAL_GetTick();
                    is_first_tof_send = 0;
                    return 1;
                }
            }
        }
    }
    return 0;
}
