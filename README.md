# UoG_TDPS_2026
2026年TDPS小车自用代码，开源供参考学习

LoRa模块使用亿佰特E22-900T22D
雷达使用海凌科LD2410S (不推荐，建议换用频率更高的机器人专用雷达)
循迹为四路红外循迹
拱门检测使用ToF200F/ToF53200 (带上位MCU的ST VL53L0X测距传感器)
MCU主控板为嘉立创天空星高配版 STM32F407VGT6
屏幕为SSD1306驱动的0.96寸128*64 OLED屏幕

本工程使用定时器中断方式进行PID控制

ioc文件为CubeMX工程
epro2文件为配套PCB转接板工程

代码建议自行优化，不可直接复用

## 目录结构说明

```
TDPS_HAL_PID_IT
├── Core/        CubeMX 生成的外设初始化代码
├── Drivers/     STM32 官方 HAL 库与 CMSIS 启动代码
├── User/        用户业务代码（各模块驱动与控制逻辑）
├── MDK-ARM/     Keil MDK-ARM 工程
├── TDPS_HAL_PID_IT.ioc          CubeMX 工程文件
├── ProPrj_TDPS_Finished_2026-08-14.epro2  配套 PCB 转接板工程（嘉立创EDA专业版）
├── .mxproject   CubeMX 工程元数据
├── L1b_Design-Tasks_An-Overview_2025-2026.pdf 2026年TDPS任务要求
└── LICENSE      MIT 开源许可证
```

### Core/ — 外设初始化（CubeMX 生成）

`Inc` 存放外设头文件，`Src` 存放对应实现。修改 CubeMX 配置后重新生成代码时，该目录会被覆盖，自定义代码请放在 `User/` 中。

| 文件 | 作用 |
| ---- | ---- |
| `main.c` | 主程序入口：时钟配置、各外设初始化、主循环（当前运行循迹 `Tracking_Pro_Run()`） |
| `gpio.c/h` | GPIO 初始化（红外循迹读取引脚等） |
| `i2c.c/h` | I2C1 初始化（OLED 通信） |
| `usart.c/h` | UART4/UART5/USART2/USART3 初始化（LoRa、LD2410S、ToF 等串口设备） |
| `tim.c/h` | 定时器初始化：TIM1/TIM2/TIM3/TIM4/TIM8/TIM9/TIM12（电机 PWM、编码器计数） |
| `rtc.c/h` | RTC 实时时钟初始化 |
| `stm32f4xx_it.c` | 中断服务函数 |
| `stm32f4xx_hal_msp.c` | 各外设 MSP（引脚复用、时钟使能、DMA/中断配置） |
| `system_stm32f4xx.c` | 系统时钟启动文件 |
| `stm32f4xx_hal_conf.h` | HAL 库模块使能配置 |

### Drivers/ — 官方库文件

| 目录 | 作用 |
| ---- | ---- |
| `STM32F4xx_HAL_Driver/` | STM32F4 HAL 库驱动源码 |
| `CMSIS/` | Cortex-M4 内核支持与器件头文件、系统启动文件 |

### User/ — 用户业务代码

`Inc` 为头文件，`Src` 为源文件，均为自写模块。

| 模块 | 文件 | 作用 |
| ---- | ---- | ---- |
| 电机 | `motor.c/h` | 麦克纳姆轮底盘控制：四轮 PWM 输出、编码器速度反馈、增量式 PID 速度环、全向运动解算（`App_Motor_Run()`） |
| 循迹 | `tracking.c/h` | 四路红外循迹传感器读取与循迹流程控制（含巡线、拱门检测判断逻辑） |
| LoRa / ToF | `lora.c/h` | 亿佰特 E22-900T22D 定点传输（`LoRa_Send_FixedPoint()`），以及 ToF 测距传感器初始化与拱门距离检测上报（`TOF_Init/TOF_CheckAndSend/TOF_Calibrate`） |
| 雷达 | `ld2410s.c/h` | 海凌科 LD2410S 人体存在雷达串口解析与距离获取 |
| OLED | `oled.c/h` | 0.96 寸 I2C OLED 显示驱动（字符/字符串显示、雷达波形绘制） |
| 字库 | `oledfont.h` | OLED 显示用 ASCII 字模 |
| 移植说明 | `README_电机驱动移植说明.md` | 电机驱动板移植说明 |
| 备份 | `*.bkp` | 对应源文件的旧版备份，可忽略 |

### MDK-ARM/ — Keil 工程

编译产物（.o/.axf/.hex/.map 等，位于 `TDPS_HAL_PID_IT/`）与个人配置（`*.uvguix.*`/`*.uvoptx`）已加入 .gitignore，不随仓库发布。

| 文件/目录 | 作用 |
| ---- | ---- |
| `TDPS_HAL_PID_IT.uvprojx` | Keil MDK-ARM 工程文件（双击打开） |
| `TDPS_HAL_PID_IT.uvoptx/.uvguix` | 工程用户选项与界面布局（个人配置，可删） |
| `startup_stm32f407xx.s` | 汇编启动文件 |
| `RTE/` | Keil RTE 组件配置 |
| `DebugConfig/` | 调试器配置 |
| `TDPS_HAL_PID_IT/` | 编译输出目录（.o/.axf/.hex/.map 等中间产物） |
| `EventRecorderStub.scvd` | Event Recorder 调试组件 |

## 使用说明

1. 使用 STM32CubeMX 打开 `TDPS_HAL_PID_IT.ioc` 可查看引脚与外设配置；
2. 使用 Keil MDK-ARM 打开 `MDK-ARM/TDPS_HAL_PID_IT.uvprojx` 编译下载（默认生成 .hex）；
3. 转接板图纸见 `ProPrj_TDPS_Finished_2026-08-14.epro2`（嘉立创EDA专业版打开）。

