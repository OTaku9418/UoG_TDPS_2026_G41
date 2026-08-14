# 电机驱动移植说明

## 1. 文件结构

```
TDPS_HAL\User\
├── Inc\
│   └── motor.h          # 电机驱动头文件（常量、结构体、API 声明）
└── Src\
    └── motor.c          # 电机驱动实现（PWM 启动、PID、编码器读取、应用层）
```

## 2. 引脚与定时器映射总表

| 电机 | 功能 | 定时器 | 通道 | 引脚 | 说明 |
|------|------|--------|------|------|------|
| **A** | PWM 正转 | TIM1 | CH1 | PE9 | 正向 PWM 输出 |
| **A** | PWM 反转 | TIM1 | CH2 | PE11 | 反向 PWM 输出 |
| **A** | 编码器 | TIM2 | CH1, CH2 | PA5, PA1 | 编码器反馈 |
| **B** | PWM 正转 | TIM1 | CH3 | PE13 | 正向 PWM 输出 |
| **B** | PWM 反转 | TIM1 | CH4 | PE14 | 反向 PWM 输出 |
| **B** | 编码器 | TIM3 | CH1, CH2 | PB4, PB5 | 编码器反馈 |
| **C** | PWM 正转 | TIM9 | CH1 | PE5 | 正向 PWM 输出 |
| **C** | PWM 反转 | TIM9 | CH2 | PE6 | 反向 PWM 输出 |
| **C** | 编码器 | TIM4 | CH1, CH2 | PD12, PD13 | 编码器反馈 |
| **D** | PWM 正转 | TIM12 | CH1 | PB14 | 正向 PWM 输出 |
| **D** | PWM 反转 | TIM12 | CH2 | PB15 | 反向 PWM 输出 |
| **D** | 编码器 | TIM8 | CH1, CH2 | PC6, PC7 | 编码器反馈 |

每个电机使用 **两路 PWM** 控制 H 桥（正转通道 + 反转通道），无需单独的 DIR 方向引脚。

---

## 3. CubeMX 中必须修改的定时器配置

当前 CubeMX 生成的默认值不能满足电机驱动需求，请按以下步骤修改：

### 3.1 TIM1（APB2 总线，定时器时钟 = 168 MHz）

| 参数 | 当前值 | 需改为 | 说明 |
|------|--------|--------|------|
| Prescaler | 168 | **3** | 168 MHz ÷ (3+1) = 42 MHz 计数频率 |
| Counter Mode | Up | Up（不变） | |
| Counter Period | 65535 | **2000** | PWM 分辨率 0–2000 |
| Auto-reload preload | Disable | Disable（不变） | |

> **计算结果**：PWM 频率 = 168 MHz ÷ ((3+1) × (2000+1)) = 168 MHz ÷ 8004 ≈ **21.0 kHz**
> （原始 C24 V5 工程为 18 kHz，21 kHz 在电机驱动可接受范围内）

### 3.2 TIM9（APB2 总线，定时器时钟 = 168 MHz）

| 参数 | 当前值 | 需改为 | 说明 |
|------|--------|--------|------|
| Prescaler | 0 | **3** | 同 TIM1 |
| Counter Period | 65535 | **2000** | 同 TIM1 |

### 3.3 TIM12（APB1 总线，定时器时钟 = 84 MHz）

| 参数 | 当前值 | 需改为 | 说明 |
|------|--------|--------|------|
| Prescaler | 0 | **1** | 84 MHz ÷ (1+1) = 42 MHz 计数频率 |
| Counter Period | 65535 | **2000** | 同 TIM1 |

> APB1 定时器时钟 = 2 × 42 MHz = 84 MHz。PSC=1 时 PWM = 84 MHz ÷ (2 × 2001) ≈ **21.0 kHz**，与 TIM1/TIM9 一致。

### 3.4 TIM8 编码器（APB2 总线）— 重要！

| 参数 | 当前值 | 需改为 | 说明 |
|------|--------|--------|------|
| Prescaler | **168** | **0** | 编码器不应分频，否则分辨率被除以 169 |
| Encoder Mode | **TI1** | **TI1 and TI2 (TI12)** | TI12 模式提供 4 倍频分辨率 |
| Counter Period | 65535 | 65535（不变） | |

> 编码器 PSC 必须为 0，否则每 169 个时钟周期才计数一次，编码器反馈将严重失真。

### 3.5 可选优化：为所有编码器添加输入滤波

| 编码器定时器 | 参数 | 建议值 |
|-------------|------|--------|
| TIM2, TIM3, TIM4, TIM8 | IC1 Filter / IC2 Filter | **10** |

> 输入滤波有助于消除编码器信号的毛刺噪声（原始 C24 V5 工程使用 Filter=10）。

### 3.6 操作步骤总结

1. 打开 `TDPS_HAL.ioc`
2. 左侧 **Timers** 分类中依次选择 TIM1 / TIM9 / TIM12 / TIM8
3. 按上表修改对应参数
4. **Project Manager → Code Generator** 确认勾选 "Keep User Code when re-generating"
5. 按 **GENERATE CODE** 重新生成代码
6. 生成的 `Core\Src\tim.c` 中 `MX_TIMx_Init()` 将包含正确的配置值

---

## 4. 将电机驱动代码加入 Keil 工程

### 4.1 添加源文件到工程

1. 打开 Keil 工程 `MDK-ARM\TDPS_HAL.uvprojx`
2. 在 Project 窗口右键 **Target 1** → **Add Group**，命名为 `User`
3. 右键 **User** 组 → **Add Existing Files to Group 'User'**
4. 添加以下文件：
   - `..\User\Src\motor.c`
5. 如果 Keil 询问是否添加 include path，选 **是**

### 4.2 添加头文件搜索路径

1. **Project → Options for Target 'Target 1' → C/C++ 标签页**
2. 在 **Include Paths** 中添加：
   ```
   ..\User\Inc
   ```

### 4.3 确认 HAL 定时器模块已启用

`Core\Inc\stm32f4xx_hal_conf.h` 中以下宏应为取消注释状态（CubeMX 默认已启用）：

```c
#define HAL_TIM_MODULE_ENABLED
```

---

## 5. 在 main.c 中调用电机驱动

### 5.1 添加头文件

在 `Core\Src\main.c` 的 `USER CODE BEGIN Includes` 区域添加：

```c
/* USER CODE BEGIN Includes */
#include "motor.h"
/* USER CODE END Includes */
```

### 5.2 初始化电机

在 `main()` 函数的 `USER CODE BEGIN 2` 区域添加：

```c
/* USER CODE BEGIN 2 */
App_Motor_Init();
/* USER CODE END 2 */
```

> `App_Motor_Init()` 会自动调用 `HAL_TIM_PWM_Start()` 和 `HAL_TIM_Encoder_Start()` 使能各定时器的 PWM 输出和编码器计数。定时器本身的寄存器配置已由 CubeMX 生成的 `MX_TIMx_Init()` 完成。

### 5.3 主循环中运行电机控制

在 `main()` 的 `USER CODE BEGIN 3` 区域添加：

```c
/* USER CODE BEGIN 3 */
Motor_Loop();
/* USER CODE END 3 */
```

> `Motor_Loop()` 内部使用 `HAL_GetTick()` 计时，每 20 ms 自动执行一次 PID 控制循环（50 Hz）。

### 5.4 完整 main.c 修改示例

```c
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE BEGIN Includes */
#include "motor.h"
/* USER CODE END Includes */

// ...（中间代码不变）...

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_UART4_Init();
  // ...（其他 MX_xxx_Init 不变）...
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  App_Motor_Init();
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    Motor_Loop();
    /* USER CODE END 3 */
  }
}
```

---

## 6. API 参考

### 6.1 初始化

```c
void App_Motor_Init(void);   // 初始化电机 PWM + 编码器（调用一次）
```

### 6.2 控制循环

```c
void Motor_Loop(void);       // 放入 while(1)，内部每 20ms 自动执行 PID
void App_Motor_Run(void);    // 手动执行一次 PID 控制（不检查时间间隔）
```

### 6.3 设置目标速度

```c
void Motor_Speed_Set(float A, float B, float C, float D);
// A, B, C, D — 四个电机的目标速度，单位 m/s
// 正值 = 前进方向，负值 = 后退方向
```

### 6.4 获取实时速度

```c
extern ROBOT_Wheel Wheel_A, Wheel_B, Wheel_C, Wheel_D;

// Wheel_A.RT  — 电机 A 实时速度 (m/s)
// Wheel_A.TG  — 电机 A 目标速度 (m/s)
// Wheel_A.PWM — 电机 A 当前 PWM 输出值
```

### 6.5 底层控制（直接 PWM 控制，绕过 PID）

```c
void MOTOR_A_SetSpeed(int16_t speed);  // 范围 -2000 ~ +2000
void MOTOR_B_SetSpeed(int16_t speed);
void MOTOR_C_SetSpeed(int16_t speed);
void MOTOR_D_SetSpeed(int16_t speed);
```

---

## 7. 关键参数说明

| 宏定义 | 值 | 说明 |
|--------|-----|------|
| `MOTOR_PWM_MAX` | 2000 | PWM 最大占空比（与 ARR=2000 对应） |
| `MOTOR_TIM_ARR` | 2000 | 定时器自动重装载值（须与 CubeMX 中一致） |
| `PID_RATE` | 50 | PID 控制频率 (Hz) |
| `WHEEL_RESOLUTION` | 1040.0 | 每转编码器计数值（26极 × 4边沿 × 20减速比） |
| `MEC_WHEEL_DIAMETER` | 0.048 | 轮径 (m) |
| `MEC_WHEEL_BASE` | 0.128 | 左右轮距 (m) |
| `MEC_ACLE_BASE` | 0.146 | 前后轴距 (m) |
| `motor_kp` | 800 | PID 比例系数 |
| `motor_kd` | 400 | PID 微分系数 |

> PID 参数从 STM32F103 原工程直接移植。由于 PWM 频率、电压、电机参数相同，通常无需重新整定。

---

## 8. 调试注意事项

1. **PWM 频率验证**：用示波器测量 PE9/PE11/PE13/PE14/PE5/PE6/PB14/PB15 任一引脚，确认 PWM 频率 ≈ 21 kHz
2. **编码器方向**：用手转动轮子，观察 `Wheel_x.RT` 的正负是否正确。若方向相反，可在 CubeMX 中将对应编码器的 IC1/IC2 Polarity 取反
3. **电机方向**：若某电机转向与预期相反，交换该电机的正转/反转 PWM 通道定义（修改 `motor.c` 中对应 `MOTOR_x_SetSpeed` 函数内的 CHx/CHy 分配）
4. **TIM8 配置**：若忘记在 CubeMX 中修改 TIM8 PSC=0 和 TI12 模式，电机 D 的编码器反馈速度将为实际值的 1/338，导致 PID 失控
