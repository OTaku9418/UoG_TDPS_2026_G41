#include "tracking.h"
#include "motor.h"
#include "oled.h"
#include "lora.h"
#include <stdio.h>
#include <string.h>

/* ---------- 状态机时间阈值 (假设每 20ms 一次) ---------- */
#define ACTION_COOLDOWN_MS  20  // 动作冷却与探头防抖时间(20毫秒)
#define LOST_FORWARD_TICKS  5
#define BACK_TICKS          12
#define SEARCH_TICKS        17
#define BACK_HIT_TURN_TICKS 15
#define CROSSROAD_STRAIGHT_TICKS   35
#define RIGHT_ANGLE_STRAIGHT_TICKS 38
#define STOP_TICKS          5
#define RIGHT_TURN_TICKS    15

typedef enum {
    REC_NONE = 0,
    REC_LOST_FORWARD,
    REC_BACK,
    REC_LEFT_SEARCH,
    REC_RETURN_FROM_LEFT,
    REC_RIGHT_SEARCH,
    REC_RETURN_FROM_RIGHT,
    REC_BACK_LEFT_TURN,
    REC_BACK_RIGHT_TURN,
    REC_ALL_ON_STOP,
    REC_ALL_ON_RIGHT_TURN,
    REC_ALL_ON_STRAIGHT,
    REC_ALL_ON_LEFT_TURN,
    REC_RIGHT_ANGLE_LEFT,
    REC_RIGHT_ANGLE_RIGHT,
    REC_RADAR_TURN_60,
    REC_RADAR_STOP_10S,
    REC_RADAR_START_RIGHT,
    REC_RADAR_START_LEFT
} RecoverState;

typedef enum {
    PHASE_INITIAL = 0,
    PHASE_AFTER_CROSS_3,
    PHASE_AFTER_CROSS_5,
    PHASE_RADAR_STOP,
    PHASE_RADAR_EXECUTE_RIGHT,
    PHASE_RADAR_EXECUTE_LEFT,
    PHASE_RADAR_EXIT
} MissionPhase;

static RecoverState recover_state = REC_NONE;
static MissionPhase mission_phase = PHASE_INITIAL;
static uint16_t recover_tick = 0;
static uint8_t all_on_count = 0;
static uint8_t right_angle_l_count = 0;
static uint8_t right_angle_r_count = 0;
static uint8_t local_right_ang_r = 0;
static uint8_t local_left_ang_l = 0;
static uint16_t stable_tracking_ticks = 0;
static uint16_t turn_cooldown_ticks = 0; // 全局出弯冷却期
static uint8_t oled_force_refresh = 0;

static void Smart_OLED_ShowState(char *new_state) {
    static char last_state[17] = {0};
    if (oled_force_refresh || strcmp(last_state, new_state) != 0) {
        OLED_ShowString(0, 0, new_state);
        strcpy(last_state, new_state);
    }
}

static void Recovery_Reset(void)
{
    recover_state = REC_NONE;
    recover_tick = 0;
    turn_cooldown_ticks = 25; // 出弯/自救后赋予 0.5s 的无敌屏蔽期
}

/* ------------------------------------------------------------------------- */
void Tracking_Init(void)
{
    Recovery_Reset();
    mission_phase = PHASE_INITIAL;
    all_on_count = 0;
    right_angle_l_count = 0;
    right_angle_r_count = 0;
    local_right_ang_r = 0;
    local_left_ang_l = 0;
    stable_tracking_ticks = 0;
    turn_cooldown_ticks = 0;
}

/* ------------------------------------------------------------------------- */
void Tracking_Pro_Reset(void)
{
    Recovery_Reset();
    all_on_count = 0;
    right_angle_l_count = 0;
    local_left_ang_l = 0;
    stable_tracking_ticks = 0;
    turn_cooldown_ticks = 0;
}

// ======================= Checkpoint Menu =======================
static uint8_t is_checkpoint_mode = 0;

void Tracking_Checkpoint_Menu(void)
{
    // 如果没有按下按键，且没在检查点模式，直接返回
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_SET && !is_checkpoint_mode) {
        return;
    }
    
    // 如果没在检查点模式，但按下了按键，说明要进入检查点模式
    if (!is_checkpoint_mode) {
        // debounce
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
            motor_speed_set(0, 0, 0, 0); // 停车
            is_checkpoint_mode = 1;
            OLED_Clear();
            OLED_ShowString(0, 0, "select checkpnt ");
            while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET); // wait release
        } else {
            return;
        }
    }

    // 现在在检查点模式下
    uint8_t current_cp = 1;
    char oled_buf[17];
    uint32_t last_press_time;
    
SELECT_START:
    last_press_time = HAL_GetTick();
    current_cp = 1;
    OLED_ShowString(0, 2, "checkpoint 1    ");
    
    while(1) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                current_cp++;
                if (current_cp > 5) current_cp = 1;
                
                if (current_cp <= 5) {
                    sprintf(oled_buf, "checkpoint %d    ", current_cp);
                } 
                OLED_ShowString(0, 2, oled_buf);
                
                last_press_time = HAL_GetTick();
                while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
            }
        }
        
        // 超过1.2秒未按键，锁定检查点
        if (HAL_GetTick() - last_press_time > 1200) {
            break;
        }
    }
    
    // 锁定检查点，等待再次按下确认
    OLED_ShowString(0, 4, "wait for resume ");
    while(1) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
                break; // 确认恢复
            }
        }
    }
    
    // 恢复检查点状态
    Tracking_Pro_Reset(); // 默认恢复所有变量
    OLED_Clear();
    
    if (current_cp == 1) {
        all_on_count = 0;
        mission_phase = PHASE_INITIAL;
    } else if (current_cp == 2) {
        all_on_count = 1;
        mission_phase = PHASE_INITIAL;
    } else if (current_cp == 3) {
        all_on_count = 2;
        mission_phase = PHASE_INITIAL;
    } else if (current_cp == 4) {
        all_on_count = 5;
        local_left_ang_l = 1;
        local_right_ang_r = 1;
        right_angle_l_count = 1;
        right_angle_r_count = 1;
        mission_phase = PHASE_AFTER_CROSS_5;
    } else if (current_cp == 5) {
        all_on_count = 7;
        mission_phase = PHASE_INITIAL;
    }
    
    oled_force_refresh = 1; // 强制刷新所有的OLED显示缓存
    is_checkpoint_mode = 0;
}

/*
 * 四路红外巡线 (带高级状态机防丢线)
 * 传感器布局 (俯视):
 *    X4(左外)   X3(左内)   X2(右内)   X1(右外)
 *      PC5        PB1        PE8       PE10
 *
 * 假定：1 = 探测到黑线，0 = 白色地面
 */
void Tracking_Pro_Run(void)
{
    Tracking_Checkpoint_Menu();
    if (is_checkpoint_mode) return;

    static uint32_t last_run = 0;
    uint32_t current_tick = HAL_GetTick();
    
    // 【核心冷却逻辑】距离上次判断如果不满 ACTION_COOLDOWN_MS，直接跳出
    // 强制无视此时段内探头发生的高频毛刺
    if (current_tick - last_run < ACTION_COOLDOWN_MS) {
        return; 
    }
    
    // Delta Time 帧率补偿：如果 OLED 刷屏等导致 while(1) 卡顿，
    // 自动计算本次循环相当于流逝了几个基准 Tick，实现时间补偿
    uint8_t tick_steps = (current_tick - last_run) / ACTION_COOLDOWN_MS;
    if (tick_steps > 10) tick_steps = 10; // 限制单次最大步长防跳跃
    last_run += tick_steps * ACTION_COOLDOWN_MS;

    if (turn_cooldown_ticks > 0) {
        if (turn_cooldown_ticks > tick_steps) turn_cooldown_ticks -= tick_steps;
        else turn_cooldown_ticks = 0;
    }

    uint8_t X1, X2, X3, X4;
    
    X4 = IR_X4_READ(); // Left-Out
    X3 = IR_X3_READ(); // Left-In
    X2 = IR_X2_READ(); // Right-In
    X1 = IR_X1_READ(); // Right-Out

    // ================= TOF 检测与 LoRa 发送逻辑 =================
    static uint32_t last_tof_check_tick = 0;
    static uint8_t tof_sent_in_current_interval = 0;
    static uint8_t last_all_on_count_for_tof = 0xFF;

    // 当路口计数发生变化时，清除“当前区间已发送”标志
    if (all_on_count != last_all_on_count_for_tof) {
        tof_sent_in_current_interval = 0;
        last_all_on_count_for_tof = all_on_count;
    }

    // 检查是否位于指定区间 (第2次到第3次路口之间 -> all_on_count == 2) 或 (>= 8)
    if ((all_on_count == 2 || all_on_count >= 8) && !tof_sent_in_current_interval) {
        // 每半秒执行一次，且必须在完美直行且非特殊恢复状态时，防止200ms的TOF阻塞导致转弯失控
        if (current_tick - last_tof_check_tick >= 500) {
            if (X4 == 0 && X3 == 1 && X2 == 1 && X1 == 0 && recover_state == REC_NONE) {
                last_tof_check_tick = HAL_GetTick(); // 记录开始检测时间
                if (TOF_CheckAndSend() == 1) {
                    tof_sent_in_current_interval = 1; // 标记该区间已成功发送
                }
                // TOF通信会阻塞较长时间，强制更新 last_run 以防时间补偿过量
                last_run = HAL_GetTick();
            }
        }
    }

    // 智能按需刷新探头状态 (仅在发生变化时刷新OLED，极大降低I2C堵塞)
    static uint8_t last_X = 0xFF;
    uint8_t curr_X = (X4 << 3) | (X3 << 2) | (X2 << 1) | X1;
    if (oled_force_refresh || curr_X != last_X) {
        OLED_ShowChar(0,  2, X4 == 1 ? 127 : ' ');
        OLED_ShowChar(16, 2, X3 == 1 ? 127 : ' ');
        OLED_ShowChar(32, 2, X2 == 1 ? 127 : ' ');
        OLED_ShowChar(48, 2, X1 == 1 ? 127 : ' ');
        last_X = curr_X;
    }

    // 智能按需刷新计数器
    static uint8_t last_all_on = 0xFF;
    if (oled_force_refresh || all_on_count != last_all_on) {
        char dbg_str[17];
        sprintf(dbg_str, "Cross Count: %-2d", all_on_count);
        OLED_ShowString(0, 4, dbg_str);
        last_all_on = all_on_count;
    }
    
    static uint8_t last_ang_l = 0xFF, last_ang_r = 0xFF;
    if (oled_force_refresh || right_angle_l_count != last_ang_l || right_angle_r_count != last_ang_r) {
        char dbg_str[17];
        sprintf(dbg_str, "AngL:%-2d AngR:%-2d", right_angle_l_count, right_angle_r_count);
        OLED_ShowString(0, 6, dbg_str);
        last_ang_l = right_angle_l_count;
        last_ang_r = right_angle_r_count;
    }
    
    oled_force_refresh = 0; // 清除强制刷新标志

    // 提取语义布尔变量
    uint8_t lost_line = (X1 == 0 && X2 == 0 && X3 == 0 && X4 == 0); 
    uint8_t all_on    = (X1 == 1 && X2 == 1 && X3 == 1 && X4 == 1);
    uint8_t is_right_angle_l = (X4 == 1 && X3 == 1 && X2 == 1 && X1 == 0);
    uint8_t is_right_angle_r = (X4 == 0 && X3 == 1 && X2 == 1 && X1 == 1);
    
    // ================= 全局出弯无敌冷却期 =================
    if (turn_cooldown_ticks > 0) {
        all_on = 0;
        is_right_angle_l = 0;
        is_right_angle_r = 0;
    }
    
    // ================= 阶段性强行屏蔽（方案一）=================
    // 第3次十字路口右转后，必须经过一个左直角弯。期间屏蔽任何假十字路口触发。
    if (mission_phase == PHASE_AFTER_CROSS_3) {
        if (all_on) {
            all_on = 0; // 强行屏蔽十字路口触发
            is_right_angle_l = 1; // 哪怕扫到了四个亮，也强行当作左直角弯处理
        }
    }
    
    // 第9次路口后，无视所有右直角弯
    if (all_on_count >= 9) {
        is_right_angle_r = 0;
    }
    
    static uint8_t right_angle_cleared = 1;
    if (!is_right_angle_l && !is_right_angle_r) {
        right_angle_cleared = 1;
    }

    // 判断当前是否处于丢线找线状态 (包含冲刺、倒车、左右扫描)
    uint8_t is_recovering = (recover_state == REC_LOST_FORWARD || recover_state == REC_BACK || 
                             recover_state == REC_LEFT_SEARCH  || recover_state == REC_RETURN_FROM_LEFT || 
                             recover_state == REC_RIGHT_SEARCH || recover_state == REC_RETURN_FROM_RIGHT);

    // 粗略的探头方位区分，用于后退时触线打方向
    uint8_t left_on   = (X4 == 1); 
    uint8_t right_on  = (X1 == 1); 
    uint8_t center_on = (X3 == 1 || X2 == 1); 

    uint8_t back_left_turn_hit  = (recover_state == REC_BACK && center_on && left_on && !right_on);
    uint8_t back_right_turn_hit = (recover_state == REC_BACK && center_on && right_on && !left_on);

    // ==================== 1. 后退碰线急转打正状态 ====================
    if(recover_state == REC_BACK_LEFT_TURN)
    {
        Smart_OLED_ShowState("Rec: Bk LeftTurn");
        motor_speed_set(0.0, 0.15, 0.0, 0.15); // 向左打方向
        recover_tick += tick_steps;
        if(recover_tick >= BACK_HIT_TURN_TICKS) Recovery_Reset();
        return;
    }

    if(recover_state == REC_BACK_RIGHT_TURN)
    {
        Smart_OLED_ShowState("Rec: Bk RghtTurn");
        motor_speed_set(0.15, 0.0, 0.15, 0.0); // 向右打方向
        recover_tick += tick_steps;
        if(recover_tick >= BACK_HIT_TURN_TICKS) Recovery_Reset();
        return;
    }

    // ==================== 2. 十字路口策略状态机 ====================
    uint8_t valid_crossroad = all_on;
    // 核心修改1：如果是刚从找线状态中碰到全亮，说明是横向扫到了线，不能算作十字路口
    if (is_recovering) {
        valid_crossroad = 0; 
    }
    // 核心修改2：十字路口生效前，必须稳定直行/微调超过 12 个 tick (~0.24s，放宽条件防漏检)
    if (stable_tracking_ticks < 12) {
        valid_crossroad = 0;
    }

    // 交叉路口防重复触发标志位：只有离开过十字路口，才允许下一次计数
    static uint8_t crossroad_cleared = 1;
    if (!valid_crossroad) {
        crossroad_cleared = 1; 
    }

    // 更新稳定巡线计数器 (必须在评估了 valid_crossroad 之后更新)
    uint8_t is_raw_stable = (
        (X4==0 && X3==1 && X2==1 && X1==0) || // 完美直行
        (X4==0 && X3==1 && X2==0 && X1==0) || // 微偏右
        (X4==0 && X3==0 && X2==1 && X1==0)    // 微偏左
    );
    if (recover_state == REC_NONE && is_raw_stable && !valid_crossroad) {
        if (stable_tracking_ticks < 0xFFFF) stable_tracking_ticks += tick_steps;
    } else {
        stable_tracking_ticks = 0; // 发生急转、路口、自救等剧烈动作，重新计算稳定时间
    }

    if (valid_crossroad && crossroad_cleared)
    {
trigger_crossroad:
        all_on_count++;
        crossroad_cleared = 0; // 锁定：直到完全离开这个路口前，不再重复计数
        
        // 每次十字路口计数时，清零左右直角弯计数
        local_left_ang_l = 0;
        local_right_ang_r = 0;
        right_angle_l_count = 0;
        right_angle_r_count = 0;
        
        if (all_on_count == 1 || all_on_count == 2) {
            recover_state = REC_ALL_ON_STRAIGHT;
        } else if (all_on_count == 3) {
            recover_state = REC_ALL_ON_RIGHT_TURN;
            mission_phase = PHASE_AFTER_CROSS_3;
            local_left_ang_l = 0;
        } else if (all_on_count == 4 || all_on_count == 5) {
            recover_state = REC_ALL_ON_STRAIGHT;
            if (all_on_count == 5) {
                mission_phase = PHASE_AFTER_CROSS_5;
                local_right_ang_r = 0;
            }
        } else if (all_on_count == 6 || all_on_count == 7 || all_on_count == 9) {
            recover_state = REC_ALL_ON_LEFT_TURN;
            mission_phase = PHASE_INITIAL;
            local_left_ang_l = 0;
        } else if (all_on_count == 8) {
            recover_state = REC_RADAR_TURN_60;
            mission_phase = PHASE_RADAR_STOP;
        } else {
            recover_state = REC_ALL_ON_STRAIGHT;
        }
        recover_tick = 0;
        return;
    }

    if (recover_state == REC_ALL_ON_STRAIGHT) {
        Smart_OLED_ShowState("Cross: Straight ");
        motor_speed_set(0.15, 0.15, 0.15, 0.15);
        recover_tick += tick_steps;
        if(recover_tick >= CROSSROAD_STRAIGHT_TICKS) Recovery_Reset();
        return;
    }

    if (recover_state == REC_ALL_ON_STOP) {
        Smart_OLED_ShowState("Cross: Stop     ");
        motor_speed_set(0.0, 0.0, 0.0, 0.0);
        recover_tick += tick_steps;
        if(recover_tick >= STOP_TICKS) {
            recover_state = REC_ALL_ON_RIGHT_TURN;
            recover_tick = 0;
        }
        return;
    }

    if (recover_state == REC_ALL_ON_RIGHT_TURN) {
        if (recover_tick < CROSSROAD_STRAIGHT_TICKS) {
            Smart_OLED_ShowState("Cross: Rght Fwd ");
            motor_speed_set(0.15, 0.15, 0.15, 0.15); // 前冲补偿
        } else {
            Smart_OLED_ShowState("Cross: TurnRight");
            motor_speed_set(0.15, -0.15, 0.15, -0.15); // 右转
            if (recover_tick > CROSSROAD_STRAIGHT_TICKS + 25) {
                if (X4 == 0 && X3 == 1 && X2 == 1 && X1 == 0) {
                    Recovery_Reset();
                    return;
                }
            }
        }
        recover_tick += tick_steps;
        return;
    }

    if (recover_state == REC_ALL_ON_LEFT_TURN) {
        if (recover_tick < CROSSROAD_STRAIGHT_TICKS) {
            Smart_OLED_ShowState("Cross: Left Fwd ");
            motor_speed_set(0.15, 0.15, 0.15, 0.15); // 前冲补偿
        } else {
            Smart_OLED_ShowState("Cross: TurnLeft ");
            motor_speed_set(-0.15, 0.15, -0.15, 0.15); // 左转
            if (recover_tick > CROSSROAD_STRAIGHT_TICKS + 25) {
                if (X4 == 0 && X3 == 1 && X2 == 1 && X1 == 0) {
                    Recovery_Reset();
                    return;
                }
            }
        }
        recover_tick += tick_steps;
        return;
    }

    if (recover_state == REC_RADAR_TURN_60) {
        if (recover_tick < CROSSROAD_STRAIGHT_TICKS) {
            Smart_OLED_ShowState("RADAR: Fwd Comp ");
            motor_speed_set(0.15, 0.15, 0.15, 0.15); // 驶过路口的直行补偿
        } else {
            Smart_OLED_ShowState("RADAR: Turn 60  ");
            motor_speed_set(0.15, -0.15, 0.15, -0.15); // 右转约60度
        }
        
        recover_tick += tick_steps;
        
        if (recover_tick >= CROSSROAD_STRAIGHT_TICKS + 45) { // 补偿 + 45 tick 转向
            recover_state = REC_RADAR_STOP_10S;
            recover_tick = 0;
        }
        return;
    }

    if (recover_state == REC_RADAR_STOP_10S) {
        Smart_OLED_ShowState("RADAR: Stop 10S ");
        
        OLED_DrawRadarWave(recover_tick); // 在下方屏幕画雷达动画
        
        motor_speed_set(0, 0, 0, 0); // 停车10秒

        recover_tick += tick_steps;
        if (recover_tick >= 500) { // 500 * 20ms = 20s
            uint8_t radar_result_is_right = 0; 
            // 读取OT2电平: 高电平向左转, 低电平向右转
            if (HAL_GPIO_ReadPin(radar_ot2_GPIO_Port, radar_ot2_Pin) == GPIO_PIN_RESET) {
                radar_result_is_right = 1; // 低电平 -> 右转
            } else {
                radar_result_is_right = 0; // 高电平 -> 左转
            }
            OLED_Clear(); // 动画结束，清除整个屏幕的图形
            oled_force_refresh = 1; // 强制重新刷新文字状态
            
            local_right_ang_r = 0;
            local_left_ang_l = 0;
            if (radar_result_is_right) {
                mission_phase = PHASE_RADAR_EXECUTE_RIGHT;
                recover_state = REC_RADAR_START_RIGHT;
            } else {
                mission_phase = PHASE_RADAR_EXECUTE_LEFT;
                recover_state = REC_RADAR_START_LEFT;
            }
            recover_tick = 0;
        }
        return;
    }

    if (recover_state == REC_RADAR_START_RIGHT) {
        Smart_OLED_ShowState("RADAR: Go Right ");
        motor_speed_set(0.15, -0.15, 0.15, -0.15); // 强制小幅右转
        recover_tick += tick_steps;
        if (recover_tick > 10) {
            if (X3 == 1 || X2 == 1) Recovery_Reset();
        }
        return;
    }

    if (recover_state == REC_RADAR_START_LEFT) {
        Smart_OLED_ShowState("RADAR: Go Left  ");
        motor_speed_set(-0.15, 0.15, -0.15, 0.15); // 强制左转找线
        recover_tick += tick_steps;
        if (recover_tick > 60) {
            if (X3 == 1 || X2 == 1) Recovery_Reset();
        }
        return;
    }

    // ==================== 2.5 直角弯策略状态机 ====================
    if (recover_state == REC_RIGHT_ANGLE_LEFT)
    {
        if (recover_tick < RIGHT_ANGLE_STRAIGHT_TICKS) {
            Smart_OLED_ShowState("Right-Ang L Fwd ");
            motor_speed_set(0.15, 0.15, 0.15, 0.15); // 前冲补偿 0.5s
            
            if (curr_X == 0x0F && mission_phase != PHASE_AFTER_CROSS_3) {
                if (right_angle_l_count > 0) right_angle_l_count--; 
                if (local_left_ang_l > 0) local_left_ang_l--;
                goto trigger_crossroad;
            }
        } else {
            Smart_OLED_ShowState("Right-Ang L Turn");
            
            uint8_t force_straight = 0;
            uint8_t ignore_line_check = 0;

            if (mission_phase == PHASE_RADAR_EXECUTE_LEFT) {
                if (local_right_ang_r < 2) {
                    force_straight = 1; // 忽略所有左直角弯
                }
            } else if (mission_phase == PHASE_RADAR_EXIT) {
                ignore_line_check = 1; // 驶离后所有直角采取有线忽略策略
            }

            if (force_straight) {
                Recovery_Reset();
                return;
            }

            if (ignore_line_check && curr_X != 0) {
                Recovery_Reset();
                return;
            }

            motor_speed_set(-0.15, 0.15, -0.15, 0.15); // 左转
            if (recover_tick > RIGHT_ANGLE_STRAIGHT_TICKS + 25) {
                if (X4 == 0 && X3 == 1 && X2 == 1 && X1 == 0) {
                    if (mission_phase == PHASE_RADAR_EXECUTE_LEFT && local_right_ang_r >= 2) {
                        mission_phase = PHASE_RADAR_EXIT;
                    } else if (mission_phase == PHASE_AFTER_CROSS_3 && local_left_ang_l >= 1) {
                        mission_phase = PHASE_INITIAL; // 结束屏蔽期
                    }
                    Recovery_Reset();
                    return;
                }
            }
        }
        recover_tick += tick_steps;
        return;
    }

    if (recover_state == REC_RIGHT_ANGLE_RIGHT)
    {
        if (recover_tick < RIGHT_ANGLE_STRAIGHT_TICKS) {
            Smart_OLED_ShowState("Right-Ang R Fwd ");
            motor_speed_set(0.15, 0.15, 0.15, 0.15); // 前冲补偿 0.5s
            
            if (curr_X == 0x0F) {
                if (right_angle_r_count > 0) right_angle_r_count--; 
                if (local_right_ang_r > 0) local_right_ang_r--;
                goto trigger_crossroad;
            }
        } else {
            Smart_OLED_ShowState("Right-Ang R Turn");
            
            uint8_t force_turn = 0;
            uint8_t force_straight = 0;

            if (mission_phase == PHASE_AFTER_CROSS_5) {
                if (local_right_ang_r == 1) force_turn = 1;
                else if (local_right_ang_r == 2) force_straight = 1;
                else if (local_right_ang_r >= 3) force_turn = 1;
            } else if (mission_phase == PHASE_RADAR_EXECUTE_RIGHT) {
                if (local_right_ang_r == 1) force_turn = 1;
            } else if (mission_phase == PHASE_RADAR_EXECUTE_LEFT) {
                if (local_right_ang_r <= 2) force_turn = 1;
            }

            if (force_straight) {
                Recovery_Reset();
                return;
            }

            if (!force_turn && curr_X != 0) { 
                Recovery_Reset();
                return;
            }
            
            motor_speed_set(0.15, -0.15, 0.15, -0.15); // 右转
            if (recover_tick > RIGHT_ANGLE_STRAIGHT_TICKS + 25) {
                if (X4 == 0 && X3 == 1 && X2 == 1 && X1 == 0) {
                    Recovery_Reset();
                    return;
                }
            }
        }
        recover_tick += tick_steps;
        return;
    }

    // ==================== 3. 拦截后退时的触线事件 ====================
    if(back_left_turn_hit)
    {
        recover_state = REC_BACK_LEFT_TURN;
        recover_tick = 0;
        motor_speed_set(0.0, 0.15, 0.0, 0.15);
        recover_tick += tick_steps;
        return;
    }

    if(back_right_turn_hit)
    {
        recover_state = REC_BACK_RIGHT_TURN;
        recover_tick = 0;
        motor_speed_set(0.15, 0.0, 0.15, 0.0);
        recover_tick += tick_steps;
        return;
    }

    // 核心修改：在整个自救扫描期间（不仅限于倒车），只要重新找到了黑线，立刻中断自救，恢复正常巡线！
    if(is_recovering && !lost_line)
    {
        Recovery_Reset();
    }

    // ==================== 4. 正常巡线模式 ====================
    if (!lost_line && recover_state == REC_NONE)
    {
        // 1. 直角弯左转 (单边三路: X4, X3, X2 = 1110)
        if(is_right_angle_l)
        {
            Smart_OLED_ShowState("Right-Ang L     ");
            if (right_angle_cleared) {
                right_angle_l_count++;
                local_left_ang_l++;
                right_angle_cleared = 0;
                stable_tracking_ticks = 0;
            }
            recover_state = REC_RIGHT_ANGLE_LEFT;
            recover_tick = 0;
            return;
        }
        // 2. 直角弯右转 (单边三路: X3, X2, X1 = 0111)
        else if(is_right_angle_r)
        {
            Smart_OLED_ShowState("Right-Ang R     ");
            if (right_angle_cleared) {
                right_angle_r_count++;
                local_right_ang_r++;
                right_angle_cleared = 0;
                stable_tracking_ticks = 0;
            }
            recover_state = REC_RIGHT_ANGLE_RIGHT;
            recover_tick = 0;
            return;
        }
        // 3. 完美直行 (仅中间两路 0110)
        else if(X4 == 0 && X3 == 1 && X2 == 1 && X1 == 0)
        {
            Smart_OLED_ShowState("Go Straight     ");
            motor_speed_set(0.15, 0.15, 0.15, 0.15);
        }
        // 4. 左内压线 (0100) -> 微左转
        else if(X4 == 0 && X3 == 1 && X2 == 0 && X1 == 0)
        {
            Smart_OLED_ShowState("Turn Left Slight");
            motor_speed_set(0.08, 0.15, 0.08, 0.15);
        }
        // 5. 左内和左外压线 (1100) -> 中度左转
        else if(X4 == 1 && X3 == 1 && X2 == 0 && X1 == 0)
        {
            Smart_OLED_ShowState("Turn Left Mid   ");
            motor_speed_set(0.0, 0.15, 0.0, 0.15);
        }
        // 6. 仅左外压线或直角弯冷却期左偏 -> 大角度左转
        else if((X4 == 1 && X3 == 0 && X2 == 0 && X1 == 0) || (X4 == 1 && X3 == 1 && X2 == 1 && X1 == 0))
        {
            Smart_OLED_ShowState("Turn Left Sharp ");
            motor_speed_set(-0.1, 0.15, -0.1, 0.15);
        }
        // 7. 右内压线 (0010) -> 微右转
        else if(X4 == 0 && X3 == 0 && X2 == 1 && X1 == 0)
        {
            Smart_OLED_ShowState("Turn Right Sligh");
            motor_speed_set(0.15, 0.08, 0.15, 0.08);
        }
        // 8. 右内和右外压线 (0011) -> 中度右转
        else if(X4 == 0 && X3 == 0 && X2 == 1 && X1 == 1)
        {
            Smart_OLED_ShowState("Turn Right Mid  ");
            motor_speed_set(0.15, 0.0, 0.15, 0.0);
        }
        // 9. 仅右外压线或直角弯冷却期右偏 -> 大角度右转
        else if((X4 == 0 && X3 == 0 && X2 == 0 && X1 == 1) || (X4 == 0 && X3 == 1 && X2 == 1 && X1 == 1))
        {
            Smart_OLED_ShowState("Turn Right Sharp");
            motor_speed_set(0.15, -0.1, 0.15, -0.1);
        }
        // 10. 其他散乱异常状态 (如跳变的 1010, 1001等)，保持直行冲过异常区
        else
        {
            Smart_OLED_ShowState("Abnormal, Fwd   ");
            motor_speed_set(0.15, 0.15, 0.15, 0.15);
        }
        return;
    }

    // ==================== 5. 丢线全套“雷达式扫描”自救状态机 ====================
    if(recover_state == REC_NONE)
    {
        recover_state = REC_LOST_FORWARD;
        recover_tick = 0;
    }

    switch(recover_state)
    {
        case REC_LOST_FORWARD:
            Smart_OLED_ShowState("Rec: Forward    ");
            motor_speed_set(0.15, 0.15, 0.15, 0.15);
            recover_tick += tick_steps;
            if(recover_tick >= LOST_FORWARD_TICKS) {
                recover_state = REC_BACK;
                recover_tick = 0;
            }
            break;

        case REC_BACK:
            Smart_OLED_ShowState("Rec: Backwards  ");
            motor_speed_set(-0.15, -0.15, -0.15, -0.15);
            recover_tick += tick_steps;
            if(recover_tick >= BACK_TICKS) {
                recover_state = REC_LEFT_SEARCH;
                recover_tick = 0;
            }
            break;

        case REC_LEFT_SEARCH:
            Smart_OLED_ShowState("Rec: Search L   ");
            motor_speed_set(-0.15, 0.15, -0.15, 0.15); // 原地左转
            recover_tick += tick_steps;
            if(recover_tick >= SEARCH_TICKS) {
                recover_state = REC_RETURN_FROM_LEFT;
                recover_tick = 0;
            }
            break;

        case REC_RETURN_FROM_LEFT:
            Smart_OLED_ShowState("Rec: Return L   ");
            motor_speed_set(0.15, -0.15, 0.15, -0.15); // 原地右转回正
            recover_tick += tick_steps;
            if(recover_tick >= SEARCH_TICKS) {
                recover_state = REC_RIGHT_SEARCH;
                recover_tick = 0;
            }
            break;

        case REC_RIGHT_SEARCH:
            Smart_OLED_ShowState("Rec: Search R   ");
            motor_speed_set(0.15, -0.15, 0.15, -0.15); // 继续原地右转
            recover_tick += tick_steps;
            if(recover_tick >= SEARCH_TICKS) {
                recover_state = REC_RETURN_FROM_RIGHT;
                recover_tick = 0;
            }
            break;

        case REC_RETURN_FROM_RIGHT:
            Smart_OLED_ShowState("Rec: Return R   ");
            motor_speed_set(-0.15, 0.15, -0.15, 0.15); // 原地左转回正
            recover_tick += tick_steps;
            if(recover_tick >= SEARCH_TICKS) {
                recover_state = REC_BACK;  // 扫不到则继续后退
                recover_tick = 0;
            }
            break;

        default:
            recover_state = REC_BACK;
            recover_tick = 0;
            break;
    }
}
