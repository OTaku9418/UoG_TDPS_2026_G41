#include "motor.h"

float motor_kp = 800.0f;
float motor_ki = 600.0f;
float motor_kd = 150.0f;

ROBOT_Wheel Wheel_A, Wheel_B, Wheel_C, Wheel_D;

/* ==================== motor PWM start ==================== */

void Motor_Init(void)
{
    /* start PWM outputs on all three timers (configuration done by CubeMX) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

/* ==================== encoder start ==================== */

void Encoder_Init(void)
{
    /* start encoder counting (configuration done by CubeMX) */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

/* ==================== encoder counter read ==================== */

int16_t Encoder_A_GetCounter(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    return cnt;
}

int16_t Encoder_B_GetCounter(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return cnt;
}

int16_t Encoder_C_GetCounter(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    return cnt;
}

int16_t Encoder_D_GetCounter(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    return cnt;
}

/* ==================== PID speed controllers ==================== */

int16_t SPEED_PidCtlA(float spd_target, float spd_current)
{
    static float pwm_out = 0.0f;
    static float err_last = 0.0f;
    static float err_prev = 0.0f;
    float err = spd_target - spd_current;

    pwm_out += motor_kp * (err - err_last) 
             + motor_ki * err 
             + motor_kd * (err - 2.0f * err_last + err_prev);
    
    err_prev = err_last;
    err_last = err;

    if (pwm_out > MOTOR_PWM_MAX)  pwm_out = MOTOR_PWM_MAX;
    if (pwm_out < -MOTOR_PWM_MAX) pwm_out = -MOTOR_PWM_MAX;

    return (int16_t)pwm_out;
}

int16_t SPEED_PidCtlB(float spd_target, float spd_current)
{
    static float pwm_out = 0.0f;
    static float err_last = 0.0f;
    static float err_prev = 0.0f;
    float err = spd_target - spd_current;

    pwm_out += motor_kp * (err - err_last) 
             + motor_ki * err 
             + motor_kd * (err - 2.0f * err_last + err_prev);
    
    err_prev = err_last;
    err_last = err;

    if (pwm_out > MOTOR_PWM_MAX)  pwm_out = MOTOR_PWM_MAX;
    if (pwm_out < -MOTOR_PWM_MAX) pwm_out = -MOTOR_PWM_MAX;

    return (int16_t)pwm_out;
}

int16_t SPEED_PidCtlC(float spd_target, float spd_current)
{
    static float pwm_out = 0.0f;
    static float err_last = 0.0f;
    static float err_prev = 0.0f;
    float err = spd_target - spd_current;

    pwm_out += motor_kp * (err - err_last) 
             + motor_ki * err 
             + motor_kd * (err - 2.0f * err_last + err_prev);
    
    err_prev = err_last;
    err_last = err;

    if (pwm_out > MOTOR_PWM_MAX)  pwm_out = MOTOR_PWM_MAX;
    if (pwm_out < -MOTOR_PWM_MAX) pwm_out = -MOTOR_PWM_MAX;

    return (int16_t)pwm_out;
}

int16_t SPEED_PidCtlD(float spd_target, float spd_current)
{
    static float pwm_out = 0.0f;
    static float err_last = 0.0f;
    static float err_prev = 0.0f;
    float err = spd_target - spd_current;

    pwm_out += motor_kp * (err - err_last) 
             + motor_ki * err 
             + motor_kd * (err - 2.0f * err_last + err_prev);
    
    err_prev = err_last;
    err_last = err;

    if (pwm_out > MOTOR_PWM_MAX)  pwm_out = MOTOR_PWM_MAX;
    if (pwm_out < -MOTOR_PWM_MAX) pwm_out = -MOTOR_PWM_MAX;

    return (int16_t)pwm_out;
}

/* ==================== motor speed set (2-channel PWM) ==================== */

/*
 * Motor A: TIM1 CH1(PE9)=fwd, CH2(PE11)=rev, encoder=TIM2
 */
void MOTOR_A_SetSpeed(int16_t speed)
{
    int16_t temp = speed;

    if (temp > MOTOR_PWM_MAX)  temp = MOTOR_PWM_MAX;
    if (temp < -MOTOR_PWM_MAX) temp = -MOTOR_PWM_MAX;

    if (temp > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, temp);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -temp);
    }
}

/*
 * Motor B: TIM1 CH3(PE13)=fwd, CH4(PE14)=rev, encoder=TIM3
 */
void MOTOR_B_SetSpeed(int16_t speed)
{
    int16_t temp = speed;

    if (temp > MOTOR_PWM_MAX)  temp = MOTOR_PWM_MAX;
    if (temp < -MOTOR_PWM_MAX) temp = -MOTOR_PWM_MAX;

    if (temp > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, temp);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -temp);
    }
}

/*
 * Motor C: TIM9 CH1(PE5)=fwd, CH2(PE6)=rev, encoder=TIM4
 */
void MOTOR_C_SetSpeed(int16_t speed)
{
    int16_t temp = speed;

    if (temp > MOTOR_PWM_MAX)  temp = MOTOR_PWM_MAX;
    if (temp < -MOTOR_PWM_MAX) temp = -MOTOR_PWM_MAX;

    if (temp > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, temp);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, -temp);
    }
}

/*
 * Motor D: TIM12 CH1(PB14)=fwd, CH2(PB15)=rev, encoder=TIM8
 */
void MOTOR_D_SetSpeed(int16_t speed)
{
    int16_t temp = speed;

    if (temp > MOTOR_PWM_MAX)  temp = MOTOR_PWM_MAX;
    if (temp < -MOTOR_PWM_MAX) temp = -MOTOR_PWM_MAX;

    if (temp > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, temp);
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, -temp);
    }
}

/* ==================== application layer ==================== */

void App_Motor_Init(void)
{
    Motor_Init();
    Encoder_Init();
    extern TIM_HandleTypeDef htim6;
    HAL_TIM_Base_Start_IT(&htim6);
}

void App_Motor_Run(void)
{
    Wheel_A.RT = (float)(Encoder_A_GetCounter() * MEC_WHEEL_SCALE);
    Wheel_B.RT = (float)(Encoder_B_GetCounter() * MEC_WHEEL_SCALE);
    Wheel_C.RT = (float)(Encoder_C_GetCounter() * MEC_WHEEL_SCALE);
    Wheel_D.RT = (float)(Encoder_D_GetCounter() * MEC_WHEEL_SCALE);

    Wheel_A.PWM = SPEED_PidCtlA( Wheel_A.TG,  Wheel_A.RT);
    Wheel_B.PWM = SPEED_PidCtlB(-Wheel_B.TG,  Wheel_B.RT);
    Wheel_C.PWM = SPEED_PidCtlC( Wheel_C.TG,  Wheel_C.RT);
    Wheel_D.PWM = SPEED_PidCtlD(-Wheel_D.TG,  Wheel_D.RT);

    MOTOR_A_SetSpeed(Wheel_A.PWM);
    MOTOR_B_SetSpeed(Wheel_B.PWM);
    MOTOR_C_SetSpeed(Wheel_C.PWM);
    MOTOR_D_SetSpeed(Wheel_D.PWM);
}

void motor_speed_set(float A, float B, float C, float D)
{
    Wheel_A.TG = A;
    Wheel_B.TG = B;
    Wheel_C.TG = C;
    Wheel_D.TG = D;
}

/* ==================== timer interrupt callback ==================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Make sure we only handle TIM6 here.
    // (If HAL_TIM_PeriodElapsedCallback is already defined elsewhere, e.g. in stm32f4xx_it.c, 
    // we would put this logic there, but it is safe to define it here if unused).
    if (htim->Instance == TIM6)
    {
        App_Motor_Run();
    }
}
