#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"

/* -------------------- chassis parameters -------------------- */
#define PI                      3.14159265358979f

#define PID_RATE                100

#define WHEEL_RESOLUTION        1040.0f
#define MEC_WHEEL_BASE          0.128f
#define MEC_ACLE_BASE           0.146f
#define MEC_WHEEL_DIAMETER      0.048f
#define MEC_WHEEL_SCALE         (PI * MEC_WHEEL_DIAMETER * PID_RATE / WHEEL_RESOLUTION)

/* -------------------- PWM limits -------------------- */
#define MOTOR_PWM_MAX           2000

/*
 * PWM period (ARR) — must match the CubeMX value for TIM1/TIM9/TIM12.
 * With ARR=2000 and correct PSC at 168 MHz, f_PWM ≈ 21 kHz.
 */
#define MOTOR_TIM_ARR           2000

/* -------------------- wheel data structure -------------------- */
typedef struct
{
    int16_t CNT_RT;
    double  RT;
    float   TG;
    int16_t PWM;
} ROBOT_Wheel;

extern ROBOT_Wheel Wheel_A, Wheel_B, Wheel_C, Wheel_D;

/* -------------------- motor init & control -------------------- */
void Motor_Init(void);
void Encoder_Init(void);

int16_t SPEED_PidCtlA(float spd_target, float spd_current);
int16_t SPEED_PidCtlB(float spd_target, float spd_current);
int16_t SPEED_PidCtlC(float spd_target, float spd_current);
int16_t SPEED_PidCtlD(float spd_target, float spd_current);

void MOTOR_A_SetSpeed(int16_t speed);
void MOTOR_B_SetSpeed(int16_t speed);
void MOTOR_C_SetSpeed(int16_t speed);
void MOTOR_D_SetSpeed(int16_t speed);

int16_t Encoder_A_GetCounter(void);
int16_t Encoder_B_GetCounter(void);
int16_t Encoder_C_GetCounter(void);
int16_t Encoder_D_GetCounter(void);

/* -------------------- application layer -------------------- */
void App_Motor_Init(void);
void App_Motor_Run(void);
void motor_speed_set(float A, float B, float C, float D);

extern float motor_kp;
extern float motor_ki;
extern float motor_kd;

#endif /* __MOTOR_H */
