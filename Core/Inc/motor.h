/**
  ******************************************************************************
  * @file    motor.h
  * @brief   TB6612FNG 双电机驱动（TIM2 硬件 PWM 调速）
  ******************************************************************************
  */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  初始化两路 PWM 与方向脚。必须在 MX_GPIO_Init() 之后调用。
  * @note   会把 PA1/PA2 从普通 GPIO 重新配置成 TIM2 复用推挽输出。
  */
void Motor_Init(void);

/**
  * @brief  设置左右轮速度
  * @param  left  左轮 -100 ~ +100（正数前进，负数后退，0 停止）
  * @param  right 右轮 -100 ~ +100
  */
void Motor_SetSpeed(int16_t left, int16_t right);

/** @brief 停止（H 桥断开，轮子滑行） */
void Motor_Stop(void);

/** @brief 刹车（H 桥上下管同时导通，电机短接制动，停车更干脆） */
void Motor_Brake(void);

/** @brief 读回当前输出占空比，调试用 */
uint16_t Motor_GetLeftDuty(void);
uint16_t Motor_GetRightDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
