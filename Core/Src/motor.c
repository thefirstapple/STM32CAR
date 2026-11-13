/**
  ******************************************************************************
  * @file    motor.c
  * @brief   TB6612FNG 双电机驱动 —— TIM2 两路硬件 PWM 调速
  *
  * 接线（TB6612 -> STM32）：
  *   PWMA -> PA2 (TIM2_CH3)      AIN1 -> PA4     AIN2 -> PA5
  *   PWMB -> PA1 (TIM2_CH2)      BIN1 -> PA6     BIN2 -> PA7
  *   STBY -> 3.3V（或 PB0 拉高）  VM -> 电池正极   VCC -> 3.3V    GND 共地
  *
  * 真值表（每路）：
  *   IN1  IN2  PWM    结果
  *    1    0   有     正转
  *    0    1   有     反转
  *    0    0   -      停止（输出断开，轮子滑行）
  *    1    1   -      刹车（输出短接，快速制动）
  *   STBY = 0         整片待机，电机全部停止
  ******************************************************************************
  */

#include "main.h"
#include "car_config.h"
#include "motor.h"

/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef s_htim2;      /* TIM2：两路 PWM 共用一个定时器，频率天然同步 */
static uint16_t s_duty_left  = 0U;
static uint16_t s_duty_right = 0U;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  取得 TIM2 的计数时钟
  * @note   TIM2 挂在 APB1 上：APB1 分频系数不为 1 时，定时器时钟 = PCLK1 x 2
  */
static uint32_t pwm_timer_clock(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

  if (HAL_RCC_GetHCLKFreq() != pclk1)
  {
    return pclk1 * 2U;                 /* 本工程 72 MHz / 2 = 36 MHz -> 定时器 72 MHz */
  }
  return pclk1;
}

/**
  * @brief  把 PA1 / PA2 从普通 GPIO 改成 TIM2 复用推挽输出
  * @note   MX_GPIO_Init() 已经把 PA2 配成普通输出了，所以这里必须先 DeInit
  */
static void pwm_gpio_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_DeInit(CAR_PWM_GPIO_PORT, CAR_PWMA_PIN | CAR_PWMB_PIN);

  gpio.Pin   = CAR_PWMA_PIN | CAR_PWMB_PIN;
  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CAR_PWM_GPIO_PORT, &gpio);
}

/**
  * @brief  把逻辑速度(0~100)换算成实际 PWM 占空比，跳过电机死区
  * @note   真实直流电机在占空比低于 CAR_PWM_DEADZONE 时根本不转。
  *         不做补偿的话，"内侧轮 15%" 其实是停转的，差速变成单轮驱动，
  *         转向过冲、来回抖，就是循迹抖动的常见根源。
  */
static uint16_t duty_from_speed(uint16_t speed)
{
  if (speed == 0U)
  {
    return 0U;
  }

  return (uint16_t)(CAR_PWM_DEADZONE +
                    ((uint32_t)speed * (uint32_t)(CAR_PWM_STEPS - CAR_PWM_DEADZONE))
                      / CAR_PWM_STEPS);
}

/**
  * @brief  设置单路电机的方向和占空比
  * @param  in1_port/in1_pin  方向脚 1
  * @param  in2_port/in2_pin  方向脚 2
  * @param  channel           TIM2 通道
  * @param  duty              -100 ~ +100
  * @retval 逻辑速度绝对值（未补偿）
  */
static uint16_t motor_drive(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                            GPIO_TypeDef *in2_port, uint16_t in2_pin,
                            uint32_t channel, int16_t duty)
{
  uint16_t magnitude;

  if (duty > (int16_t)CAR_PWM_STEPS)
  {
    duty = (int16_t)CAR_PWM_STEPS;
  }
  else if (duty < -(int16_t)CAR_PWM_STEPS)
  {
    duty = -(int16_t)CAR_PWM_STEPS;
  }

  if (duty > 0)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    magnitude = (uint16_t)duty;
  }
  else if (duty < 0)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    magnitude = (uint16_t)(-duty);
  }
  else
  {
    /* 占空比为 0：两脚同时拉低，H 桥断开，电机滑行 */
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    magnitude = 0U;
  }

  /* 死区补偿后再写进比较寄存器 */
  __HAL_TIM_SET_COMPARE(&s_htim2, channel, (uint32_t)duty_from_speed(magnitude));

  return magnitude;
}

/* Exported functions --------------------------------------------------------*/

void Motor_Init(void)
{
  TIM_OC_InitTypeDef oc = {0};
  uint32_t tim_clk;
  uint32_t prescaler;

  __HAL_RCC_TIM2_CLK_ENABLE();
  pwm_gpio_init();

  /* 计数频率 = 目标 PWM 频率 x 分辨率级数；PSC 取整即可 */
  tim_clk   = pwm_timer_clock();
  prescaler = tim_clk / ((uint32_t)CAR_PWM_FREQ_HZ * CAR_PWM_STEPS);
  if (prescaler < 1U)
  {
    prescaler = 1U;
  }
  if (prescaler > 0x10000U)
  {
    prescaler = 0x10000U;
  }

  s_htim2.Instance               = TIM2;
  s_htim2.Init.Prescaler         = prescaler - 1U;
  s_htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  s_htim2.Init.Period            = (uint32_t)CAR_PWM_STEPS - 1U;
  s_htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  s_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&s_htim2) != HAL_OK)
  {
    Error_Handler();
  }

  oc.OCMode     = TIM_OCMODE_PWM1;
  oc.Pulse      = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&s_htim2, &oc, CAR_PWMA_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&s_htim2, &oc, CAR_PWMB_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&s_htim2, CAR_PWMA_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&s_htim2, CAR_PWMB_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  Motor_Stop();

#if CAR_DRIVE_STBY_PIN
  HAL_GPIO_WritePin(CAR_STBY_PORT, CAR_STBY_PIN, GPIO_PIN_SET);   /* 拉高 STBY 使能驱动 */
#endif
}

void Motor_SetSpeed(int16_t left, int16_t right)
{
#if CAR_SWAP_LEFT_RIGHT
  /* 左右电机接反：把两条通道交换，所有转向方向一次性纠正 */
  {
    int16_t tmp = left;
    left  = right;
    right = tmp;
  }
#endif

  /* 方向修正：装配反了只需要改 car_config.h 里的 SIGN，不用动接线 */
  int16_t l = (CAR_LEFT_MOTOR_SIGN > 0) ? left  : (int16_t)(-left);
  int16_t r = (CAR_RIGHT_MOTOR_SIGN > 0) ? right : (int16_t)(-right);

  s_duty_left  = motor_drive(CAR_BIN1_PORT, CAR_BIN1_PIN,
                             CAR_BIN2_PORT, CAR_BIN2_PIN,
                             CAR_PWMB_CHANNEL, l);
  s_duty_right = motor_drive(CAR_AIN1_PORT, CAR_AIN1_PIN,
                             CAR_AIN2_PORT, CAR_AIN2_PIN,
                             CAR_PWMA_CHANNEL, r);
}

void Motor_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&s_htim2, CAR_PWMB_CHANNEL, 0U);
  __HAL_TIM_SET_COMPARE(&s_htim2, CAR_PWMA_CHANNEL, 0U);

  HAL_GPIO_WritePin(CAR_BIN1_PORT, CAR_BIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAR_BIN2_PORT, CAR_BIN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAR_AIN1_PORT, CAR_AIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAR_AIN2_PORT, CAR_AIN2_PIN, GPIO_PIN_RESET);

  s_duty_left  = 0U;
  s_duty_right = 0U;
}

void Motor_Brake(void)
{
  /* 两脚同时为高 -> 下桥臂短接电机，制动 */
  HAL_GPIO_WritePin(CAR_BIN1_PORT, CAR_BIN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CAR_BIN2_PORT, CAR_BIN2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CAR_AIN1_PORT, CAR_AIN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CAR_AIN2_PORT, CAR_AIN2_PIN, GPIO_PIN_SET);

  __HAL_TIM_SET_COMPARE(&s_htim2, CAR_PWMB_CHANNEL, 0U);
  __HAL_TIM_SET_COMPARE(&s_htim2, CAR_PWMA_CHANNEL, 0U);

  s_duty_left  = 0U;
  s_duty_right = 0U;
}

uint16_t Motor_GetLeftDuty(void)
{
  return s_duty_left;
}

uint16_t Motor_GetRightDuty(void)
{
  return s_duty_right;
}
