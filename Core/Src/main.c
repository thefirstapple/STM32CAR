/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "car_config.h"
#include "motor.h"
#include "tracking.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* 下面的 volatile 变量专门留给调试器观察（VSCode/Keil 的 Watch 窗口直接输名字） */
volatile uint8_t  g_car_pattern  = 0U;   /* 5 路传感器原始组合，bit0=OUT1 .. bit4=OUT5 */
volatile int16_t  g_car_error    = 0;    /* 偏差 -100 ~ +100，正数表示线在右边 */
volatile uint8_t  g_car_state    = 0U;   /* 0=正常压线 1=十字/横线 2=脱线 */
volatile int16_t  g_car_left     = 0;    /* 左轮输出 -100 ~ +100 */
volatile int16_t  g_car_right    = 0;    /* 右轮输出 -100 ~ +100 */
volatile uint32_t g_car_loops    = 0U;   /* 控制循环执行次数，不增长说明程序卡死 */
volatile uint8_t  g_car_phase    = 0U;   /* 0=正常 1=脱线惯性 2=正向搜索 3=反向搜索 4=放弃停车 */

static uint32_t s_ctrl_tick    = 0U;     /* 控制节拍基准 */
static uint8_t  s_lost_active  = 0U;     /* 是否处于脱线恢复流程 */
static uint32_t s_lost_start   = 0U;     /* 本次脱线开始的时刻 */
static int16_t  s_lost_dir     = 1;      /* 搜索方向：+1 往右扫，-1 往左扫 */
static int16_t  s_err_filt     = 0;      /* 低通滤波后的偏差，抑制抖动 */
static int16_t  s_err_prev     = 0;      /* 上一帧滤波偏差，用于微分项（探头前置补偿） */
static int16_t  s_last_left    = 0;      /* 脱线前的左轮输出，惯性段沿用 */
static int16_t  s_last_right   = 0;      /* 脱线前的右轮输出 */
static int16_t  s_last_error   = 0;      /* 最近一次有效偏差，供调试观察 */
static uint8_t  s_cross_active = 0U;     /* 是否正在直行通过十字 */
static uint32_t s_cross_start  = 0U;     /* 开始压十字的时刻 */
static uint8_t  s_cross_timeout = 0U;    /* 本次压十字已超时，锁存后转入脱线恢复 */
static uint8_t  s_corner_active = 0U;    /* 是否正在执行直角转弯 */
static uint32_t s_corner_start  = 0U;    /* 直角转弯开始时刻 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Car_Init(void);
static void Car_ControlStep(void);
#if (CAR_SELFTEST_SPIN_MS > 0U)
static void Car_SpinCheck(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  限幅
  */
static int16_t clamp_speed(int32_t value, int16_t limit)
{
  if (value > (int32_t)limit)
  {
    return limit;
  }
  if (value < -(int32_t)limit)
  {
    return (int16_t)(-limit);
  }
  return (int16_t)value;
}

/**
  * @brief  上电轮子自检（三段），用来定位"方向反"和"某一路不转"
  * @note   1) 两轮同时小幅度前进 -> 看车头是往前还是往后
  *            往后 => 车轮前后装反，改 CAR_LEFT/RIGHT_MOTOR_SIGN
  *         2) 只转左轮 -> 验证 PWMB(PA1) 这一路通不通
  *         3) 只转右轮 -> 验证 PWMA(PA2) 这一路通不通
  */
#if (CAR_SELFTEST_SPIN_MS > 0U)
static void Car_SpinCheck(void)
{
  Motor_SetSpeed(30, 30);                     /* 1) 两轮一起前进 */
  HAL_Delay(CAR_SELFTEST_SPIN_MS);
  Motor_Stop();
  HAL_Delay(150U);

  Motor_SetSpeed(30, 0);                      /* 2) 只有左轮（BIN / PWMB） */
  HAL_Delay(CAR_SELFTEST_SPIN_MS);
  Motor_Stop();
  HAL_Delay(150U);

  Motor_SetSpeed(0, 30);                      /* 3) 只有右轮（AIN / PWMA） */
  HAL_Delay(CAR_SELFTEST_SPIN_MS);
  Motor_Stop();
  HAL_Delay(150U);
}
#endif

/**
  * @brief  小车初始化：状态灯 -> 循迹阵列(+自动极性) -> 电机(+轮子自检)
  */
static void Car_Init(void)
{
#if CAR_USE_STATUS_LED
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;
#endif

#if CAR_USE_STATUS_LED
  /* 板载 PC13 LED（低电平点亮）。.ioc 里没配置这个脚，这里自行初始化。
   * 最先初始化它，后面每一步的状态都能立刻从灯上看出来 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  gpio.Pin   = GPIO_PIN_13;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);

  /* ===== 固件版本指示：快闪 3 次 = 本版 =====
   * 用来确认板子里烧进去的确实是这次编译出来的程序。
   * 看不到这 3 次闪烁，说明烧的还是旧 hex，后面调参数都没有意义。 */
  for (i = 0U; i < 3U; i++)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(100U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(200U);
  }
  HAL_Delay(800U);           /* 停一下，和后面的初始化完成提示区分开 */
#endif

  /* 5 路输入统一改成内部上拉：本车黑线对应低电平，
   * 那么"没接线/悬空的脚"会被上拉成高 = 不压线，不会误触发。
   * 注意：必须先配好 GPIO，再 Track_Init()，否则第一帧读到的是未配置的电平。 */
  {
    GPIO_InitTypeDef gpio_in = {0};

    gpio_in.Mode = GPIO_MODE_INPUT;
    gpio_in.Pull = CAR_SENSOR_PULL;

    gpio_in.Pin = CAR_S1_PIN; HAL_GPIO_Init(CAR_S1_PORT, &gpio_in);
    gpio_in.Pin = CAR_S2_PIN; HAL_GPIO_Init(CAR_S2_PORT, &gpio_in);
    gpio_in.Pin = CAR_S3_PIN; HAL_GPIO_Init(CAR_S3_PORT, &gpio_in);
    gpio_in.Pin = CAR_S4_PIN; HAL_GPIO_Init(CAR_S4_PORT, &gpio_in);
    gpio_in.Pin = CAR_S5_PIN; HAL_GPIO_Init(CAR_S5_PORT, &gpio_in);

    /* PB3（.ioc 里被重复标成 OUT3 的另一个脚）已经确认不用了。
     * 悬空的输入脚会被电机干扰耦合噪声，这里拉成上拉输入，避免它乱跳。 */
    gpio_in.Pin = OUT3_Pin;   HAL_GPIO_Init(OUT3_GPIO_Port, &gpio_in);
  }

  Track_Init();

#if CAR_AUTO_POLARITY
  /* 自动校准极性：约 0.4 秒。
   * 判据是"压线率"——正常赛道背景不可能有大片黑，
   * 如果采样时超过 60% 的读数都是"压线"，说明极性反了，自动翻转。 */
  (void)Track_AutoPolarity(40U, 10U);
#endif

#if CAR_SENSOR_SELFTEST
  /* 自检模式：不初始化电机，车绝不会动。
   * PC13 灯跟随传感器 —— 任意一路压到黑线灯亮，用来核对极性。 */
  for (;;)
  {
    Track_Update();

    g_car_pattern = Track_Get()->pattern;
    g_car_error   = Track_Get()->error;
    g_car_state   = (uint8_t)Track_Get()->state;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,
                      (Track_Get()->count > 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_Delay(20U);
  }
#endif

  Motor_Init();

#if (CAR_SELFTEST_SPIN_MS > 0U)
  Car_SpinCheck();
#endif

#if CAR_USE_STATUS_LED
  /* 初始化完成：长亮 500ms 一次（和上面的"版本 3 短闪"区分开） */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(500U);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
#endif

  s_ctrl_tick     = HAL_GetTick();
  s_lost_active   = 0U;
  s_cross_active  = 0U;
  s_cross_timeout = 0U;
  s_lost_dir      = 1;
  s_last_error    = 0;
  s_err_filt      = 0;
  s_err_prev      = 0;
  s_last_left     = 0;
  s_last_right    = 0;
  g_car_phase     = 0U;

  Motor_Stop();
}

/**
  * @brief  循迹控制核心：读传感器 -> 算差速 -> 输出电机
  * @note   每 CAR_CTRL_PERIOD_MS 调用一次
  */
static void Car_ControlStep(void)
{
  const TrackSample *s;
  uint32_t now = HAL_GetTick();
  int32_t  turn;
  int32_t  turn_limit;
  int16_t  base;
  int16_t  abs_err;
  int16_t  left    = 0;
  int16_t  right   = 0;
  uint8_t  brake   = 0U;
  uint8_t  recover = 0U;

  Track_Update();
  s = Track_Get();

  g_car_pattern = s->pattern;
  g_car_error   = s->error;
  g_car_state   = (uint8_t)s->state;
  g_car_loops++;

  if (s->state == TRACK_STATE_GO)
  {
    int16_t gain;
    int16_t d;
    int16_t err_use;

    s_lost_active  = 0U;
    s_cross_active = 0U;
    s_last_error   = s->error;

    /* ---------------- 直角弯：同一侧连续两路以上压线 ---------------- */
    if (s->corner != 0U)
    {
      /* 直角弯方向：corner_dir 表示"线在哪一侧"，再乘两个符号——
       *   CAR_STEER_DIRECTION  和单路循迹共用（保证左右约定一致）
       *   CAR_CORNER_DIR_SIGN  直角弯实测和单路相反，单独翻一次
       * 两侧速度现在都取 20，实测左侧 28 太快、右侧 20 正常。 */
      int16_t cdir   = (int16_t)(s->corner_dir * CAR_STEER_DIRECTION * CAR_CORNER_DIR_SIGN);
      int16_t cspeed = (s->corner_dir > 0) ? (int16_t)CAR_CORNER_SPEED_RIGHT
                                           : (int16_t)CAR_CORNER_SPEED;

      if (s_corner_active == 0U)
      {
        s_corner_active = 1U;
        s_corner_start  = now;
      }

      if ((now - s_corner_start) <= (uint32_t)CAR_CORNER_MAX_MS)
      {
        /* 原地转向：一侧正转、另一侧反转，转弯半径最小。
         * 一直转到 OUT3 重新压线（corner 条件不再成立）为止，
         * 然后自然退出这个分支，回到下面的正常修正。 */
        left  = (int16_t)(-cdir * cspeed);
        right = (int16_t)(cdir * cspeed);
        g_car_phase = 6U;

        /* 转弯过程中清掉滤波残值：转完一旦 OUT3 压线，误差应该是干净的 0，
         * 否则残留的旧偏差会让车在转完之后再多甩一下。 */
        s_err_filt = 0;
      }
      else
      {
        /* 转了 1.4 秒还没转回来：放弃，转去脱线恢复 */
        s_corner_active = 0U;
        recover = 1U;
      }
    }
    else
    {
      s_corner_active = 0U;
    }

    /* 只有"不在直角弯里"时才走正常循迹计算，否则会把上面的原地转向覆盖掉 */
    if ((recover == 0U) && (s->corner == 0U))
    {
      /* 直接使用传感器算出的偏差。
       * 注意：全黑（count=5）时加权平均是 0 -> 直行。
       * 之前这里"沿用最后一次方向"会把车钉在原地一直转，已去掉。 */
      err_use = s->error;

      /* 一阶低通：开关量传感器在两路之间跳变时，原始误差会 ±50 地跳，
       * 直接拿去做 P 控制就会左右摆。这里把跳变磨平，是抑制抖动的关键。 */
      s_err_filt = (int16_t)((((int32_t)s_err_filt * (100 - CAR_ERROR_FILTER))
                              + ((int32_t)err_use * CAR_ERROR_FILTER)) / 100);

      abs_err = (s_err_filt >= 0) ? s_err_filt : (int16_t)(-s_err_filt);

    /* 死区：滤波后误差很小就直行，避免中间那一路来回跳引起抖动 */
    if (abs_err < (int16_t)CAR_STEER_DEADBAND)
    {
      turn = 0;
      base = (int16_t)CAR_BASE_SPEED;
    }
    else
    {
      /* 误差越大越慢：弯道里主动降速，转弯半径才压得住，不会冲出去 */
      base = (int16_t)((int32_t)CAR_BASE_SPEED -
                       ((int32_t)abs_err * (int32_t)CAR_TURN_SLOWDOWN) / 100);

      /* 增益调度：单路轻微偏离用柔和增益（平滑回中，不画龙），
       * 两路以上明显偏一侧用果断增益（直角弯要猛拐） */
      gain = (abs_err < (int16_t)CAR_SOFT_BAND) ? (int16_t)CAR_GAIN_SOFT
                                                : (int16_t)CAR_STEER_GAIN;

      /* 微分项：偏差变化越快，额外加越多转向量。
       * 探头装在车轴前方 45mm，是"提前看到"弯道的；
       * 有了这一项，车会在偏差刚出现时就先转，而不是等误差攒大再追。 */
      d = (int16_t)(s_err_filt - s_err_prev);
      if (d > (int16_t)CAR_D_LIMIT)
      {
        d = (int16_t)CAR_D_LIMIT;
      }
      else if (d < -(int16_t)CAR_D_LIMIT)
      {
        d = -(int16_t)CAR_D_LIMIT;
      }

      turn = (((int32_t)s_err_filt * gain * CAR_STEER_DIRECTION)
              + ((int32_t)d * CAR_STEER_KD * CAR_STEER_DIRECTION)) / 100;
    }

    s_err_prev = s_err_filt;
    if (base < (int16_t)CAR_MIN_SPEED)
    {
      base = (int16_t)CAR_MIN_SPEED;
    }

    /* 转向力度上限：小误差温和（内轮不反转），大误差放开（内轮可反转，急弯才转得过来） */
#if (CAR_ALLOW_INNER_REVERSE == 0)
    turn_limit = (int32_t)base - (int32_t)CAR_MIN_SPEED;
#else
    turn_limit = (abs_err >= (int16_t)CAR_HARD_TURN_ERROR)
                   ? ((int32_t)base + (int32_t)CAR_MIN_SPEED)
                   : ((int32_t)base - (int32_t)CAR_MIN_SPEED);
#endif
    if (turn_limit < 0)
    {
      turn_limit = 0;
    }
    if (turn > turn_limit)
    {
      turn = turn_limit;
    }
    if (turn < -turn_limit)
    {
      turn = -turn_limit;
    }

    left  = clamp_speed((int32_t)base + turn, CAR_MAX_SPEED);
    right = clamp_speed((int32_t)base - turn, CAR_MAX_SPEED);

    /* 内侧轮保底正速度：完全停转的轮子容易被线拖住 */
    if ((left > 0) && (left < (int16_t)CAR_MIN_SPEED))
    {
      left = (int16_t)CAR_MIN_SPEED;
    }
    if ((right > 0) && (right < (int16_t)CAR_MIN_SPEED))
    {
      right = (int16_t)CAR_MIN_SPEED;
    }

      g_car_phase = 0U;

      s_last_left  = left;
      s_last_right = right;
    }
  }
  else if (s->state == TRACK_STATE_CROSS)
  {
    if (s_cross_active == 0U)
    {
      s_cross_active  = 1U;
      s_cross_start   = now;
      s_cross_timeout = 0U;
    }

    if ((s_cross_timeout == 0U) && ((now - s_cross_start) <= (uint32_t)CAR_CROSS_MAX_MS))
    {
      /* 十字 / 横向黑线：低速直行通过（速度必须低，否则车头会冲过拐角，
       * 红外探头越过垂直线就再也看不到轨道了） */
      s_lost_active = 0U;
      left  = (int16_t)CAR_CROSS_SPEED;
      right = (int16_t)CAR_CROSS_SPEED;

      g_car_phase = 0U;

      s_last_left  = left;
      s_last_right = right;
    }
    else
    {
      /* 压在大片黑上超过时限：锁存后转入脱线恢复（直角弯、T 字路口靠这段处理）。
       * 锁存是必要的，否则每帧都会重新计时，变成"直行 400ms -> 恢复 1 帧 -> 又直行"的死循环。 */
      s_cross_timeout = 1U;
      recover = 1U;
    }
  }
  else
  {
    /* 五路全白：彻底脱线 */
    s_cross_active  = 0U;
    s_cross_timeout = 0U;
    recover = 1U;
  }

  if (recover != 0U)
  {
    /* ---------- 没识别到黑线：短暂惯性后刹车停住 ---------- */
    uint32_t t;

    if (s_lost_active == 0U)
    {
      s_lost_active = 1U;
      s_lost_start  = now;
      /* 刚脱线，滤波状态无意义，直接清掉避免带进下一段循迹 */
      s_err_filt    = 0;
    }

    t = now - s_lost_start;

    if (t < (uint32_t)CAR_LOST_COAST_MS)
    {
      /* 惯性：按脱线前的方向再走一点点。
       * 处理"瞬间过缝"这类短暂丢线——多数情况这一小段就能重新压回线上，
       * 不用立刻急刹，避免车一顿一顿。 */
      left  = (int16_t)(((int32_t)s_last_left  * 70) / 100);
      right = (int16_t)(((int32_t)s_last_right * 70) / 100);
      g_car_phase = 1U;
    }
    else
    {
      /* 确认丢线：刹车停住，等人把车放回轨道。
       * 不再倒车/原地扫描——那样会在赛道上乱窜。 */
      brake = 1U;
      left  = 0;
      right = 0;
      g_car_phase = 5U;
    }
  }

  if (brake != 0U)
  {
    Motor_Brake();
  }
  else
  {
    Motor_SetSpeed(left, right);
  }

  g_car_left  = left;
  g_car_right = right;

#if CAR_USE_STATUS_LED
  /* 灯语（判断程序在干什么，最直接的窗口）：
   *   常亮       = 压到黑线，正常循迹
   *   快闪(100ms)= 4~5 路同时压线，正在直行通过十字
   *   慢闪(250ms)= 脱线，正在自己搜线（惯性/正向搜索/反向搜索）
   *   熄灭       = 搜索超时，已刹车放弃 */
  if (g_car_phase == 0U)
  {
    if (s->state == TRACK_STATE_CROSS)
    {
      if ((g_car_loops % 5U) == 0U)
      {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      }
    }
    else
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
  }
  else if (g_car_phase <= 4U)
  {
    if ((g_car_loops % 25U) == 0U)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  }
#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  Car_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* 固定 10 ms 节拍跑控制，Systick 的 1 ms 精度足够 */
    if ((now - s_ctrl_tick) >= (uint32_t)CAR_CTRL_PERIOD_MS)
    {
      if ((now - s_ctrl_tick) >= ((uint32_t)CAR_CTRL_PERIOD_MS * 5U))
      {
        s_ctrl_tick = now;                              /* 落后太多就重新对齐 */
      }
      else
      {
        s_ctrl_tick += (uint32_t)CAR_CTRL_PERIOD_MS;
      }

      Car_ControlStep();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
