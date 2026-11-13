/**
  ******************************************************************************
  * @file    tracking.c
  * @brief   五路 TCRT5000 循迹：采集 -> 去抖 -> 加权求偏差
  *
  * 位置权重（从左到右）：
  *   OUT1    OUT2    OUT3    OUT4    OUT5
  *   -20     -10      0      +10     +20
  *
  * 偏差 error 的物理含义（决定转向方向）：
  *   error < 0  黑线偏左  -> 车偏右  -> 需要左转（左轮慢、右轮快）
  *   error > 0  黑线偏右  -> 车偏左  -> 需要右转（左轮快、右轮慢）
  ******************************************************************************
  */

#include "main.h"
#include "car_config.h"
#include "tracking.h"

/* Private defines -----------------------------------------------------------*/
#define TRACK_CHANNELS     5U
/* 十字/横线判定：只有 5 路**全部**压线才算。
 * 现在直接设成 6（= 永远不触发），因为实测发现黑线比探头跨度还宽时，
 * 正常行驶就会 5 路全亮，被判成"十字"后强制直行 —— 表现就是
 * "明明偏离了却一直往前开，不修正"。
 * 关掉以后所有读数都走正常的加权修正逻辑，全黑时的处理交给 main.c
 * （沿用最后一次有效方向继续修正）。 */
#define TRACK_CROSS_COUNT  6U
#define TRACK_WEIGHT_MAX   20      /* 最外侧传感器的权重绝对值 */

/* 脱线迟滞：连续这么多帧全白才真的判定为"脱线"。
 * **传感器优先**：压到 2 帧（20ms）就够滤掉单帧毛刺，真脱线时也能最快反应，
 * 不会因为迟迟不判脱线而让车继续往前冲。 */
#define TRACK_LOST_CONFIRM 2U

/* Private variables ---------------------------------------------------------*/
static const int16_t s_weight[TRACK_CHANNELS] = { -20, -10, 0, 10, 20 };

/* 5 路传感器的端口/引脚表，顺序固定：0 = OUT1(最左) .. 4 = OUT5(最右) */
static GPIO_TypeDef *const s_port[TRACK_CHANNELS] =
{
  CAR_S1_PORT, CAR_S2_PORT, CAR_S3_PORT, CAR_S4_PORT, CAR_S5_PORT
};
static const uint16_t s_pin[TRACK_CHANNELS] =
{
  CAR_S1_PIN, CAR_S2_PIN, CAR_S3_PIN, CAR_S4_PIN, CAR_S5_PIN
};

/* 当前生效的"黑线电平"。上电可由 Track_AutoPolarity() 自动改写 */
static GPIO_PinState s_black_level = (GPIO_PinState)CAR_SENSOR_BLACK_LEVEL;

static TrackSample s_sample;
static uint8_t     s_candidate    = 0xFFU;   /* 正在等待确认的原始组合 */
static uint8_t     s_stable_frames = 0U;     /* 该组合已连续出现的帧数 */
static uint8_t     s_lost_frames   = 0U;     /* 已经连续多少帧全白（脱线迟滞用） */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  读一路传感器，返回 1 表示压到黑线
  */
static uint8_t sensor_black(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == s_black_level) ? 1U : 0U;
}

/**
  * @brief  一次性把 5 路电平读成 5 bit 组合
  * @note   5 路全部在 GPIOB：OUT1=PB6, OUT2=PB8, OUT3=PB7, OUT4=PB15, OUT5=PB5
  *         （OUT3 已确认接 PB7；不再兼容 PB3——那个脚悬空时会被干扰读到低电平，
  *           导致中间那一路永远"压线"，车在白纸上也一直跑）
  */
static uint8_t read_pattern(void)
{
  uint8_t pattern = 0U;
  uint8_t i;

  for (i = 0U; i < TRACK_CHANNELS; i++)
  {
    if (sensor_black(s_port[i], s_pin[i]) != 0U)
    {
      pattern = (uint8_t)(pattern | (uint8_t)(1U << i));
    }
  }

  return pattern;
}

/* Exported functions --------------------------------------------------------*/

void Track_Init(void)
{
  uint8_t i;

  /* 上电先直接采一次，避免第一帧把"还没读到"误判成脱线而原地转圈 */
  s_candidate      = read_pattern();
  s_sample.pattern = s_candidate;
  s_stable_frames  = CAR_SENSOR_DEBOUNCE;

  s_sample.count = 0U;
  s_sample.error = 0;
  s_sample.corner = 0U;
  s_sample.corner_dir = 0;
  s_sample.state = TRACK_STATE_LOST;
  for (i = 0U; i < TRACK_CHANNELS; i++)
  {
    s_sample.bits[i] = 0U;
  }

  /* 立刻按实测值算一帧，让误差和状态在进入主循环前就是正确的 */
  Track_Update();
}

void Track_Update(void)
{
  uint8_t pattern;
  uint8_t count  = 0U;
  int32_t sum    = 0;
  uint8_t i;

  pattern = read_pattern();

  /* 去抖：连续 CAR_SENSOR_DEBOUNCE 帧一致才认可，抑制黑白交界处的抖动 */
  if (pattern == s_candidate)
  {
    if (s_stable_frames < 0xFFU)
    {
      s_stable_frames++;
    }
  }
  else
  {
    s_candidate     = pattern;
    s_stable_frames = 1U;
  }

  if (s_stable_frames >= CAR_SENSOR_DEBOUNCE)
  {
    s_sample.pattern = pattern;
  }

  /* 先只统计，不写结果：全白时还要经过脱线迟滞才决定认不认 */
  for (i = 0U; i < TRACK_CHANNELS; i++)
  {
    if (((s_sample.pattern >> i) & 0x01U) != 0U)
    {
      count++;
      sum += (CAR_SENSOR_ORDER_SIGN > 0) ? s_weight[i] : -s_weight[i];
    }
  }

  if (count == 0U)
  {
    if (s_lost_frames < 0xFFU)
    {
      s_lost_frames++;
    }

    if (s_lost_frames < TRACK_LOST_CONFIRM)
    {
      /* 短暂全白（过缝 / 边界抖动）：保持上一次的有效状态，
       * 让上层继续按原方向修正，不要触发倒车找线 */
      return;
    }
  }
  else
  {
    s_lost_frames = 0U;
  }

  /* 到这里才写结果 */
  for (i = 0U; i < TRACK_CHANNELS; i++)
  {
    s_sample.bits[i] = (uint8_t)((s_sample.pattern >> i) & 0x01U);
  }
  s_sample.count = count;

  /* 归一化到 -100 ~ +100：全部偏一侧时就是 ±100，便于直接乘增益 */
  if (count == 0U)
  {
    s_sample.error = 0;
  }
  else
  {
    s_sample.error = (int16_t)((sum * 100) / ((int32_t)count * TRACK_WEIGHT_MAX));
  }

  /* 路况判定 */
  if (count == 0U)
  {
    s_sample.state = TRACK_STATE_LOST;
  }
  else if (count >= TRACK_CROSS_COUNT)
  {
    s_sample.state = TRACK_STATE_CROSS;
  }
  else
  {
    s_sample.state = TRACK_STATE_GO;
  }

  /* 直角弯判定：同一侧连续两路以上压线。
   *   左侧 = OUT1 + OUT2        右侧 = OUT4 + OUT5
   * 用"哪侧路数多"来判方向，两侧完全对称；
   * 之前写成 if/else if（先判左侧），遇到两侧各两路时永远判成左转，右侧被吃掉。
   * 全黑（count 到 5）不参与，因为那时左右都满足、判不出方向。 */
  if ((count != 0U) && (count < TRACK_CHANNELS))
  {
    uint8_t left_n  = (uint8_t)(s_sample.bits[0] + s_sample.bits[1]);
    uint8_t right_n = (uint8_t)(s_sample.bits[3] + s_sample.bits[4]);

    if ((left_n >= 2U) || (right_n >= 2U))
    {
      s_sample.corner = 1U;

      if (left_n > right_n)
      {
        s_sample.corner_dir = -1;        /* 线在左侧 -> 左转 */
      }
      else if (right_n > left_n)
      {
        s_sample.corner_dir = 1;         /* 线在右侧 -> 右转 */
      }
      else
      {
        /* 两侧各两路（畸形情况）：按当前偏差方向走 */
        s_sample.corner_dir = (s_sample.error > 0) ? 1 : -1;
      }
    }
    else
    {
      s_sample.corner     = 0U;
      s_sample.corner_dir = 0;
    }
  }
  else
  {
    s_sample.corner     = 0U;
    s_sample.corner_dir = 0;
  }
}

const TrackSample *Track_Get(void)
{
  return &s_sample;
}

GPIO_PinState Track_AutoPolarity(uint16_t samples, uint16_t interval_ms)
{
  uint32_t pressed = 0U;
  uint32_t total   = 0U;
  uint16_t n;
  uint8_t  i;

  if (samples == 0U)
  {
    samples = 1U;
  }

  for (n = 0U; n < samples; n++)
  {
    for (i = 0U; i < TRACK_CHANNELS; i++)
    {
      if (HAL_GPIO_ReadPin(s_port[i], s_pin[i]) == s_black_level)
      {
        pressed++;
      }
      total++;
    }
    HAL_Delay(interval_ms);
  }

  /* 压线率超过 60%：赛道背景不可能有这么大片黑，说明极性反了，翻转 */
  if ((total != 0U) && ((pressed * 100U) > (total * 60U)))
  {
    s_black_level = (s_black_level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }

  s_lost_frames = 0U;
  Track_Update();   /* 用最终极性立刻重算一帧 */

  return s_black_level;
}

void Track_SetBlackLevel(GPIO_PinState level)
{
  s_black_level = level;
}

GPIO_PinState Track_GetBlackLevel(void)
{
  return s_black_level;
}
