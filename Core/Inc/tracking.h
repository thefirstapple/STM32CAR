/**
  ******************************************************************************
  * @file    tracking.h
  * @brief   五路 TCRT5000 循迹传感器采集与偏差计算
  ******************************************************************************
  */

#ifndef __TRACKING_H
#define __TRACKING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 传感器阵列当前看到的路况 */
typedef enum
{
  TRACK_STATE_GO = 0,   /*!< 正常压线行驶 */
  TRACK_STATE_CROSS,    /*!< 4~5 路同时压线：十字路口或横向黑线 */
  TRACK_STATE_LOST      /*!< 一路都没压到：脱线 */
} TrackState;

/** @brief 一帧采样结果，全部字段都可以在调试器里直接看 */
typedef struct
{
  uint8_t    bits[5];   /*!< [0]=OUT1(最左) .. [4]=OUT5(最右)，1 = 压到黑线 */
  uint8_t    pattern;   /*!< bit0..bit4 对应 OUT1..OUT5 的原始组合 */
  uint8_t    count;     /*!< 压线传感器数量 0~5 */
  int16_t    error;     /*!< -100 = 线在最左，0 = 正中，+100 = 线在最右 */
  uint8_t    corner;    /*!< 1 = 同一侧连续两路以上压线，判定为直角轨道 */
  int8_t     corner_dir;/*!< 直角弯方向：-1 = 线在左侧要左转，+1 = 线在右侧要右转 */
  TrackState state;
} TrackSample;

/** @brief 清空内部状态，上电调用一次即可 */
void Track_Init(void);

/** @brief 采样一次并更新偏差，按控制周期调用 */
void Track_Update(void);

/** @brief 取最近一帧结果 */
const TrackSample *Track_Get(void);

/**
  * @brief  上电自动识别"黑线对应哪个电平"
  * @param  samples      采样次数
  * @param  interval_ms  每次间隔
  * @retval GPIO_PIN_SET / GPIO_PIN_RESET
  * @note   判据：采 5 路 x N 次，统计"被判成压线"的比例。
  *         正常情况（车在赛道或白纸上），压线率不会超过 60%；
  *         如果超过 60%，说明当前极性把"背景"当成了黑线，直接翻转。
  *         这样无论你是白底黑线还是相反极性的模块，都能自动对上。
  */
GPIO_PinState Track_AutoPolarity(uint16_t samples, uint16_t interval_ms);

/** @brief 手动指定黑线电平（自检或调试用） */
void Track_SetBlackLevel(GPIO_PinState level);

/** @brief 读取当前生效的黑线电平 */
GPIO_PinState Track_GetBlackLevel(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRACKING_H */
