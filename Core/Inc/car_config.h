/**
  ******************************************************************************
  * @file    car_config.h
  * @brief   循迹小车全部可调参数与引脚映射（只改这一个文件就能适配新接线）
  *
  * 硬件：STM32F103C8T6 + TB6612FNG + 5 路 TCRT5000 循迹模块
  *
  * 引脚来源：STM32CAR.ioc 标签 + 手写接线表
  *   PWMA(右)  PA2  —— 原 .ioc 里是普通 GPIO 输出，本工程把它改作 TIM2_CH3
  *   PWMB(左)  PA1  —— TIM2_CH2，.ioc 里原本空着，接线时要接到 PA1
  *   AIN1/AIN2 PA4/PA5  右电机方向（AO1/AO2）
  *   BIN1/BIN2 PA6/PA7  左电机方向（BO1/BO2）
  *   STBY      PB0(DIANJI)，高电平使能；若已直接接 3.3V 则此脚不接也正常
  *   OUT1..OUT5 PB6 / PB8 / PB7 / PB15 / PB5  循迹输入（对应 .ioc 的 OUT1/OUT2/OUT3B7/OUT4/OUT5）
  ******************************************************************************
  */

#ifndef __CAR_CONFIG_H
#define __CAR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*==============================================================================
 * 1. PWM 输出（TIM2，两路同步，20 kHz）
 *============================================================================*/

/* PWM 频率。20 kHz 超出人耳敏感范围，电机不会啸叫；若驱动板吃不消可降到 10000 */
#define CAR_PWM_FREQ_HZ           20000U

/* 占空比分辨率：0 ~ CAR_PWM_STEPS 对应 0 ~ 100% */
#define CAR_PWM_STEPS             100U

/* PWM 输出引脚（PA1 = TIM2_CH2，PA2 = TIM2_CH3） */
/* 若你把 PWMB 焊到了 PA3，把下面一行打开即可（TIM2_CH4） */
/* #define CAR_PWMB_USE_PA3 */
#define CAR_PWM_GPIO_PORT         GPIOA
#define CAR_PWMA_PIN              GPIO_PIN_2   /* 右电机 PWM */
#ifdef CAR_PWMB_USE_PA3
  #define CAR_PWMB_PIN            GPIO_PIN_3
  #define CAR_PWMB_CHANNEL        TIM_CHANNEL_4
#else
  #define CAR_PWMB_PIN            GPIO_PIN_1   /* 左电机 PWM */
  #define CAR_PWMB_CHANNEL        TIM_CHANNEL_2
#endif
#define CAR_PWMA_CHANNEL          TIM_CHANNEL_3

/*==============================================================================
 * 2. TB6612 方向控制脚
 *============================================================================*/
#define CAR_AIN1_PORT             AIN1_GPIO_Port
#define CAR_AIN1_PIN              AIN1_Pin
#define CAR_AIN2_PORT             AIN2_GPIO_Port
#define CAR_AIN2_PIN              AIN2_Pin
#define CAR_BIN1_PORT             BIN1_GPIO_Port
#define CAR_BIN1_PIN              BIN1_Pin
#define CAR_BIN2_PORT             BIN2_GPIO_Port
#define CAR_BIN2_PIN              BIN2_Pin

/* STBY：1 = 用 PB0 拉高使能，0 = STBY 已直接接 3.3V，代码不管这个脚 */
#define CAR_DRIVE_STBY_PIN        0
#define CAR_STBY_PORT             DIANJI_GPIO_Port
#define CAR_STBY_PIN              DIANJI_Pin

/* 电机装配方向修正：你这台实测是"给前进信号车却倒着走"，
 * 所以两个都取 -1。改完之后 Motor_SetSpeed 里 left/right 的含义
 * 仍然是"车的左轮/右轮往前跑多少"，转向逻辑不受影响。 */
#define CAR_LEFT_MOTOR_SIGN       (-1)
#define CAR_RIGHT_MOTOR_SIGN      (-1)

/* 左右电机接反修正。
 * 【怎么判断】上电自检有三段动作，看第二段（只驱动"左轮"）：
 *     转的是**左边**那个轮子 -> 保持 0
 *     转的是**右边**那个轮子 -> 说明 BIN 通道实际接的是右轮，改成 1
 *
 * ⚠️ 它和 CAR_STEER_DIRECTION 在数学上完全等价（都是交换左右轮），
 *    **两个不要同时改**，同时改会互相抵消，看起来就像"改了也没用"。
 *    正确做法：只调 CAR_STEER_DIRECTION 一个，这里保持 0。 */
#define CAR_SWAP_LEFT_RIGHT       0

/*==============================================================================
 * 3. 五路循迹传感器
 *============================================================================*/

/* 传感器检到黑线时 DO 输出的电平。
 * 【本车实测】黑线 = 模块输出**低**电平（白纸上输出高）。
 * 判据来自实测现象：按高电平判断时"白纸跑、黑线不跑" —— 白纸被当成压线了。
 *
 * 换模块或改接线后如果又出现"白纸跑、黑线不跑"，就把这一行翻一次：
 *   GPIO_PIN_RESET  <->  GPIO_PIN_SET
 * 只需要试这一次，两种情况必有一个对。 */
#define CAR_SENSOR_BLACK_LEVEL    GPIO_PIN_RESET

/* 5 路输入统一启用内部上拉。
 * 意义：黑线是低电平，那么"没接的脚 / 悬空脚"被上拉成高电平 = 不压线，
 * 不会因为某个脚悬空而误判成压线（这是"白纸上也一直跑"的另一个常见原因）。 */
#define CAR_SENSOR_PULL           GPIO_PULLUP

/* OUT1 在最左边、OUT5 在最右边时保持 +1；装反了改成 -1，权重会自动左右翻转 */
#define CAR_SENSOR_ORDER_SIGN     (+1)

/* 去抖帧数：连续 N 帧电平一致才采纳。
 * **传感器优先**，设成 1 = 每帧直接用最新读数。
 * 之前是 2，导致车在动的时候 pattern 每帧都在变，读数永远"凑不齐两帧相同"，
 * s_sample.pattern 一直停在旧值 —— 表现就是"明明压到线了却直走、不修正"。 */
#define CAR_SENSOR_DEBOUNCE      1U

#define CAR_S1_PORT               OUT1_GPIO_Port    /* PB6  */
#define CAR_S1_PIN                OUT1_Pin
#define CAR_S2_PORT               OUT2_GPIO_Port    /* PB8  */
#define CAR_S2_PIN                OUT2_Pin
#define CAR_S3_PORT               OUT3B7_GPIO_Port  /* PB7  —— 已确认 OUT3 接这里 */
#define CAR_S3_PIN                OUT3B7_Pin
#define CAR_S4_PORT               OUT4_GPIO_Port    /* PB15 */
#define CAR_S4_PIN                OUT4_Pin
#define CAR_S5_PORT               OUT5_GPIO_Port    /* PB5  */
#define CAR_S5_PIN                OUT5_Pin

/*==============================================================================
 * 4. 循迹控制参数（先跑起来再按需要微调）
 *============================================================================*/

/* 控制周期，单位 ms。10 ms = 100 Hz，足够跟上小车 */
#define CAR_CTRL_PERIOD_MS        10U

/* 直道基础速度，0 ~ 100（对应占空比百分比）
 * 38 太快：直角弯前车头会先冲过拐角，红外探头越过垂直线就看不到轨道。
 * 26 起步；还嫌快就 22，太慢就 30。 */
#define CAR_BASE_SPEED            26

/* 差速最大速度上限，防止转弯时一侧超速打滑 */
#define CAR_MAX_SPEED             55

/* 转向时内侧轮允许的最低速度，避免完全停转导致卡死 */
#define CAR_MIN_SPEED             12

/* 转向增益（大误差段）：误差 ±100 时叠加 ±CAR_STEER_GAIN 的速度差
 * 车速降下来以后每帧位移变小，增益可以给得更果断 */
#define CAR_STEER_GAIN            65

/* 小误差段的柔和增益 */
#define CAR_GAIN_SOFT             38

/* 增益切换点：|误差| 小于这个值用柔和增益，大于它用 CAR_STEER_GAIN */
#define CAR_SOFT_BAND             30

/* 误差低通滤波：新读数占的百分比。
 * **传感器优先**：给到 80，基本就是"读到什么就用什么"，只滤掉单帧毛刺。
 * 之前 55 会让车滞后 2 帧才响应，弯道里慢半拍。 */
#define CAR_ERROR_FILTER          80

/* 转向死区：滤波后 |误差| 小于这个值就不修正。**传感器优先**，压到 3 */
#define CAR_STEER_DEADBAND        3

/*==============================================================================
 * 3.1 探头前置补偿（你说探头在前轮前方 4~5 cm，这条很关键）
 *============================================================================*/

/* 探头中心到车轴（前轮）的距离，单位 mm。装在前轮前方约 45mm。
 * 探头是"提前看到"路况的，等车轴走到那儿已经晚了一步：
 *   - 弯道里车会比线慢半拍 -> 冲出外侧
 *   - 直角弯里探头先扫到横线，车轴还在后面
 * 所以转向必须"提前做"，用下面的微分增益把这段距离补回来。 */
#define CAR_SENSOR_AHEAD_MM       45

/* 微分增益（百分比）：偏差变化越快，额外加的转向越多，相当于预判。
 * 探头越靠前、车速越快，这个值要越大。45mm 前置 + 现在的低速：60~80 */
#define CAR_STEER_KD              70

/* 微分项限幅：开关量传感器会在两路之间跳变，D 项不加限幅会打出冲击 */
#define CAR_D_LIMIT               45

/* PWM 死区补偿：真实电机在占空比低于这个值时根本不转。
 * 不补偿的话，"内侧轮 15%" 实际上是停转的，差速变成单轮驱动，
 * 转向就会过冲、来回抖。补偿后 1~100 的逻辑速度都能真实落在电机的可转区间。 */
#define CAR_PWM_DEADZONE          18U

/* 转向方向（**唯一的转向总开关**）：
 *   +1 = 线在左边 -> 向左转（物理正确，也是你要的目标：
 *        单路偏离时把车修正回 OUT3 压线）
 *   -1 = 整体反过来（硬件左右接反时用这个）
 * 直角弯会自动跟随这个值，不需要另外调。 */
#define CAR_STEER_DIRECTION       (+1)

/* 1 = 自动学习转向方向（推荐开着）
 * 原理：如果连续若干帧"修正之后偏差反而变大"，说明左右修正方向是反的，
 *       程序自己把方向翻过来并锁定，不用你去猜。
 * 现象：车一开始抖几下、往一侧偏，然后自己稳下来沿线跑，就是它在自动纠正。 */
#define CAR_DIR_AUTO_LEARN        0

/* 自动学习触发条件：偏差落在这个区间才统计（太小没意义，太大多半是直角弯） */
#define CAR_DIR_LEARN_MIN_ERR     40
#define CAR_DIR_LEARN_MAX_ERR     90

/* 连续这么多帧"越修越偏"就翻转方向（10ms/帧，8 帧 = 80ms） */
#define CAR_DIR_LEARN_FRAMES      8U

/* 误差大时自动降速：|error| = 100 时，基础速度降低这么多。
 * 基础速度已经只有 26，这里不必再猛降，否则转弯时车几乎停住 */
#define CAR_TURN_SLOWDOWN         12

/* 1 = 允许内侧轮反转（急弯时内侧轮倒转，转弯半径最小）
 * 不是一直反转：只有 |error| 超过 CAR_HARD_TURN_ERROR 才放开，直道仍然温和不画龙 */
#define CAR_ALLOW_INNER_REVERSE   1

/* |error| 超过这个值就放开内轮反转上限（0~100）。越大越保守 */
#define CAR_HARD_TURN_ERROR       45

/* 全黑（十字/横线）时的通过速度。
 * 这里同时是"直角弯前的速度"——太大就会冲过拐角，探头越过垂直线。 */
#define CAR_CROSS_SPEED           20

/* 十字/横线最多直行这么久；超时说明压在大片黑上，转去脱线搜索 */
#define CAR_CROSS_MAX_MS          350U

/*==============================================================================
 * 4.1 脱线后的自动找回（弯道/直角弯/车身歪着起跑都靠这段）
 *============================================================================*/

/* 脱线恢复分三段，单位 ms。顺序很重要：
 *   第 1 段 倒车   —— 两轮同时后退，把红外探头退回能看到轨道的位置。
 *                    直角弯冲过拐角时，垂直线已经在车头后方，只有退回来才扫得到。
 *   第 2 段 原地扫描 —— 外轮前进、内轮后退，绕车轴转一圈找线
 *   第 3 段 大弧扫描 —— 外轮快、内轮慢，边前进边画弧，覆盖稍远处
 *   三段都失败才刹车停住 */
#define CAR_LOST_COAST_MS         250U    /* 惯性段：先按原方向低速再走一点（瞬间过缝） */
#define CAR_LOST_BACK_MS          300U    /* 倒车时长（直角弯最关键的一段） */
#define CAR_LOST_BACK_SPEED       22      /* 倒车速度（外侧轮，内侧轮取一半） */
#define CAR_LOST_SEARCH_MS        900U    /* 原地扫描时长 */
#define CAR_LOST_SWEEP_MS         1200U   /* 大弧扫描时长 */

/* 搜索时外侧轮速度（内侧轮反向，速度取一半） */
#define CAR_LOST_SPEED            22

/*==============================================================================
 * 4.2 直角弯（同一侧连续两路以上压线）
 *============================================================================*/

/* 直角弯时原地转向的速度：一侧正转、另一侧反转，转弯半径最小。
 * 左右两侧分开给，方便按实测差异微调；现在两侧用同一个值。
 * 两侧方向相反是对的，不需要改方向。 */
#define CAR_CORNER_SPEED          20
#define CAR_CORNER_SPEED_RIGHT    20

/* 直角弯的转向符号：固定 -1，让它和单路循迹保持同一个左右方向。
 * 原因是两者算法不同（直角弯是原地反转，单路是差速），
 * 同一套"线在哪边"的输入必须靠这个 -1 才能输出同一个转向。
 * 调方向请只改 CAR_STEER_DIRECTION，不要动这里。 */
#define CAR_CORNER_DIR_SIGN       (-1)

/* 直角转弯最长持续多久，超时就放弃、转去脱线恢复（防止一直原地打转耗电） */
#define CAR_CORNER_MAX_MS         1400U

/*==============================================================================
 * 5. 调试
 *============================================================================*/

/* 1 = 用板载 PC13 LED 做状态指示（.ioc 未配置 PC13，这里自行初始化）
 *     上电后快闪 3 次 = 初始化完成
 *     运行中：压到黑线（GO）常亮 / 十字或横线（CROSS）快闪 / 没压线（LOST）熄灭
 *     也就是说：灯亮 = 程序认为"检测到黑线"；灯灭 = 程序认为"没有黑线" */
#define CAR_USE_STATUS_LED        1

/* 1 = 上电自动校准传感器极性
 *     注意：现在极性已经按实测固定成 GPIO_PIN_SET（信号灯灭=检到黑线=DO高），
 *     自动校准反而可能在上电位置不巧时翻错，所以默认关闭。
 *     只有当换了一批极性相反的模块时才需要打开。 */
#define CAR_AUTO_POLARITY         0

/* 上电自检：三段式，用来一次性定位"方向反"和"某一路不转"
 *   >0 = 开启，数值是每段动作时间（毫秒）；0 = 关闭（比赛时关掉省时间）
 * 现象与结论：
 *   ① 两轮同时小幅前进 —— 车头往**后**走 => 车轮前后装反，
 *      改 CAR_LEFT_MOTOR_SIGN / CAR_RIGHT_MOTOR_SIGN 为 -1（不是改 STEER_DIRECTION！）
 *   ② 只转左轮 —— 左轮不动 => PWMB 没接到 PA1（这一路没有 PWM 信号）
 *   ③ 只转右轮 —— 右轮不动 => PWMA(PA2) 接线有问题
 *   ②③ 都正常但车不按预期转弯 => 才是 STEER_DIRECTION / SENSOR_ORDER_SIGN 的问题 */
#define CAR_SELFTEST_SPIN_MS      200U

/* 1 = 传感器自检模式（**电机完全不动**，用来核对极性和左右顺序）
 *     进这个模式后：PC13 灯直接跟随传感器 —— 任意一路压到黑线灯就亮。
 *     用手指/黑胶带逐个挡传感器：
 *       灯亮 = 该路"检测到黑线"有效（说明 CAR_SENSOR_BLACK_LEVEL 设对了）
 *       灯不亮 = 极性和 CAR_SENSOR_BLACK_LEVEL 相反，需要改它
 *     核对完把这里改回 0 重新编译，才会上赛道。 */
#define CAR_SENSOR_SELFTEST       0

/* 自检模式靠 PC13 灯显示结果，没灯就没意义，这里直接拦住 */
#if (CAR_SENSOR_SELFTEST != 0) && (CAR_USE_STATUS_LED == 0)
  #error "CAR_SENSOR_SELFTEST requires CAR_USE_STATUS_LED = 1"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CAR_CONFIG_H */
