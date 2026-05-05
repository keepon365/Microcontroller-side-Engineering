/**
 * @file hardware_config.h
 * @brief 硬件引脚分配与参数配置（基于 STM32F103RCT6，HAL 库）
 * @note  本文件与 CubeMX 配置保持一致，但存在引脚冲突需手动调整（见注释）
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "stm32f1xx_hal.h"

/*============================ 调试引脚保护 ============================*/
/* 注意：以下引脚被调试接口占用，请勿分配给其他外设 */
// PA13 (JTMS/SWDIO), PA14 (JTCK/SWCLK), PA15 (JTDI), PB3 (JTDO), PB4 (JTRST)

/*============================ 电机驱动 (L298N) ============================*/
/* 警告：当前分配存在引脚复用冲突，建议修改：
   - M2 方向引脚与 USART2 冲突
   - M4 方向引脚与 I2C2 SCL 冲突
   - 部分方向引脚与 PWM 输出引脚重叠（若 PWM 使用对应定时器通道）
   实际使用时请调整至不冲突的引脚，并同步更新 CubeMX 配置。
*/

// 电机1 (M1) 方向控制引脚
#define M1_IN1_PORT        GPIOA
#define M1_IN1_PIN         GPIO_PIN_0
#define M1_IN2_PORT        GPIOA
#define M1_IN2_PIN         GPIO_PIN_1

// 电机2 (M2) 方向控制引脚
#define M2_IN1_PORT        GPIOA
#define M2_IN1_PIN         GPIO_PIN_2   /* 与 USART2_TX 冲突！*/
#define M2_IN2_PORT        GPIOA
#define M2_IN2_PIN         GPIO_PIN_3   /* 与 USART2_RX 冲突！*/

// 电机3 (M3) 方向控制引脚
#define M3_IN1_PORT        GPIOB
#define M3_IN1_PIN         GPIO_PIN_0
#define M3_IN2_PORT        GPIOB
#define M3_IN2_PIN         GPIO_PIN_1

// 电机4 (M4) 方向控制引脚
#define M4_IN1_PORT        GPIOB
#define M4_IN1_PIN         GPIO_PIN_2
#define M4_IN2_PORT        GPIOB
#define M4_IN2_PIN         GPIO_PIN_10  /* 与 I2C2_SCL 冲突！*/

// 电机 PWM 定时器与通道（L298N 使能引脚）
// 注意：PWM 输出引脚由定时器通道决定，实际连接时需确保与方向引脚不冲突
// TIM2 通道1~4 对应 PA0~PA3（与 M1、M2 方向引脚重叠，需调整）
// TIM3 通道3~4 对应 PB0、PB1（与 M3 方向引脚重叠，需调整）
#define MOTOR_PWM_TIM2      TIM2
#define MOTOR_PWM_TIM3      TIM3
#define MOTOR1_PWM_CH       TIM_CHANNEL_1   // TIM2_CH1 (PA0)
#define MOTOR2_PWM_CH       TIM_CHANNEL_2   // TIM2_CH2 (PA1)
#define MOTOR3_PWM_CH       TIM_CHANNEL_3   // TIM3_CH3 (PB0)
#define MOTOR4_PWM_CH       TIM_CHANNEL_4   // TIM3_CH4 (PB1)

/*============================ 舵机控制 (TIM4) ============================*/
// 周期 20ms → 频率 50Hz，脉宽 0.5~2.5ms 对应占空比 2.5%~12.5%
#define SERVO_TIM           TIM4
#define SERVO_PWM_FREQ      50          // Hz
#define SERVO_PWM_PERIOD_MS 20          // ms
#define SERVO_PULSE_MIN_US  500         // 最小脉宽 0.5ms
#define SERVO_PULSE_MAX_US  2500        // 最大脉宽 2.5ms
// 占空比范围（ARR 自动重装载值由定时器时钟计算，此处仅作参考）
#define SERVO_DUTY_MIN      0.025f      // 2.5%
#define SERVO_DUTY_MAX      0.125f      // 12.5%

// 舵机引脚分配
#define SERVO1_PORT         GPIOB
#define SERVO1_PIN          GPIO_PIN_6   // TIM4_CH1
#define SERVO2_PORT         GPIOB
#define SERVO2_PIN          GPIO_PIN_7   // TIM4_CH2
#define SERVO3_PORT         GPIOB
#define SERVO3_PIN          GPIO_PIN_8   // TIM4_CH3

/*============================ 四路巡线传感器 (I2C2) ============================*/
#define LINE_SENSOR_I2C     I2C2
#define LINE_SENSOR_ADDR    0x30         // 设备地址（占位，需实测）
// I2C 引脚
#define LINE_SENSOR_SCL_PORT GPIOB
#define LINE_SENSOR_SCL_PIN  GPIO_PIN_10 /* 与 M4_IN2 冲突！建议更换至其他空闲引脚 */
#define LINE_SENSOR_SDA_PORT GPIOB
#define LINE_SENSOR_SDA_PIN  GPIO_PIN_11

/*============================ 蓝牙 HC-05 (USART2) ============================*/
#define BT_USART            USART2
#define BT_BAUDRATE         115200
// 蓝牙引脚
#define BT_TX_PORT          GPIOA
#define BT_TX_PIN           GPIO_PIN_2   /* 与 M2_IN1 冲突！*/
#define BT_RX_PORT          GPIOA
#define BT_RX_PIN           GPIO_PIN_3   /* 与 M2_IN2 冲突！*/

/*============================ 转向选择开关 ============================*/
#define STEER_SWITCH_PORT   GPIOB
#define STEER_SWITCH_PIN    GPIO_PIN_12
// 开关状态定义
#define STEER_LEFT          GPIO_PIN_RESET   // 低电平=左转
#define STEER_RIGHT         GPIO_PIN_SET     // 高电平=右转

/*============================ 复位按键 (下降沿触发) ============================*/
#define RESET_BUTTON_PORT   GPIOB
#define RESET_BUTTON_PIN    GPIO_PIN_13
// 有效电平
#define RESET_PRESSED       GPIO_PIN_RESET   // 按下为低电平（下降沿触发）

/*============================ 编码器预留 (输入捕获) ============================*/
/* 预留 TIM1、TIM2、TIM3、TIM5 用于编码器输入捕获，引脚未分配，后续按需定义 */
#define ENCODER_TIM1        TIM1
#define ENCODER_TIM2        TIM2
#define ENCODER_TIM3        TIM3
#define ENCODER_TIM5        TIM5
// 注意：使用编码器时需配置为输入捕获模式，具体引脚由实际硬件决定

/*============================ 运动学参数（需实测调参）============================*/
// 轮距、轴距等，单位 mm
#define WHEEL_BASE          200.0f       // 轴距，需实测
#define TRACK_WIDTH         180.0f       // 轮距，需实测
#define WHEEL_DIAMETER      65.0f        // 车轮直径，需实测
#define ENCODER_PPR         400          // 编码器线数（每圈脉冲数），需实测

/*============================ PID 控制参数（需实测整定）============================*/
// 速度环 PID
#define SPEED_PID_KP        1.0f         // 比例系数，需整定
#define SPEED_PID_KI        0.1f         // 积分系数，需整定
#define SPEED_PID_KD        0.0f         // 微分系数，需整定
// 方向环 PID
#define DIR_PID_KP          1.2f         // 比例系数，需整定
#define DIR_PID_KI          0.05f        // 积分系数，需整定
#define DIR_PID_KD          0.01f        // 微分系数，需整定

/*============================ 寻迹传感器阈值（需实测）============================*/
// 四路传感器黑白阈值，0=黑，1=白（取决于实际传感器）
#define LINE_THRESHOLD_1    1500         // 传感器1 AD值/比较值，需实测
#define LINE_THRESHOLD_2    1500         // 传感器2
#define LINE_THRESHOLD_3    1500         // 传感器3
#define LINE_THRESHOLD_4    1500         // 传感器4

/*============================ 舵机角度限位（需实测）============================*/
// 舵机1 角度范围（单位：度）
#define SERVO1_ANGLE_MIN    -90.0f
#define SERVO1_ANGLE_MAX    90.0f
// 舵机2
#define SERVO2_ANGLE_MIN    -90.0f
#define SERVO2_ANGLE_MAX    90.0f
// 舵机3
#define SERVO3_ANGLE_MIN    -90.0f
#define SERVO3_ANGLE_MAX    90.0f

/*============================ 其他配置 ============================*/
// 系统主频（与 CubeMX 一致）
#define HSE_VALUE           8000000U     // 外部晶振 8MHz
#define SYSCLK_FREQ         72000000U    // 系统时钟 72MHz

#endif /* HARDWARE_CONFIG_H */
