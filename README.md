[小车代码详解.md](https://github.com/user-attachments/files/29727881/default.md)
# ESP32-Camera-Tracking-Car
ESP32 photoelectric camera tracking car
# 光电循迹小车代码详解

> 本文档详细解释双ESP32光电循迹智能小车的完整代码，帮助你从零理解每一行代码的作用。

---

## 目录

- [系统架构总览](#系统架构总览)
- [文件一：main.cpp — 主控板（ESP32 V1）](#文件一maincpp--主控板esp32-v1)
  - [1. 引脚定义](#1-引脚定义)
  - [2. 调参区域（核心参数）](#2-调参区域核心参数)
  - [3. 全局句柄和状态](#3-全局句柄和状态)
  - [4. 工具函数](#4-工具函数)
  - [5. 速度内环 PI 控制器](#5-速度内环-pi-控制器)
  - [6. 硬件初始化](#6-硬件初始化)
  - [7. 循线传感器处理（模拟归一化）](#7-循线传感器处理模拟归一化)
  - [8. 主程序 — 完整控制循环](#8-主程序--完整控制循环)
- [文件二：main(1).cpp — 视觉/姿态板（ESP32-S3）](#文件二main1cpp--视觉姿态板esp32-s3)
  - [1. 常量与全局变量](#1-常量与全局变量)
  - [2. I²C 自动发现与 MPU6050 驱动](#2-ic-自动发现与-mpu6050-驱动)
  - [3. 陀螺仪零偏标定](#3-陀螺仪零偏标定)
  - [4. 直道/弯道判别状态机](#4-直道弯道判别状态机)
  - [5. UART 通信与 LED 指示](#5-uart-通信与-led-指示)
  - [6. 主函数流程](#6-主函数流程)
- [两板协作流程](#两板协作流程)
- [关键算法深入](#关键算法深入)
  - [模拟归一化算法](#模拟归一化算法)
  - [加权重心法](#加权重心法)
  - [双环串级 PID](#双环串级-pid)
  - [弯道减速策略](#弯道减速策略)
  - [丢线恢复策略](#丢线恢复策略)

---

## 系统架构总览

```
┌─────────────────────────────────────────────────────────────────┐
│                        物理世界                               │
│   赛道黑线 ──→ 5路ITR9909传感器 ──→ 直流电机 ──→ 编码器        │
└─────────────────────────────────────────────────────────────────┘
         │                    ▲                │
         ▼                    │                ▼
┌─────────────────┐   ┌─────────────────┐
│  ESP32 V1 主控板  │◄──┤ ESP32-S3 姿态板 │
│  (main.cpp)      │UART│ (main(1).cpp)   │
│                 │    │                 │
│ • 传感器ADC读取   │    │ • MPU6050陀螺仪  │
│ • 模拟归一化      │    │ • I²C自动发现    │
│ • 加权重心法      │    │ • 直道/弯道判别  │
│ • 外环PD控制器    │    │ • 零偏自动标定   │
│ • 内环PI控制器    │    │ • 状态LED指示    │
│ • 弯道减速        │    │                 │
│ • 丢线恢复        │    │                 │
└─────────────────┘   └─────────────────┘
```

两块板通过 **UART 串口** 通信（S3板的TX(GPIO45) → 主控板的RX(GPIO22)），每 **20ms** 发送一个字节：`'1'` = 直道，`'0'` = 弯道。

---

## 文件一：main.cpp — 主控板（ESP32 V1）

**功能**：实时控制核心，负责传感器数据采集、循线算法执行和电机驱动。所有控制回路的计算都在此板上运行。

**文件规模**：839 行 | **框架**：PlatformIO + Arduino 框架（实际使用 ESP-IDF API）

### 1. 引脚定义

```cpp
// ===== 电机引脚 =====
// 左轮：IN1=GPIO15(PWM), IN2=GPIO13(方向)
#define PIN_IN1_A 15
#define PIN_IN2_A 13

// 右轮：IN1=GPIO14(PWM), IN2=GPIO25(方向)
#define PIN_IN1_B 14
#define PIN_IN2_B 25

// ===== 编码器引脚 =====
// 左轮编码器 A/B 相接 GPIO16/17，右轮接 GPIO18/19
#define PIN_E1A 16
#define PIN_E1B 17
#define PIN_E2A 18
#define PIN_E2B 19

// ===== 5路光电传感器（从左到右） =====
// L2=GPIO34, L1=GPIO35, C=GPIO32, R1=GPIO33, R2=GPIO27
#define TRACK_GPIO_1 34
#define TRACK_GPIO_2 35
#define TRACK_GPIO_3 32
#define TRACK_GPIO_4 33
#define TRACK_GPIO_5 27

// ===== S3陀螺仪板通信串口 =====
// S3_TX(GPIO45) → 本板 RX(GPIO22)
// 本板 TX(GPIO23) → S3_RX(GPIO46), 两块板必须共地
#define GYRO_UART_RX_PIN 22
#define GYRO_UART_TX_PIN 23
#define GYRO_UART_BAUD 115200
```

**为什么用这些引脚？**
- GPIO34/35/32/33 是 **ADC1** 的通道，ESP32 的 ADC2 和 WiFi 共享资源，用 ADC1 更稳定
- GPIO27 是 ADC2_CH7，单独使用没有问题（本系统没有开 WiFi）
- 编码器用 GPIO16-19，这些引脚支持 ESP32 的 **PCNT（脉冲计数）** 硬件外设，不占 CPU

---

### 2. 调参区域（核心参数）

这是整个程序最重要的部分——所有你可以调整的参数都在这里。

#### A. 串口调试

```cpp
#define DEBUG_PRINT_EVERY_N 5   // 每5个控制周期打印一次（减少串口开销）
```

#### B. 硬件设置

```cpp
// PWM 配置
#define PWM_FREQ_HZ        1000      // PWM频率1000Hz（L298N推荐1-25kHz）
#define PWM_RESOLUTION     LEDC_TIMER_10_BIT  // 10位分辨率 → 0~1023
#define PWM_MAX_DUTY       900.0f    // 正向最大占空比（1023的88%，留余量保护电机）

// 反转限幅（小车也可以倒车）
#define REVERSE_SPEED_REF_MAX  6500.0f   // 反转最大速度（pps）
#define PWM_REV_MAX_DUTY       550.0f    // 反转最大占空比

// 死区补偿
#define LEFT_DEAD_PWM      270.0f    // 左轮死区阈值（低于此PWM电机不转）
#define RIGHT_DEAD_PWM     295.0f    // 右轮死区阈值
#define REV_DEAD_PWM       150.0f    // 反向死区

// 编码器方向校正
#define LEFT_ENC_SIGN       1.0f     // 前进时左轮编码器为正
#define RIGHT_ENC_SIGN     -1.0f     // 前进时右轮编码器为负（硬件接线反了，软件纠正）
```

> **死区是什么？** 电机的 PWM 占空比低于一定值时，电机不会转动（摩擦力大于电磁力矩），这个阈值就是死区。如果直接输出一个很小的 PWM（比如 50），电机不转但程序以为在转，积分项会不断累积导致失控。

#### C. 系统控制参数

```cpp
#define CONTROL_DT_MS      20        // 控制周期20ms → 50Hz控制频率
```

**循线外环 PD 控制器：**

```cpp
#define BASE_SPEED_PPS      48000.0f  // 直道巡航速度（编码器脉冲/秒，约0.6m/s）
#define KP_LINE             12000.0f  // 外环比例增益：偏差→速度差的缩放
#define KD_LINE             4200.0f   // 外环微分增益：抑制震荡
#define NONLINEAR_TURN_GAIN 18900.0f  // 非线性转向增益：大偏差时强力拉回
#define DELTA_SPEED_MAX     26000.0f  // 左右轮速度差上限

#define POS_FILTER_ALPHA    0.20f     // EMA低通滤波系数（越小越平滑但越滞后）
#define LINE_ERROR_DEADBAND 0.08f     // 偏差死区（偏差<0.08视为居中，避免微抖）

// 弯道减速
#define CURVE_SLOWDOWN_GAIN  22500.0f // 弯道减速系数（|偏差|越大，速度降越多）
#define CURVE_MIN_SPEED_PPS  19200.0f // 弯道最低速度保护（直道速度的40%）

// 丢线处理
#define LOST_BASE_SPEED_PPS  13500.0f // 丢线后降速至此值
#define LINE_LOST_HOLD_CYCLES 2       // 丢线后保持最近偏差的周期数（2×20ms=40ms）
```

**速度内环 PI 控制器：**

```cpp
#define KP_SPEED_L  0.040f    // 左轮速度环P增益
#define KI_SPEED_L  0.300f    // 左轮速度环I增益
#define KP_SPEED_R  0.038f    // 右轮速度环P增益（略低，补偿机械差异）
#define KI_SPEED_R  0.290f    // 右轮速度环I增益
#define SPEED_INTEGRAL_LIMIT 6000.0f  // 积分限幅（防止积分饱和）
```

**速度前馈模型（基于实测 PWM-速度 线性拟合）：**

```cpp
// 前馈 = 基准PWM + (目标速度 - 基准速度) / 斜率
#define LEFT_FF_PWM_BASE            557.0f   // PWM=557时左轮约48000pps
#define LEFT_SPEED_AT_FF_PWM_BASE   48000.0f
#define LEFT_KM_PPS_PER_PWM         90.0f    // 左轮每PWM增加约90pps

#define RIGHT_FF_PWM_BASE           557.0f
#define RIGHT_SPEED_AT_FF_PWM_BASE  48000.0f
#define RIGHT_KM_PPS_PER_PWM        95.0f    // 右轮每PWM增加约95pps（右轮略快）
```

> **前馈是什么？** 不用前馈时，PI控制器要从零开始计算PWM——先看出速度误差，再慢慢加上去，响应慢。有了前馈，直接给一个"差不多对"的PWM（基于实测模型计算），PI只需要修正10-15%的小误差，响应速度大幅提升。

#### D. 传感器标定

```cpp
// 每个传感器的白值和黑值需要标定
// 白=高ADC(~4095)，黑=低ADC(~1200)
static int sensor_white[5] = {4095, 4095, 4095, 4095, 4095};
static int sensor_black[5] = {1200, 1200, 1200, 1200, 1200};

// 5路归一化强度之和超过此值才认为"看到黑线"
#define LINE_SUM_MIN  0.10f
```

---

### 3. 全局句柄和状态

```cpp
// LEDC PWM 通道分配（ESP32的硬件PWM）
#define CH_LEFT_IN1   LEDC_CHANNEL_0   // 左轮IN1(PWM)
#define CH_LEFT_IN2   LEDC_CHANNEL_1   // 左轮IN2(方向)
#define CH_RIGHT_IN1  LEDC_CHANNEL_2   // 右轮IN1
#define CH_RIGHT_IN2  LEDC_CHANNEL_3   // 右轮IN2(方向)

// PCNT 编码器计数单元
static pcnt_unit_handle_t pcnt_left_unit = NULL;
static pcnt_unit_handle_t pcnt_right_unit = NULL;

// ADC 句柄（ADC1和ADC2各一个）
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_oneshot_unit_handle_t adc2_handle = NULL;

// PI控制器结构体
typedef struct {
    float kp;          // 比例增益
    float ki;          // 积分增益
    float integral;    // 积分累加值
    float integral_limit; // 积分限幅
} PI_Controller;

// 左右轮各一个独立的PI控制器
static PI_Controller pi_left  = { .kp = 0.040f, .ki = 0.300f, ... };
static PI_Controller pi_right = { .kp = 0.038f, .ki = 0.290f, ... };
```

---

### 4. 工具函数

#### 限幅函数

```cpp
static inline float clampf_local(float x, float minValue, float maxValue) {
    if (x < minValue) return minValue;
    if (x > maxValue) return maxValue;
    return x;
}
```

#### Slew Rate 限速函数

```cpp
// 平滑过渡：每次最多变化 max_step
static inline float slew_to(float current, float target, float max_step) {
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}
```

> **为什么需要 slew rate？** 如果直道加速信号突然来，速度从 48000 瞬间跳到 66000，小车会因为速度突变而失稳。用 slew rate 限制每次只增加 1200pps，速度平滑提升，小车稳定。

#### 电机驱动函数

```cpp
// 左轮驱动
static void motor_left_set_pwm(float pwm) {
    if (pwm > 0.0f) {
        // 正转：IN1给PWM，IN2=0
        ledc_set_channel_duty(CH_LEFT_IN2, 0.0f);
        ledc_set_channel_duty(CH_LEFT_IN1, pwm);
    } else if (pwm < 0.0f) {
        // 反转：IN1=0，IN2给PWM
        ledc_set_channel_duty(CH_LEFT_IN1, 0.0f);
        ledc_set_channel_duty(CH_LEFT_IN2, -pwm);
    } else {
        // 刹车：两个IN都=0
        ledc_set_channel_duty(CH_LEFT_IN1, 0.0f);
        ledc_set_channel_duty(CH_LEFT_IN2, 0.0f);
    }
}
```

> **H桥驱动原理**：L298N 内部有两个 H 桥，每个 H 桥有两个输入（IN1、IN2）：
> - IN1=PWM, IN2=0 → 电机正转
> - IN1=0, IN2=PWM → 电机反转
> - IN1=0, IN2=0 → 电机停止（刹车）
> - 右轮因为机械安装方向不同，代码中把 IN1/IN2 的角色互换了

---

### 5. 速度内环 PI 控制器

这是整个控制系统中最重要的函数之一。

#### 前馈计算

```cpp
static float feedforward_left_pwm(float speed_ref) {
    // PWM = 基准PWM + (目标速度 - 基准速度) / 每PWM速度增量
    float pwm = LEFT_FF_PWM_BASE
              + (speed_ref - LEFT_SPEED_AT_FF_PWM_BASE) / LEFT_KM_PPS_PER_PWM;
    return pwm;
}
```

**举例**：目标速度=60000pps → PWM = 557 + (60000-48000)/90 = 557+133 = 690

#### 死区补偿

```cpp
static float apply_signed_deadzone(float pwm, float ref_speed,
                                    float forward_dead_pwm, ...) {
    if (fabsf(ref_speed) < MIN_ACTIVE_SPEED_REF) return 0.0f;  // 目标太小，不转

    if (ref_speed > 0.0f) {
        // 正向死区：如果输出PWM低于死区阈值，直接跳到阈值
        if (pwm > 0.0f && pwm < forward_dead_pwm) pwm = forward_dead_pwm;
    }
    // ... 反向同理
    return clampf_local(pwm, pwm_min, pwm_max);
}
```

> **为什么这样做？** 比如左轮死区270，PI算出PWM=200。如果不补偿，输出200 → 电机不转 → 速度=0 → 误差持续存在 → 积分疯狂累积 → 等PWM终于超过270时积分已过大 → 电机猛冲 → 超调。补偿后：直接输出270，电机立刻开始转，积分项正常运作。

#### PI 更新函数（完整版本）

```cpp
static float pi_speed_update(PI_Controller *pi, float error, float dt,
                              float ff_pwm, float ref_speed, float dead_pwm) {
    // 1. 目标速度为0 → 清空积分，输出0
    if (fabsf(ref_speed) < MIN_ACTIVE_SPEED_REF) {
        pi->integral = 0.0f;
        return 0.0f;
    }

    // 2. 计算候选积分（先假设本次更新积分）
    float candidate_integral = pi->integral + error * dt;
    candidate_integral = clampf_local(candidate_integral, -pi->integral_limit, pi->integral_limit);

    // 3. 抗积分饱和（Anti-Windup）
    //    计算未饱和输出 = 前馈 + P项 + I项（用候选积分）
    float unsat_pwm = ff_pwm + pi->kp * error + pi->ki * candidate_integral;

    //    如果输出已饱和 且 误差方向与饱和方向一致 → 不更新积分！
    bool high_sat = (unsat_pwm > pwm_max) && (error > 0.0f);
    bool low_sat  = (unsat_pwm < pwm_min) && (error < 0.0f);

    if (!high_sat && !low_sat) {
        pi->integral = candidate_integral;  // 正常更新积分
    }
    // 否则积分保持不变（冻结）

    // 4. 用实际积分计算PWM输出
    float pwm = ff_pwm + pi->kp * error + pi->ki * pi->integral;

    // 5. 限幅 + 死区补偿
    pwm = clampf_local(pwm, pwm_min, pwm_max);
    pwm = apply_signed_deadzone(pwm, ref_speed, dead_pwm, pwm_min, pwm_max);

    return pwm;
}
```

> **抗积分饱和图解**：
> ```
> 正常情况：PWM=500(未饱和) → 积分正常更新
> 饱和情况：PWM=1023(已到上限) + 速度还是不够(error>0)
>         → 积分冻结！避免积分无限增长
>         → 等error变负时自动解冻
> ```

---

### 6. 硬件初始化

#### PWM 初始化

```cpp
static void init_pwm(void) {
    // 配置定时器：1000Hz, 10位分辨率
    ledc_timer_config_t timer_conf = {
        .freq_hz = PWM_FREQ_HZ,          // 1000Hz
        .duty_resolution = PWM_RESOLUTION, // 10位(0~1023)
        .timer_num = LEDC_TIMER_0,
    };

    // 配置4个PWM通道（左IN1、左IN2、右IN1、右IN2）
    // 每个通道绑定到对应的GPIO
    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ch_conf[i]);
    }
    motors_stop();  // 初始化后立即停车
}
```

#### 编码器（PCNT）初始化

```cpp
static void init_one_pcnt_unit_signed(pcnt_unit_handle_t *unit_out,
                                       int pulse_gpio, int dir_gpio) {
    // 1. 创建PCNT单元：计数范围 -32768 ~ 32767
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
    };

    // 2. 配置毛刺滤波器：滤除<1000ns的脉冲（编码器抖动）
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };

    // 3. 配置通道：
    //    A相上升沿 → 计数+1 或 -1（取决于B相电平）
    //    B相电平决定方向
    pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, ...);
    pcnt_channel_set_level_action(chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                       PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
}
```

> **编码器原理**：电机每转一圈，A/B 相各输出 390 个脉冲，A 超前 B 90°（正转）或 B 超前 A 90°（反转）。ESP32 的 PCNT 硬件自动根据 A/B 相的相位关系决定计数方向，从寄存器直接读数即可得到速度。

#### ADC 初始化

```cpp
static void init_adc(void) {
    // 创建ADC1和ADC2的oneshot单元
    // 配置5个通道，衰减11dB（测量范围约0-3.3V→对应0-4095）
    // GPIO34→ADC1_CH6, GPIO35→ADC1_CH7, GPIO32→ADC1_CH4,
    // GPIO33→ADC1_CH5, GPIO27→ADC2_CH7
}
```

---

### 7. 循线传感器处理（模拟归一化）

这是使小车能"看到"黑线的核心算法。

#### 数据结构

```cpp
typedef struct {
    int raw[5];          // 5路原始ADC值（0~4095）
    float intensity[5];  // 5路归一化强度值（0~1，1=纯黑）
    float pos;           // 计算出的黑线位置（-2.0 ~ +2.0）
    float sum;           // 5路强度值之和（用于判断是否丢线）
    bool valid;          // 是否有效检测到黑线
} line_data_t;
```

#### 单传感器归一化

```cpp
static float normalize_sensor_to_black(int raw, int idx) {
    float white = (float)sensor_white[idx];  // 该传感器在白底上的ADC读数
    float black = (float)sensor_black[idx];  // 该传感器在黑线上的ADC读数

    // 核心公式：(white - raw) / (white - black)
    // raw=white → x=0（纯白）
    // raw=black → x=1（纯黑）
    // raw在中间 → x在0~1之间（连续值！）
    float x = (white - (float)raw) / (white - black);
    return clampf_local(x, 0.0f, 1.0f);
}
```

> **为什么不用简单的"大于阈值=白，小于阈值=黑"？** 二值化丢失了模拟量信息。归一化保留了"有多黑"的连续信息，配合加权重心法可以计算亚传感器间距精度的线偏差。

#### 5路传感器整体读取

```cpp
static line_data_t read_line_sensors(void) {
    line_data_t d = {};
    const float weight[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

    float weighted_sum = 0.0f;
    float intensity_sum = 0.0f;

    for (int i = 0; i < 5; i++) {
        d.raw[i] = read_track_adc(i);                       // 读ADC
        d.intensity[i] = normalize_sensor_to_black(d.raw[i], i); // 归一化

        weighted_sum += d.intensity[i] * weight[i];  // 分子：Σ(强度×权重)
        intensity_sum += d.intensity[i];              // 分母：Σ(强度)
    }

    if (intensity_sum > LINE_SUM_MIN) {
        // 看到黑线：计算重心位置
        d.pos = weighted_sum / intensity_sum;  // 加权重心法
        d.valid = true;
    } else {
        // 丢线：所有传感器都在白底上
        d.pos = 0.0f;
        d.valid = false;
    }
    return d;
}
```

> **加权重心法图解**：
> ```
> 传感器位置： L2    L1    C     R1    R2
> 权重：        -2    -1    0     +1    +2
> 传感器间距：  9mm   9mm   9mm   9mm
>              ←── 总跨度36mm ──→
>
> 例1：只有C在黑线上 → pos = (0×0)/(1) = 0（居中）
> 例2：C和R1各一半→ pos = (0+1)/(1+1) = 0.5（偏右4.5mm）
> 例3：只有R1在黑线上 → pos = 1.0（偏右9mm）
>
> 偏差分辨率 ≈ 4.5mm（9mm/2个权重单位）
> ```

---

### 8. 主程序 — 完整控制循环

#### 陀螺仪直道加速相关常量

```cpp
#define GYRO_STRAIGHT_TIMEOUT_MS   250    // 陀螺仪信号超时（超时后忽略）
#define GYRO_LINE_ERROR_TH         0.45f  // 光电偏差阈值（确认直道用）
#define GYRO_STRAIGHT_CONFIRM      5      // 连续确认5次才触发加速
#define GYRO_SPEED_BOOST_PPS       18000.0f  // 直道加速增量
#define GYRO_SPEED_BOOST_STEP_PPS  1200.0f   // 每次增加1200pps（平滑）
```

#### UART 通信初始化与读取

```cpp
// 初始化UART2：RX=GPIO22, TX=GPIO23, 115200bps
static void init_gyro_uart(void) { ... }

// 读取S3发来的数据
static bool read_gyro_straight_uart(bool *straight_out, int64_t *last_update_us) {
    uint8_t data[32];
    int len = uart_read_bytes(GYRO_UART_NUM, data, sizeof(data), 0);
    for (int i = 0; i < len; i++) {
        if (data[i] == '1') { *straight_out = true; ... }   // 直道
        else if (data[i] == '0') { *straight_out = false; ... } // 弯道
    }
}
```

#### `app_main()` — 程序入口

```cpp
extern "C" void app_main(void) {
    // === 初始化阶段 ===
    init_pwm();      // 1. 配置4路PWM
    init_adc();      // 2. 配置5路ADC
    init_pcnt();     // 3. 配置2路编码器
    init_gyro_uart();// 4. 配置与S3板的UART通信

    motors_stop();
    vTaskDelay(pdMS_TO_TICKS(2000));  // 上电后等待2秒（安全延时）

    // 清零编码器计数
    read_and_clear_pcnt(pcnt_left_unit);
    read_and_clear_pcnt(pcnt_right_unit);

    // === 状态变量初始化 ===
    float pos_filtered = 0.0f;       // EMA滤波后的偏差位置
    float prev_line_error = 0.0f;    // 上一周期的线偏差（用于微分）
    float d_line_filtered = 0.0f;    // 微分项滤波值
    float last_valid_pos = 0.0f;     // 最后一次有效偏差（丢线时保持用）
    int   short_lost_count = 0;      // 短暂丢线计数器
    bool  gyro_straight = false;     // 陀螺仪报告的直道标志
    float gyro_speed_boost = 0.0f;   // 直道加速量

    // === 主控制循环（50Hz，每20ms一次） ===
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_DT_MS)); // 精确20ms周期

        // -------- 第0步：读陀螺仪 --------
        read_gyro_straight_uart(&gyro_straight, &gyro_last_update_us);
        bool gyro_fresh = (超时检查);
        bool gyro_straight_now = gyro_fresh && gyro_straight;

        // -------- 第1步：读编码器计算速度 --------
        int cnt_l = read_and_clear_pcnt(pcnt_left_unit);   // 读左轮20ms内的脉冲数
        int cnt_r = read_and_clear_pcnt(pcnt_right_unit);
        float speed_l = LEFT_ENC_SIGN * cnt_l / dt;   // 速度 = 脉冲数/时间×方向
        float speed_r = RIGHT_ENC_SIGN * cnt_r / dt;

        // -------- 第2步：读5路光电传感器 --------
        line_data_t line = read_line_sensors();

        // 丢线处理：如果短暂丢线(<2周期)，保持上一次的有效偏差继续走
        if (line.valid) {
            last_valid_pos = line.pos;
            short_lost_count = 0;
        } else if (short_lost_count < LINE_LOST_HOLD_CYCLES) {
            short_lost_count++;
            line.valid = true;        // 强制标记"有效"
            line.pos = last_valid_pos; // 使用上次的位置
        }

        // -------- 第3步：计算基础速度 + 循迹偏差 --------
        float base_speed = BASE_SPEED_PPS;  // 48000

        if (line.valid) {
            // 3a. EMA低通滤波偏差
            pos_filtered = 0.20 * pos_filtered + 0.80 * line.pos;

            // 3b. 线偏差（取反，使偏差方向与转向方向一致）
            line_error = -pos_filtered;

            // 3c. 死区处理：微小偏差视为0
            if (fabsf(line_error) < 0.08f) line_error = 0.0f;

            // 3d. 陀螺仪直道加速判断（双确认）
            bool line_is_straight = fabsf(line_error) < 0.45f;
            if (gyro_straight_now && line_is_straight) {
                straight_count++;  // 陀螺仪说直 + 光电说直 → 确认
            } else {
                straight_count = 0;
            }

            // 3e. 加速量平滑过渡
            float boost_target = (straight_count >= 5) ? 18000.0f : 0.0f;
            gyro_speed_boost = slew_to(gyro_speed_boost, boost_target, 1200.0f);

            // 3f. 弯道减速：基础速度 = (直道速度+加速) - 减速系数×|偏差|
            float boosted_base = 48000 + gyro_speed_boost;  // 最多66000
            base_speed = boosted_base - 22500 * fabsf(pos_filtered);
            base_speed = clampf_local(base_speed, 19200, boosted_base);
        } else {
            // 丢线超过2周期：大幅降速 + 固定偏差
            base_speed = 13500;
            if (prev_line_error > 0.05f) line_error = 1.5f;   // 上次偏右→继续右转
            else if (prev_line_error < -0.05f) line_error = -1.5f; // 上次偏左→继续左转
            straight_count = 0;
            gyro_speed_boost = slew_to(gyro_speed_boost, 0, 1200); // 平滑退出加速
        }

        // -------- 第4步：外环PD → 左右轮目标速度差 --------
        // 微分计算（d(error)/dt）
        float d_line = (line_error - prev_line_error) / dt;
        d_line_filtered = 0.75 * d_line_filtered + 0.25 * d_line; // EMA滤波微分

        // 非线性转向增益（大偏差时强力拉回）
        float nonlinear = 18900 * line_error * fabsf(line_error);

        // delta_speed = Kp×偏差 + Kd×微分 + 非线性项
        float delta_speed_ref = 12000*line_error + 4200*d_line_filtered + nonlinear;
        delta_speed_ref = clampf_local(delta_speed_ref, -26000, 26000);

        // 左右轮目标速度 = 基础速度 ± 速度差
        float speed_l_ref = base_speed - delta_speed_ref;
        float speed_r_ref = base_speed + delta_speed_ref;

        // -------- 第5步：内环PI → PWM --------
        float err_l = speed_l_ref - speed_l;  // 左轮速度误差
        float err_r = speed_r_ref - speed_r;

        float ff_l = feedforward_left_pwm_signed(speed_l_ref);   // 前馈PWM
        float ff_r = feedforward_right_pwm_signed(speed_r_ref);

        float pwm_l = pi_speed_update(&pi_left, err_l, dt, ff_l, speed_l_ref, 270);
        float pwm_r = pi_speed_update(&pi_right, err_r, dt, ff_r, speed_r_ref, 295);

        motor_left_set_pwm(pwm_l);
        motor_right_set_pwm(pwm_r);

        // -------- 第6步：调试打印 --------
        // 每5个周期打印一次所有关键变量
    }
}
```

#### 主循环的完整数据流

```
每20ms执行一次：

 5路ADC ──→ 归一化 ──→ 加权重心法 ──→ pos(-2~+2)
                                          │
                                   EMA滤波(pos_filtered)
                                          │
                                   ┌──────┴──────┐
                                   │  外环 PD     │← Kp=12000, Kd=4200
                                   │  error→delta │   +非线性增益
                                   └──────┬──────┘
                                          │
                              ┌───────────┴───────────┐
                              │  delta_speed_ref       │
                              │  ±26000pps限幅         │
                              └───────────┬───────────┘
                                          │
                    base_speed ──→ speed_ref_L = base - delta
                    (弯道自适应)    speed_ref_R = base + delta
                                          │
                              ┌───────────┴───────────┐
                              │   内环 PI ×2           │← 前馈+死区+抗饱和
                              │   speed_ref→PWM        │
                              └───────────┬───────────┘
                                          │
                              ┌───────────┴───────────┐
                              │   电机驱动              │
                              │   PWM→L298N→直流电机   │
                              └───────────────────────┘
                                          │
                              编码器速度 ──→ 反馈回内环
```

---

## 文件二：main(1).cpp — 视觉/姿态板（ESP32-S3）

**功能**：感知增强单元，负责 MPU6050 陀螺仪姿态检测和直道/弯道判别，通过 UART 向主控板发送标志位。

**文件规模**：368 行 | **框架**：ESP-IDF（C语言）

### 1. 常量与全局变量

```cpp
// I²C 配置
static const i2c_port_t I2C_PORT = I2C_NUM_0;
static const int I2C_FREQ_HZ = 400000;       // 400kHz Fast Mode
static const uint8_t MPU_ADDRS[] = {0x68, 0x69}; // MPU6050可能的I²C地址

// UART 配置（与主控通信）
static const uart_port_t LINK_UART = UART_NUM_1;
static const int LINK_TX_PIN = 45;    // → 主控RX(GPIO22)
static const int LINK_RX_PIN = 46;    // ← 主控TX(GPIO23)
static const int LINK_BAUD = 115200;
static const int LINK_PERIOD_MS = 20;  // 每20ms发送一次

// 陀螺仪标定
static const int GYRO_CALIB_SAMPLES = 80;  // 取80个样本计算零偏

// 直道/弯道判别阈值（带迟滞）
static const float STRAIGHT_GZ_DPS_TH = 5.0f;       // 进入直道阈值（°/s）
static const float STRAIGHT_GZ_DPS_EXIT_TH = 8.0f;   // 退出直道阈值（°/s）
static const int STRAIGHT_CONFIRM_CYCLES = 12;        // 连续12次确认直道
static const int STRAIGHT_LOST_CYCLES = 5;            // 连续5次确认弯道

// 状态LED
static const int STATUS_LED_PIN = 2;     // 板载LED
static const int BOOST_BLINK_PERIOD_MS = 160;  // 加速时LED闪烁周期
```

### 2. I²C 自动发现与 MPU6050 驱动

#### 为什么需要自动发现？

不同厂家的 GY-521（MPU6050 模块）引脚丝印不统一——有的 SDA=GPIO21/SCL=GPIO47，有的正好相反。自动发现机制解决了这个问题。

```cpp
static bool findMpu() {
    // 尝试两种引脚组合
    const int pin_sets[][2] = {
        {21, 47},  // 默认：SDA=GPIO21, SCL=GPIO47
        {47, 21},  // 交换：SDA=GPIO47, SCL=GPIO21
    };

    for (int p = 0; p < 2; p++) {
        int sda = pin_sets[p][0];
        int scl = pin_sets[p][1];

        if (!i2cStartDriver(sda, scl)) continue;  // 尝试启动I²C

        // 扫描I²C总线（地址1~127）
        for (int addr = 1; addr < 127; addr++) {
            if (probeAddr(addr)) {
                printf(" 0x%02X", addr);  // 打印找到的设备
            }
        }

        // 对每个可能的MPU地址，读WHO_AM_I寄存器(0x75)验证
        for (uint8_t addr : MPU_ADDRS) {
            uint8_t who = 0;
            readRegs(addr, 0x75, &who, 1);
            if (who == 0x68 || who == 0x69) {
                active_addr = addr;
                return true;  // 找到了！
            }
        }
    }
    return false;  // 两种组合都没找到
}
```

#### MPU6050 初始化

```cpp
static bool initMpu() {
    writeReg(active_addr, 0x6B, 0x00);  // 寄存器0x6B=0：唤醒MPU6050（退出睡眠）
    sleepMs(100);
    writeReg(active_addr, 0x1B, 0x00);  // 陀螺仪量程±250°/s
    writeReg(active_addr, 0x1C, 0x00);  // 加速度计量程±2g
    writeReg(active_addr, 0x1A, 0x03);  // 数字低通滤波DLPF=3（带宽约44Hz）
    sleepMs(100);
    return true;
}
```

> **MPU6050 关键寄存器**：
> | 寄存器 | 地址 | 说明 |
> |--------|------|------|
> | PWR_MGMT_1 | 0x6B | 写0x00唤醒芯片 |
> | GYRO_CONFIG | 0x1B | 设置陀螺仪量程 |
> | ACCEL_CONFIG | 0x1C | 设置加速度计量程 |
> | CONFIG | 0x1A | 设置DLPF低通滤波 |
> | WHO_AM_I | 0x75 | 固定返回0x68，用于验证芯片 |
> | ACCEL_XOUT_H | 0x3B | 传感器数据起始地址（连续14字节） |

#### 读取运动数据

```cpp
static bool readMotion(MotionData *motion) {
    uint8_t data[14] = {};
    readRegs(active_addr, 0x3B, data, 14);  // 从0x3B开始读14字节

    // 解析6个16位有符号数（大端序）
    motion->ax = readI16BE(&data[0])  / 16384.0f;  // ±2g → 16384 LSB/g
    motion->ay = readI16BE(&data[2])  / 16384.0f;
    motion->az = readI16BE(&data[4])  / 16384.0f;
    motion->gx = readI16BE(&data[8])  / 131.0f;    // ±250°/s → 131 LSB/(°/s)
    motion->gy = readI16BE(&data[10]) / 131.0f;
    motion->gz = readI16BE(&data[12]) / 131.0f;
    motion->temp = readI16BE(&data[6]) / 340.0f + 36.53f;

    return true;
}
```

> **数据格式**：MPU6050 的 14 字节数据布局：
> ```
> [0:1]  AX    [2:3]  AY    [4:5]  AZ
> [6:7]  TEMP  [8:9]  GX    [10:11] GY   [12:13] GZ
> ```

---

### 3. 陀螺仪零偏标定

```cpp
static void calibrateGyroBias() {
    printf("Calibrating gyro bias, keep S3 still...\n");
    float sum = 0.0f;
    int ok = 0;
    for (int i = 0; i < 80; i++) {   // 采集80个样本
        MotionData motion = {};
        if (readMotion(&motion)) {
            sum += motion.gz;
            ok++;
        }
        sleepMs(20);  // 每20ms采一次，总共约1.6秒
    }
    if (ok > 0) {
        gz_bias = sum / (float)ok;  // 零偏 = 平均值
    }
    // 实际使用时：实际角速度 = 原始读数 - 零偏
}
```

> **为什么要标定？** MPU6050 上电后即使静止，Z 轴陀螺仪也可能输出非零值（比如 0.5°/s），这叫零偏。如果不标定，小车会误判弯道。标定后 `实际gz = 原始gz - 零偏`，静止时输出趋近于 0。

---

### 4. 直道/弯道判别状态机

这是 S3 板最核心的算法：

```cpp
static void sendGyroToMain(const MotionData &motion) {
    float gz = motion.gz - gz_bias;  // 扣除零偏

    // ===== 状态判断（带迟滞） =====
    if (fabsf(gz) < 5.0f) {
        straight_count++;   // Z轴角速度<5°/s → 直道计数+1
        curve_count = 0;    // 弯道计数清零
    } else if (fabsf(gz) > 8.0f) {
        curve_count++;      // Z轴角速度>8°/s → 弯道计数+1
        straight_count = 0; // 直道计数清零
    }
    // 注：5~8°/s之间的角速度 → 两边都不计数（保持当前状态）

    // ===== 状态切换（带确认延时） =====
    if (straight_count >= 12) {   // 连续12周期(240ms)确认直道
        straight_state = true;
    }
    if (curve_count >= 5) {       // 连续5周期(100ms)确认弯道
        straight_state = false;
    }

    // ===== 发送结果 =====
    char line[8];
    int n = snprintf(line, sizeof(line), "%d\n", straight_state ? 1 : 0);
    uart_write_bytes(LINK_UART, line, n);  // 发送'1'或'0'给主控
}
```

> **迟滞（Hysteresis）的作用**：
> ```
> 进入直道需要 |gz| < 5°/s  持续240ms
> 退出直道需要 |gz| > 8°/s  持续100ms
> 中间的3°/s是"死区"——防止临界抖动
>
> 如果没有迟滞：
> gz在5°/s附近波动 → 直道↔弯道疯狂切换 → 小车速度忽快忽慢 → 失控
> ```

---

### 5. UART 通信与 LED 指示

#### 接收主控回传的指令

```cpp
static void parseMainCommand(const char *line) {
    // 主控发来 "BOOST=1" → 进入直道加速状态 → LED快闪
    // 主控发来 "BOOST=0" → 退出加速 → LED熄灭
    if (line前5个字符 == "BOOST") {
        boost_hint = (等号后面的数字 == '1');
    }
}
```

#### LED 状态更新

```cpp
static void statusLedUpdate() {
    // 超时保护：超过300ms没收到BOOST指令 → 自动熄灭LED
    if (超时) boost_hint = false;

    if (!boost_hint) {
        statusLedSet(false);  // LED熄灭
        return;
    }

    // 加速时LED以160ms周期闪烁（快闪）
    // 前80ms亮，后80ms灭
    TickType_t phase = now % 160ms;
    statusLedSet(phase < 80ms);  // 前半周期亮，后半周期灭
}
```

> **LED 指示灯行为**：
> - LED 熄灭 = 弯道或怠速
> - LED 快闪（约6Hz）= 直道加速中

---

### 6. 主函数流程

```cpp
extern "C" void app_main(void) {
    // 1. 初始化
    statusLedInit();  // GPIO2设为输出（板载LED）
    linkInit();       // 初始化UART1（与主控通信）

    // 2. 自动发现MPU6050
    if (!findMpu()) {
        printf("MPU6050 NOT FOUND.\n");
        while (1) sleepMs(1000);  // 找不到就死循环
    }

    // 3. 初始化MPU6050
    initMpu();  // 唤醒+配置量程+DLPF

    // 4. 陀螺仪零偏标定（1.6秒）
    calibrateGyroBias();

    // 5. 主循环（50Hz）
    while (1) {
        receiveMainCommands();  // 读主控回传的BOOST指令
        statusLedUpdate();      // 更新LED闪烁

        MotionData motion = {};
        if (readMotion(&motion)) {
            sendGyroToMain(motion);  // 判别直/弯→发送'0'/'1'
        }

        sleepMs(20);  // 等待20ms（50Hz）
    }
}
```

---

## 两板协作流程

```
    ESP32-S3 (姿态板)              ESP32 V1 (主控板)
    ════════════════              ════════════════
    
    上电→LED初始化                   上电→PWM/ADC/PCNT/UART初始化
    ↓                               ↓
    I²C自动发现MPU6050              等待2秒（安全延时）
    ├─ 尝试SDA=21,SCL=47            ↓
    └─ 失败则交换引脚               清零编码器
    ↓                               ↓
    MPU6050初始化                  ┌──── 主循环开始(50Hz) ────┐
    ↓                              │                         │
    陀螺仪零偏标定(80样本)          │ 1. 读UART ←────────────┼── '1'/'0'
    ↓                              │                         │
    ┌─── 主循环(50Hz) ───┐         │ 2. 读编码器→speed_l/r    │
    │                    │         │                         │
    │ 读MPU6050          │         │ 3. 读5路ADC→归一化→pos  │
    │ ↓                  │         │                         │
    │ gz-gz_bias→判别    │  UART   │ 4. 丢线检查/保持        │
    │ ├─ |gz|<5 →直道++  │────────→│                         │
    │ └─ |gz|>8 →弯道++  │  '1'   │ 5. 弯道减速/直道加速    │
    │ ↓                  │  '0'   │                         │
    │ 发送'1'或'0'       │         │ 6. 外环PD→delta_speed   │
    │ ↓                  │         │                         │
    │ 收主控BOOST→LED    │← ─ ─ ─ ─│    发送 "BOOST=1/0"     │
    │ ↓                  │         │                         │
    │ LED快闪/熄灭        │         │ 7. 内环PI→PWM→电机      │
    └────────────────────┘         │                         │
                                   │ 8. 调试打印             │
                                   └─────────────────────────┘
```

> **信息流总结**：
> - S3 → 主控（每20ms）：`'1'`（直道）或 `'0'`（弯道）
> - 主控 → S3（需要时）：`"BOOST=1\n"`（进入加速）或 `"BOOST=0\n"`（退出加速）
> - S3 收到 BOOST=1 → LED 开始快闪
> - 主控收到 `'1'` + 光电偏差确认 → 开始加速（slew rate 平滑过渡）

---

## 关键算法深入

### 模拟归一化算法

```
原始ADC ──→ 归一化 ──→ 强度值(0~1)

每个传感器独立标定：
  white[i] = 传感器i在白底上的ADC读数（≈4095）
  black[i] = 传感器i在黑线上的ADC读数（≈1200）

归一化公式：
  intensity[i] = (white[i] - raw[i]) / (white[i] - black[i])

物理意义：
  intensity=1.0 = "这个是纯黑的！"（传感器正好在黑线上）
  intensity=0.0 = "这个是纯白的！"（传感器完全在白底上）
  intensity=0.5 = "有点灰..."（传感器在黑线边缘）

优点（对比传统二值化）：
  ✅ 保留了"有多黑"的连续信息
  ✅ 不同传感器的个体差异被归一化消除
  ✅ 环境光变化时只需更新white/black基准值
  ✅ 为加权重心法提供高质量输入
```

### 加权重心法

```
传感器： L2      L1      C       R1      R2
位置：  -18mm   -9mm    0mm     +9mm    +18mm
权重：   -2      -1      0       +1      +2

公式：
  pos = Σ(intensity[i] × weight[i]) / Σ(intensity[i])

举例计算：
  假设各传感器读数为：
  L2: ADC=4000 → intensity=0.1 (偏白)
  L1: ADC=3500 → intensity=0.4 (偏灰)
  C:  ADC=2000 → intensity=0.9 (很黑！)
  R1: ADC=3800 → intensity=0.2 (偏白)
  R2: ADC=4000 → intensity=0.1 (偏白)

  weighted_sum = 0.1×(-2) + 0.4×(-1) + 0.9×0 + 0.2×1 + 0.1×2
               = -0.2 + (-0.4) + 0 + 0.2 + 0.2
               = -0.2

  intensity_sum = 0.1+0.4+0.9+0.2+0.1 = 1.7

  pos = -0.2 / 1.7 ≈ -0.12

  解读：黑线略微偏左（C传感器稍微偏左的位置）
  物理偏移 ≈ -0.12 × 9mm ≈ -1mm（向左偏了约1mm）
```

### 双环串级 PID

```
           外环 PD                        内环 PI
           ═══════                       ═══════
目标：     保持黑线居中                   保持轮速=设定值
输入：     线偏差 pos(-2~+2)              速度误差(speed_ref - speed_actual)
输出：     目标速度差 delta_speed          PWM占空比

为什么外环不用 I？
  → 弯道中的偏差是正常的状态信号，不是"需要消除的稳态误差"
  → 使用 I 会在弯道中累积积分 → 出弯后大幅回摆
  → 外环用 PD 就够了：P快速响应，D抑制震荡

为什么内环用 I？
  → 左右轮机械差异是持续的稳态误差
  → 用 I 能自动消除这种不对称
  → 加上抗饱和机制，不会失控

外环公式：
  delta = Kp×error + Kd×d(error)/dt + K_nonlinear×error×|error|
       = 12000×error + 4200×d(error)/dt + 18900×error×|error|

  |error|小 → 线性项为主，温和修正
  |error|大 → 平方项主导，强力拉回

内环公式：
  PWM = 前馈 + Kp×error + Ki×∫error·dt
  前馈：基于实测PWM-速度模型的"猜测值"
  反馈：修正前馈的误差（约占10-15%）
```

### 弯道减速策略

```
基础速度 = BASE_SPEED_PPS + 陀螺仪直道加速 - CURVE_SLOWDOWN_GAIN × |偏差|

举例（弯道中）：
  BASE_SPEED = 48000
  直道加速 = 0（弯道中不加速）
  |偏差| = 1.2（较大弯道）
  
  base_speed = 48000 - 22500 × 1.2
             = 48000 - 27000
             = 21000 → 但下限保护CURVE_MIN_SPEED_PPS=19200
             
  实际速度 ≈ 21000pps（直道速度的44%，安全过弯）

举例（直道中）：
  BASE_SPEED = 48000
  直道加速 = 18000（陀螺仪确认直道）
  |偏差| = 0.05（基本居中）
  
  boosted_base = 48000 + 18000 = 66000
  base_speed = 66000 - 22500 × 0.05
             = 66000 - 1125
             ≈ 64875pps（接近全速）
```

### 丢线恢复策略

```
               在线检测                        丢线检测
               ════════                       ════════
         Σ(intensity) > 0.10           Σ(intensity) ≤ 0.10
         
         
阶段1：短暂丢线（≤2周期，≤40ms）
  → 保持上次偏差继续转向
  → 速度不变
  → 假设：黑线还在附近，马上就能重新检测到
  
  
阶段2：持续丢线（>2周期，>40ms）
  → 降速至 LOST_BASE_SPEED_PPS(13500pps)
  → 施加固定偏差(±1.5)，根据上次偏差方向
  → 小车以弧线方式"搜索"黑线
  → 一旦传感器重新检测到 → 立即恢复正常控制
  
上次偏右 → 继续右转寻找
上次偏左 → 继续左转寻找
```

---

## 参数速查表

### main.cpp 核心参数

| 参数 | 值 | 说明 |
|------|------|------|
| CONTROL_DT_MS | 20 | 控制周期20ms(50Hz) |
| BASE_SPEED_PPS | 48000 | 直道基础速度 |
| KP_LINE | 12000 | 外环P增益 |
| KD_LINE | 4200 | 外环D增益 |
| NONLINEAR_TURN_GAIN | 18900 | 非线性转向增益 |
| DELTA_SPEED_MAX | 26000 | 速度差限幅 |
| CURVE_SLOWDOWN_GAIN | 22500 | 弯道减速系数 |
| CURVE_MIN_SPEED_PPS | 19200 | 弯道最低速度 |
| LOST_BASE_SPEED_PPS | 13500 | 丢线后速度 |
| GYRO_SPEED_BOOST_PPS | 18000 | 直道加速增量 |
| LINE_LOST_HOLD_CYCLES | 2 | 丢线保持周期数 |
| POS_FILTER_ALPHA | 0.20 | 偏差EMA滤波系数 |
| LINE_ERROR_DEADBAND | 0.08 | 偏差死区 |
| KP_SPEED_L / KP_SPEED_R | 0.040 / 0.038 | 内环P增益 |
| KI_SPEED_L / KI_SPEED_R | 0.300 / 0.290 | 内环I增益 |
| LEFT_DEAD_PWM / RIGHT_DEAD_PWM | 270 / 295 | 电机死区PWM |
| LEFT_KM_PPS_PER_PWM | 90 | 左轮PWM→速度斜率 |
| RIGHT_KM_PPS_PER_PWM | 95 | 右轮PWM→速度斜率 |

### main(1).cpp 核心参数

| 参数 | 值 | 说明 |
|------|------|------|
| I2C_FREQ_HZ | 400000 | I²C Fast Mode |
| LINK_PERIOD_MS | 20 | 向主控发送周期 |
| GYRO_CALIB_SAMPLES | 80 | 零偏标定样本数 |
| STRAIGHT_GZ_DPS_TH | 5.0 | 进入直道角速度阈值 |
| STRAIGHT_GZ_DPS_EXIT_TH | 8.0 | 退出直道角速度阈值 |
| STRAIGHT_CONFIRM_CYCLES | 12 | 直道确认周期数(240ms) |
| STRAIGHT_LOST_CYCLES | 5 | 弯道确认周期数(100ms) |

---

## 调参建议

如果你需要调整参数让小车跑得更好，按以下顺序来：

1. **先确认传感器标定正确**：刷标定固件，把每个传感器的 white/black 值填入代码
2. **确认编码器方向**：手动推车，看串口打印的 speed_l/speed_r 是否为正
3. **调整基础速度**：从低速(30000pps)开始，跑稳了再逐步提高
4. **调整外环 Kp**：如果小车在直道上S形摆动，说明Kp太大；如果过弯迟钝，说明Kp太小
5. **调整外环 Kd**：如果小车在直道上抖动，增大Kd；如果过弯转向不足，减小Kd
6. **调整弯道减速**：如果过弯冲出赛道，增大 CURVE_SLOWDOWN_GAIN
7. **调整直道加速**：如果加速时失稳，减小 GYRO_SPEED_BOOST_PPS 或增大 STRAIGHT_CONFIRM_CYCLES

---

*文档基于实际代码生成，所有参数值均来自 main.cpp 和 main(1).cpp 的实际定义。*
