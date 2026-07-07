#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_idf_version.h"

#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"

// ============================================================
// 双环循线版本：模拟归一化 + 短时丢线保持
// 基于 motor(1).c，适配本车硬件引脚/编码器方向/传感器顺序
// ============================================================




// ============================================================
// 1. 引脚定义
// ============================================================

// 电机 A = 左轮  （注意：本车 IN1_A=GPIO15，IN2_A=GPIO13，与 motor(1).c 相反）
#define PIN_IN1_A 15
#define PIN_IN2_A 13

// 电机 B = 右轮  （右轮前进 = IN1_B/GPIO14 给 PWM；IN2_B/GPIO25 给0）
#define PIN_IN1_B 14
#define PIN_IN2_B 25

// 编码器（引脚与 motor(1).c 相同）
#define PIN_E1A 16
#define PIN_E1B 17
#define PIN_E2A 18
#define PIN_E2B 19

// 循迹传感器，从左到右：L2=GPIO34, L1=GPIO35, C=GPIO32, R1=GPIO33, R2=GPIO27
#define TRACK_GPIO_1 34
#define TRACK_GPIO_2 35
#define TRACK_GPIO_3 32
#define TRACK_GPIO_4 33
#define TRACK_GPIO_5 27

// S3 陀螺仪板通信：S3 TX -> 本板 GPIO22，S3 RX <- 本板 GPIO23，两块板必须共地
#define GYRO_UART_NUM      UART_NUM_2
#define GYRO_UART_RX_PIN   22
#define GYRO_UART_TX_PIN   23
#define GYRO_UART_BAUD     115200




// ============================================================
// 2. 调参区域
// ============================================================

// A. 串口调试
#define DEBUG_PRINT_EVERY_N 5


// B. 硬件设置
// ---------------- PWM ----------------
#define PWM_FREQ_HZ        1000
#define PWM_RESOLUTION     LEDC_TIMER_10_BIT
#define PWM_ABS_LIMIT      1023.0f
#define PWM_MAX_DUTY       900.0f

// ---------------- 有限反拖 ----------------
#define REVERSE_SPEED_REF_MAX  6500.0f
#define PWM_REV_MAX_DUTY       550.0f
#define REV_DEAD_PWM           150.0f

#define LEFT_DEAD_PWM      270.0f
#define RIGHT_DEAD_PWM     295.0f

#define MIN_ACTIVE_SPEED_REF  1000.0f

// ---------------- 编码器方向（实测：前进时左轮 cnt 为正、右轮 cnt 为负） ----------------
// 目标：前进（ref>0）时 speed_l / speed_r 都必须为正，否则速度环误差爆表、PWM 顶满失控
#define LEFT_ENC_SIGN       1.0f
#define RIGHT_ENC_SIGN     -1.0f


// C. 系统控制
// ---------------- 控制周期 ----------------
#define CONTROL_DT_MS      20
#define CONTROL_DT_S       0.020f

// ---------------- 循线外环 PD ----------------
// 以下参数基于本车 btlog 实测版本
#define BASE_SPEED_PPS     48000.0f
#define SPEED_REF_MAX      88000.0f

#define KP_LINE            12000.0f
#define KD_LINE            4200.0f
#define NONLINEAR_TURN_GAIN 18900.0f
#define DELTA_SPEED_MAX    26000.0f

// 如果小车会远离黑线，把这里改为 -1.0f
#define LINE_CONTROL_SIGN  1.0f

#define POS_FILTER_ALPHA   0.20f
#define LINE_ERROR_DEADBAND 0.08f

// 弯道减速
#define CURVE_SLOWDOWN_GAIN 22500.0f
#define CURVE_MIN_SPEED_PPS 19200.0f

// 丢线处理
#define STOP_WHEN_LINE_LOST  0
#define LOST_BASE_SPEED_PPS  13500.0f
#define LINE_LOST_HOLD_CYCLES 2

// ---------------- 速度内环 PI ----------------
#define KP_SPEED_L         0.040f
#define KI_SPEED_L         0.300f

#define KP_SPEED_R         0.038f
#define KI_SPEED_R         0.290f

#define SPEED_INTEGRAL_LIMIT    6000.0f

// ---------------- 前馈（低速工作点，PWM=400 时实测） ----------------
#define LEFT_FF_PWM_BASE            557.0f
#define LEFT_SPEED_AT_FF_PWM_BASE   48000.0f
#define LEFT_KM_PPS_PER_PWM         90.0f

#define RIGHT_FF_PWM_BASE           557.0f
#define RIGHT_SPEED_AT_FF_PWM_BASE  48000.0f
#define RIGHT_KM_PPS_PER_PWM        95.0f


// D. 五路循迹传感器标定
// ！！重要：以下是基于你车传感器的估算值（白底~45，黑线~115）
// 请先刷传感器测试固件，将探头分别对准白底和黑线，
// 用串口打印的 min/max 填入对应位置（索引 0=GPIO34 到 4=GPIO27）
// ！！
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define ADC_ATTEN_USED      ADC_ATTEN_DB_12
#else
#define ADC_ATTEN_USED      ADC_ATTEN_DB_11
#endif

//                             [L2]  [L1]  [C ]  [R1]  [R2]
//                            GPIO34 GPIO35 GPIO32 GPIO33 GPIO27
// 极性：白=高ADC(~4095)，黑=低ADC(~1200)；归一化用 (white-raw)/(white-black)
static int sensor_white[5] = {4095, 4095, 4095, 4095, 4095};
static int sensor_black[5] = {1200, 1200, 1200, 1200, 1200};

// 五个探头归一化强度之和超过此值才认为看到黑线
// 本车传感器弱，用 0.10（比 motor(1).c 的 0.15 更宽松）
#define LINE_SUM_MIN       0.10f




// ============================================================
// 3. 全局句柄和状态
// ============================================================

static const char *TAG = "LINE_CAR";

#define LEDC_MODE_USED      LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_USED     LEDC_TIMER_0

#define CH_LEFT_IN1         LEDC_CHANNEL_0
#define CH_LEFT_IN2         LEDC_CHANNEL_1
#define CH_RIGHT_IN1        LEDC_CHANNEL_2
#define CH_RIGHT_IN2        LEDC_CHANNEL_3

static pcnt_unit_handle_t pcnt_left_unit = NULL;
static pcnt_unit_handle_t pcnt_right_unit = NULL;

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_oneshot_unit_handle_t adc2_handle = NULL;

typedef struct {
    float kp;
    float ki;
    float integral;
    float integral_limit;
} PI_Controller;

static PI_Controller pi_left = {
    .kp = KP_SPEED_L,
    .ki = KI_SPEED_L,
    .integral = 0.0f,
    .integral_limit = SPEED_INTEGRAL_LIMIT,
};

static PI_Controller pi_right = {
    .kp = KP_SPEED_R,
    .ki = KI_SPEED_R,
    .integral = 0.0f,
    .integral_limit = SPEED_INTEGRAL_LIMIT,
};




// ============================================================
// 4. 工具函数
// ============================================================

static inline float clampf_local(float x, float minValue, float maxValue)
{
    if (x < minValue) return minValue;
    if (x > maxValue) return maxValue;
    return x;
}

static inline float slew_to(float current, float target, float max_step)
{
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}

static void ledc_set_channel_duty(ledc_channel_t channel, float duty)
{
    duty = clampf_local(duty, 0.0f, PWM_ABS_LIMIT);
    uint32_t duty_u32 = (uint32_t)(duty + 0.5f);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE_USED, channel, duty_u32));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE_USED, channel));
}

// 左轮可反转
static void motor_left_set_pwm(float pwm)
{
    if (pwm > 0.0f) {
        pwm = clampf_local(pwm, 0.0f, PWM_MAX_DUTY);
        ledc_set_channel_duty(CH_LEFT_IN2, 0.0f);
        ledc_set_channel_duty(CH_LEFT_IN1, pwm);
    } else if (pwm < 0.0f) {
        pwm = clampf_local(pwm, -PWM_REV_MAX_DUTY, 0.0f);
        ledc_set_channel_duty(CH_LEFT_IN1, 0.0f);
        ledc_set_channel_duty(CH_LEFT_IN2, -pwm);
    } else {
        ledc_set_channel_duty(CH_LEFT_IN1, 0.0f);
        ledc_set_channel_duty(CH_LEFT_IN2, 0.0f);
    }
}

// 右轮可反转（前进 = CH_RIGHT_IN2/GPIO25 给 PWM，右电机整体反向）
static void motor_right_set_pwm(float pwm)
{
    if (pwm > 0.0f) {
        pwm = clampf_local(pwm, 0.0f, PWM_MAX_DUTY);
        ledc_set_channel_duty(CH_RIGHT_IN1, 0.0f);
        ledc_set_channel_duty(CH_RIGHT_IN2, pwm);
    } else if (pwm < 0.0f) {
        pwm = clampf_local(pwm, -PWM_REV_MAX_DUTY, 0.0f);
        ledc_set_channel_duty(CH_RIGHT_IN2, 0.0f);
        ledc_set_channel_duty(CH_RIGHT_IN1, -pwm);
    } else {
        ledc_set_channel_duty(CH_RIGHT_IN1, 0.0f);
        ledc_set_channel_duty(CH_RIGHT_IN2, 0.0f);
    }
}

static void motors_stop(void)
{
    ledc_set_channel_duty(CH_LEFT_IN1, 0.0f);
    ledc_set_channel_duty(CH_LEFT_IN2, 0.0f);
    ledc_set_channel_duty(CH_RIGHT_IN1, 0.0f);
    ledc_set_channel_duty(CH_RIGHT_IN2, 0.0f);
}




// ============================================================
// 5. 速度环：前馈 + 死区 + PI
// ============================================================

static float feedforward_left_pwm(float speed_ref)
{
    float pwm = LEFT_FF_PWM_BASE + (speed_ref - LEFT_SPEED_AT_FF_PWM_BASE) / LEFT_KM_PPS_PER_PWM;
    return pwm;
}

static float feedforward_right_pwm(float speed_ref)
{
    float pwm = RIGHT_FF_PWM_BASE + (speed_ref - RIGHT_SPEED_AT_FF_PWM_BASE) / RIGHT_KM_PPS_PER_PWM;
    return pwm;
}

static float feedforward_left_pwm_signed(float speed_ref)
{
    if (speed_ref > MIN_ACTIVE_SPEED_REF) {
        return feedforward_left_pwm(speed_ref);
    }
    return 0.0f;
}

static float feedforward_right_pwm_signed(float speed_ref)
{
    if (speed_ref > MIN_ACTIVE_SPEED_REF) {
        return feedforward_right_pwm(speed_ref);
    }
    return 0.0f;
}

static float apply_signed_deadzone(
    float pwm,
    float ref_speed,
    float forward_dead_pwm,
    float pwm_min,
    float pwm_max
)
{
    if (fabsf(ref_speed) < MIN_ACTIVE_SPEED_REF) {
        return 0.0f;
    }

    if (ref_speed > 0.0f) {
        if (pwm > 0.0f && pwm < forward_dead_pwm) {
            pwm = forward_dead_pwm;
        }
    } else if (ref_speed < 0.0f) {
        if (pwm < 0.0f && -pwm < REV_DEAD_PWM) {
            pwm = -REV_DEAD_PWM;
        } else if (pwm > 0.0f && pwm < REV_DEAD_PWM) {
            pwm = REV_DEAD_PWM;
        }
    }

    return clampf_local(pwm, pwm_min, pwm_max);
}

static float pi_speed_update(
    PI_Controller *pi,
    float error,
    float dt,
    float ff_pwm,
    float ref_speed,
    float dead_pwm
)
{
    if (fabsf(ref_speed) < MIN_ACTIVE_SPEED_REF) {
        pi->integral = 0.0f;
        return 0.0f;
    }

    float pwm_min = 0.0f;
    float pwm_max = PWM_MAX_DUTY;

    if (ref_speed < -MIN_ACTIVE_SPEED_REF) {
        pwm_min = -PWM_REV_MAX_DUTY;
        pwm_max = PWM_REV_MAX_DUTY;
    }

    float candidate_integral = pi->integral + error * dt;
    candidate_integral = clampf_local(
        candidate_integral,
        -pi->integral_limit,
        pi->integral_limit
    );

    float unsat_pwm =
        ff_pwm +
        pi->kp * error +
        pi->ki * candidate_integral;

    bool high_sat = (unsat_pwm > pwm_max) && (error > 0.0f);
    bool low_sat  = (unsat_pwm < pwm_min) && (error < 0.0f);

    if (!high_sat && !low_sat) {
        pi->integral = candidate_integral;
    }

    float pwm =
        ff_pwm +
        pi->kp * error +
        pi->ki * pi->integral;

    pwm = clampf_local(pwm, pwm_min, pwm_max);
    pwm = apply_signed_deadzone(pwm, ref_speed, dead_pwm, pwm_min, pwm_max);

    return pwm;
}




// ============================================================
// 6. 硬件初始化
// ============================================================

static void init_pwm(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE_USED,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_USED,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch_conf[4] = {
        {
            .gpio_num = PIN_IN1_A,
            .speed_mode = LEDC_MODE_USED,
            .channel = CH_LEFT_IN1,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_USED,
            .duty = 0,
            .hpoint = 0,
        },
        {
            .gpio_num = PIN_IN2_A,
            .speed_mode = LEDC_MODE_USED,
            .channel = CH_LEFT_IN2,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_USED,
            .duty = 0,
            .hpoint = 0,
        },
        {
            .gpio_num = PIN_IN1_B,
            .speed_mode = LEDC_MODE_USED,
            .channel = CH_RIGHT_IN1,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_USED,
            .duty = 0,
            .hpoint = 0,
        },
        {
            .gpio_num = PIN_IN2_B,
            .speed_mode = LEDC_MODE_USED,
            .channel = CH_RIGHT_IN2,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_USED,
            .duty = 0,
            .hpoint = 0,
        },
    };

    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ch_conf[i]));
    }

    motors_stop();
}

static void init_one_pcnt_unit_signed(
    pcnt_unit_handle_t *unit_out,
    int pulse_gpio,
    int dir_gpio
)
{
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, unit_out));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*unit_out, &filter_config));

    pcnt_channel_handle_t chan = NULL;
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = pulse_gpio,
        .level_gpio_num = dir_gpio,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(*unit_out, &chan_config, &chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD
    ));

    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    ));

    ESP_ERROR_CHECK(pcnt_unit_enable(*unit_out));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*unit_out));
    ESP_ERROR_CHECK(pcnt_unit_start(*unit_out));
}

static void init_pcnt(void)
{
    init_one_pcnt_unit_signed(&pcnt_left_unit, PIN_E1A, PIN_E1B);
    init_one_pcnt_unit_signed(&pcnt_right_unit, PIN_E2A, PIN_E2B);
}

static int read_and_clear_pcnt(pcnt_unit_handle_t unit)
{
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &count));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    return count;
}

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_USED,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    // 传感器顺序（左→右）: GPIO34 GPIO35 GPIO32 GPIO33 GPIO27
    // 对应 ADC 通道:       1_CH6  1_CH7  1_CH4  1_CH5  2_CH7
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &chan_cfg));  // GPIO34
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &chan_cfg));  // GPIO35
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &chan_cfg));  // GPIO32
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &chan_cfg));  // GPIO33
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_7, &chan_cfg));  // GPIO27
}

static int read_track_adc(int index)
{
    int raw = 0;

    // index 0=GPIO34(L2) … 4=GPIO27(R2)，从左到右
    switch (index) {
        case 0: ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &raw)); break;  // GPIO34
        case 1: ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &raw)); break;  // GPIO35
        case 2: ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &raw)); break;  // GPIO32
        case 3: ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &raw)); break;  // GPIO33
        case 4: ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_7, &raw)); break;  // GPIO27
        default: raw = 0; break;
    }

    return raw;
}




// ============================================================
// 7. 循线传感器处理：模拟归一化
// ============================================================

typedef struct {
    int raw[5];
    float intensity[5];
    float pos;
    float sum;
    bool valid;
} line_data_t;

static float normalize_sensor_to_black(int raw, int idx)
{
    float white = (float)sensor_white[idx];
    float black = (float)sensor_black[idx];
    float denom = white - black;  // 白>黑，极性：高ADC=白，低ADC=黑

    if (fabsf(denom) < 1.0f) {
        return 0.0f;
    }

    float x = (white - (float)raw) / denom;
    return clampf_local(x, 0.0f, 1.0f);
}

static line_data_t read_line_sensors(void)
{
    line_data_t d = {};

    const float weight[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

    float weighted_sum = 0.0f;
    float intensity_sum = 0.0f;

    for (int i = 0; i < 5; i++) {
        d.raw[i] = read_track_adc(i);
        d.intensity[i] = normalize_sensor_to_black(d.raw[i], i);

        weighted_sum += d.intensity[i] * weight[i];
        intensity_sum += d.intensity[i];
    }

    d.sum = intensity_sum;

    if (intensity_sum > LINE_SUM_MIN) {
        d.pos = weighted_sum / intensity_sum;
        d.valid = true;
    } else {
        d.pos = 0.0f;
        d.valid = false;
    }

    return d;
}




// ============================================================
// 8. 主程序 —— motor(1).c 风格：目标速度 + 前馈 + 编码器 PI
// ============================================================

// S3 发送 1/0；直线时只提高“目标速度”，不直接给 PWM 加量。
#define GYRO_STRAIGHT_ENABLE          1
#define GYRO_STRAIGHT_TIMEOUT_MS      250
#define GYRO_LINE_ERROR_TH            0.45f
#define GYRO_STRAIGHT_CONFIRM         5
#define GYRO_SPEED_BOOST_PPS          18000.0f
#define GYRO_SPEED_BOOST_STEP_PPS     1200.0f

static void init_gyro_uart(void)
{
    uart_config_t config = {};
    config.baud_rate = GYRO_UART_BAUD;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(GYRO_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GYRO_UART_NUM, &config));
    ESP_ERROR_CHECK(uart_set_pin(
        GYRO_UART_NUM,
        GYRO_UART_TX_PIN,
        GYRO_UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));
}

static bool read_gyro_straight_uart(bool *straight_out, int64_t *last_update_us)
{
    uint8_t data[32];
    int len = uart_read_bytes(GYRO_UART_NUM, data, sizeof(data), 0);
    bool updated = false;

    for (int i = 0; i < len; i++) {
        if (data[i] == '1') {
            *straight_out = true;
            *last_update_us = esp_timer_get_time();
            updated = true;
        } else if (data[i] == '0') {
            *straight_out = false;
            *last_update_us = esp_timer_get_time();
            updated = true;
        }
    }

    return updated;
}

extern "C" void app_main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    ESP_LOGI(TAG, "Init PWM...");
    init_pwm();

    ESP_LOGI(TAG, "Init ADC...");
    init_adc();

    ESP_LOGI(TAG, "Init encoder PCNT...");
    init_pcnt();

    ESP_LOGI(TAG, "Init gyro UART...");
    init_gyro_uart();

    ESP_LOGI(TAG, "Closed-loop line follower started.");
    ESP_LOGI(TAG, "Motors start after 2 seconds.");

    motors_stop();
    vTaskDelay(pdMS_TO_TICKS(2000));

    read_and_clear_pcnt(pcnt_left_unit);
    read_and_clear_pcnt(pcnt_right_unit);

    float pos_filtered = 0.0f;
    float prev_line_error = 0.0f;
    float d_line_filtered = 0.0f;
    float last_valid_pos = 0.0f;
    bool  have_last_line = false;
    int   short_lost_count = 0;
    int   loop_count = 0;
    bool  gyro_straight = false;
    int64_t gyro_last_update_us = 0;
    int   straight_count = 0;
    float gyro_speed_boost = 0.0f;
    int64_t last_us = esp_timer_get_time();

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_DT_MS));

        read_gyro_straight_uart(&gyro_straight, &gyro_last_update_us);
        int64_t now_us = esp_timer_get_time();
        bool gyro_fresh = (gyro_last_update_us > 0) &&
                          ((now_us - gyro_last_update_us) < (int64_t)GYRO_STRAIGHT_TIMEOUT_MS * 1000);
        bool gyro_straight_now = gyro_fresh && gyro_straight;

        float dt = (float)(now_us - last_us) / 1000000.0f;
        last_us = now_us;
        if (dt <= 0.0f || dt > 0.1f) {
            dt = CONTROL_DT_S;
        }

        // ---------------- 1. 编码器速度 ----------------
        int cnt_l = read_and_clear_pcnt(pcnt_left_unit);
        int cnt_r = read_and_clear_pcnt(pcnt_right_unit);
        float speed_l = LEFT_ENC_SIGN * (float)cnt_l / dt;
        float speed_r = RIGHT_ENC_SIGN * (float)cnt_r / dt;

        // ---------------- 2. 读线 ----------------
        line_data_t line = read_line_sensors();
        bool raw_line_valid = line.valid;
        bool hold_line = false;

        if (line.valid) {
            have_last_line = true;
            last_valid_pos = line.pos;
            short_lost_count = 0;
        } else if (have_last_line && short_lost_count < LINE_LOST_HOLD_CYCLES) {
            short_lost_count++;
            line.valid = true;
            line.pos = last_valid_pos;
            hold_line = true;
        }

        // ---------------- 3. base speed + 循迹误差 ----------------
        float base_speed = BASE_SPEED_PPS;
        float line_error = 0.0f;

        if (line.valid) {
            pos_filtered = POS_FILTER_ALPHA * pos_filtered + (1.0f - POS_FILTER_ALPHA) * line.pos;
            line_error = -pos_filtered;
            if (fabsf(line_error) < LINE_ERROR_DEADBAND) {
                line_error = 0.0f;
            }

            bool line_is_straight = fabsf(line_error) < GYRO_LINE_ERROR_TH;
            if (gyro_straight_now && line_is_straight) {
                straight_count++;
            } else {
                straight_count = 0;
            }

#if GYRO_STRAIGHT_ENABLE
            float boost_target = (straight_count >= GYRO_STRAIGHT_CONFIRM) ? GYRO_SPEED_BOOST_PPS : 0.0f;
            gyro_speed_boost = slew_to(gyro_speed_boost, boost_target, GYRO_SPEED_BOOST_STEP_PPS);
#else
            gyro_speed_boost = 0.0f;
#endif

            float boosted_base = BASE_SPEED_PPS + gyro_speed_boost;
            base_speed = boosted_base - CURVE_SLOWDOWN_GAIN * fabsf(pos_filtered);
            base_speed = clampf_local(base_speed, CURVE_MIN_SPEED_PPS, boosted_base);
        } else {
            base_speed = LOST_BASE_SPEED_PPS;
            if (prev_line_error > 0.05f) {
                line_error = 1.5f;
            } else if (prev_line_error < -0.05f) {
                line_error = -1.5f;
            } else {
                line_error = 0.0f;
            }
            straight_count = 0;
            gyro_speed_boost = slew_to(gyro_speed_boost, 0.0f, GYRO_SPEED_BOOST_STEP_PPS);
        }

        // ---------------- 4. 外环 PD -> 左右轮目标速度 ----------------
        float d_line = (line_error - prev_line_error) / dt;
        d_line_filtered = 0.75f * d_line_filtered + 0.25f * d_line;
        float nonlinear = NONLINEAR_TURN_GAIN * line_error * fabsf(line_error);
        float delta_speed_ref = LINE_CONTROL_SIGN * (KP_LINE * line_error + KD_LINE * d_line_filtered + nonlinear);
        delta_speed_ref = clampf_local(delta_speed_ref, -DELTA_SPEED_MAX, DELTA_SPEED_MAX);
        prev_line_error = line_error;

        float speed_l_ref = base_speed - delta_speed_ref;
        float speed_r_ref = base_speed + delta_speed_ref;
        speed_l_ref = clampf_local(speed_l_ref, -REVERSE_SPEED_REF_MAX, SPEED_REF_MAX);
        speed_r_ref = clampf_local(speed_r_ref, -REVERSE_SPEED_REF_MAX, SPEED_REF_MAX);

        // ---------------- 5. 速度 PI -> PWM ----------------
        float err_l = speed_l_ref - speed_l;
        float err_r = speed_r_ref - speed_r;
        float ff_l = feedforward_left_pwm_signed(speed_l_ref);
        float ff_r = feedforward_right_pwm_signed(speed_r_ref);
        float pwm_l = pi_speed_update(&pi_left, err_l, dt, ff_l, speed_l_ref, LEFT_DEAD_PWM);
        float pwm_r = pi_speed_update(&pi_right, err_r, dt, ff_r, speed_r_ref, RIGHT_DEAD_PWM);

        motor_left_set_pwm(pwm_l);
        motor_right_set_pwm(pwm_r);

        // ---------------- 3. 调试打印 ----------------
        loop_count++;
        if (loop_count % DEBUG_PRINT_EVERY_N == 0) {
            printf(
                "raw_valid=%d valid=%d hold=%d lost_cnt=%d gyro=%d fresh=%d sc=%d gboost=%.0f "
                "pos=%.3f pos_f=%.3f err=%.3f sum=%.3f base=%.0f delta=%.0f | "
                "L ref=%.0f spd=%.0f cnt=%d pwm=%.1f | "
                "R ref=%.0f spd=%.0f cnt=%d pwm=%.1f | "
                "raw=[%d,%d,%d,%d,%d] norm=[%.2f,%.2f,%.2f,%.2f,%.2f]\n",
                raw_line_valid ? 1 : 0,
                line.valid ? 1 : 0,
                hold_line ? 1 : 0,
                short_lost_count,
                gyro_straight ? 1 : 0,
                gyro_fresh ? 1 : 0,
                straight_count,
                gyro_speed_boost,
                line.pos, pos_filtered, line_error, line.sum,
                base_speed, delta_speed_ref,
                speed_l_ref, speed_l, cnt_l, pwm_l,
                speed_r_ref, speed_r, cnt_r, pwm_r,
                line.raw[0], line.raw[1], line.raw[2], line.raw[3], line.raw[4],
                line.intensity[0], line.intensity[1], line.intensity[2],
                line.intensity[3], line.intensity[4]
            );
        }
    }
}
