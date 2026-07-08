#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "driver/uart.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef APP_HD_STREAM

static const uart_port_t LINK_UART = UART_NUM_1;
static const int LINK_TX_PIN = 45;
static const int LINK_RX_PIN = 46;
static const int LINK_BAUD = 115200;

static const uint8_t MPU_ADDRS[] = {0x68, 0x69};

static const int LED_PIN = 2;

static bool mpu_ready = false;
static uint8_t mpu_addr = 0;
static bool i2c_ready = false;
static bool link_ready = false;
static int straight_frames = 0;
static int turn_frames = 0;
static int turn_dir_hold = 0;

struct MotionData {
    float gz;
};

struct CamData {
    float p;
    float a;
    float c;
    float q;
    float w;
    bool lost;
    bool boost;
    int turn_dir;
    int contrast;
    int valid_rows;
};

static float clampf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void sleep_ms(int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void led_init(void)
{
    gpio_config_t cfg = {};
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pin_bit_mask = 1ULL << LED_PIN;
    gpio_config(&cfg);
    gpio_set_level((gpio_num_t)LED_PIN, 0);
}

static void link_init(void)
{
    uart_config_t cfg = {};
    cfg.baud_rate = LINK_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    if (uart_param_config(LINK_UART, &cfg) != ESP_OK) return;
    if (uart_set_pin(LINK_UART, LINK_TX_PIN, LINK_RX_PIN,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return;
    esp_err_t err = uart_driver_install(LINK_UART, 1024, 256, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return;
    link_ready = true;
}

static void send_boot_status(float p, float q)
{
    if (!link_ready) return;
    char line[96];
    int n = snprintf(line, sizeof(line),
                     "GZ 0.00\nCAM P=%.2f A=0.00 C=0.00 Q=%.2f W=0.00 L=1\n",
                     (double)p, (double)q);
    if (n > 0) uart_write_bytes(LINK_UART, line, n);
}

static esp_err_t camera_init(void)
{
    camera_config_t c = {};
    c.pin_pwdn = 42;
    c.pin_reset = -1;
    c.pin_xclk = 15;
    c.pin_sccb_sda = 4;
    c.pin_sccb_scl = 5;
    c.pin_d7 = 16;
    c.pin_d6 = 17;
    c.pin_d5 = 18;
    c.pin_d4 = 12;
    c.pin_d3 = 10;
    c.pin_d2 = 8;
    c.pin_d1 = 9;
    c.pin_d0 = 11;
    c.pin_vsync = 6;
    c.pin_href = 7;
    c.pin_pclk = 13;
    c.xclk_freq_hz = 20000000;
    c.ledc_timer = LEDC_TIMER_0;
    c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_RGB565;
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 20;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        printf("CAM init failed: 0x%04X\n", (unsigned)err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 2);
        s->set_saturation(s, -2);
        s->set_gain_ctrl(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_whitebal(s, 0);
        printf("CAM OK PID=0x%04X RGB565 QVGA\n", s->id.PID);
    }
    return ESP_OK;
}

// --- Software I2C for MPU6050 ---
// Keep MPU6050 independent from the camera SCCB/I2C driver.
static int g_mpu_sda = 21;
static int g_mpu_scl = 47;

static void sw_i2c_delay(void)
{
    esp_rom_delay_us(20);  // about 25 kHz, tolerant of long wires and weak pullups
}

static void sw_i2c_pin_init(int sda, int scl)
{
    g_mpu_sda = sda;
    g_mpu_scl = scl;

    gpio_config_t cfg = {};
    cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    cfg.pin_bit_mask = (1ULL << g_mpu_sda) | (1ULL << g_mpu_scl);
    gpio_config(&cfg);

    gpio_set_level((gpio_num_t)g_mpu_sda, 1);
    gpio_set_level((gpio_num_t)g_mpu_scl, 1);
    for (int i = 0; i < 50; i++) sw_i2c_delay();
    i2c_ready = true;
}

static void mpu_print_bus_idle(const char *tag)
{
    gpio_set_level((gpio_num_t)g_mpu_sda, 1);
    gpio_set_level((gpio_num_t)g_mpu_scl, 1);
    for (int i = 0; i < 50; i++) sw_i2c_delay();
    printf("%s idle SDA=%d SCL=%d\n", tag,
           gpio_get_level((gpio_num_t)g_mpu_sda),
           gpio_get_level((gpio_num_t)g_mpu_scl));
}

static inline void sw_sda(int level)
{
    gpio_set_level((gpio_num_t)g_mpu_sda, level ? 1 : 0);
}

static inline void sw_scl(int level)
{
    gpio_set_level((gpio_num_t)g_mpu_scl, level ? 1 : 0);
}

static inline int sw_read_sda(void)
{
    return gpio_get_level((gpio_num_t)g_mpu_sda);
}

static void sw_i2c_start(void)
{
    sw_sda(1); sw_scl(1); sw_i2c_delay();
    sw_sda(0); sw_i2c_delay();
    sw_scl(0); sw_i2c_delay();
}

static void sw_i2c_stop(void)
{
    sw_sda(0); sw_i2c_delay();
    sw_scl(1); sw_i2c_delay();
    sw_sda(1); sw_i2c_delay();
}

static bool sw_i2c_write_byte(uint8_t value)
{
    for (int i = 7; i >= 0; i--) {
        sw_sda((value >> i) & 1);
        sw_i2c_delay();
        sw_scl(1); sw_i2c_delay();
        sw_scl(0); sw_i2c_delay();
    }

    sw_sda(1);  // release for ACK
    sw_i2c_delay();
    sw_scl(1); sw_i2c_delay();
    bool ack = (sw_read_sda() == 0);
    sw_scl(0); sw_i2c_delay();
    return ack;
}

static uint8_t sw_i2c_read_byte(bool ack)
{
    uint8_t value = 0;
    sw_sda(1);  // release SDA
    for (int i = 7; i >= 0; i--) {
        sw_scl(1); sw_i2c_delay();
        if (sw_read_sda()) value |= (1U << i);
        sw_scl(0); sw_i2c_delay();
    }

    sw_sda(ack ? 0 : 1);
    sw_i2c_delay();
    sw_scl(1); sw_i2c_delay();
    sw_scl(0); sw_i2c_delay();
    sw_sda(1);
    return value;
}

static esp_err_t write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    sw_i2c_start();
    bool ok = sw_i2c_write_byte((addr << 1) | 0) &&
              sw_i2c_write_byte(reg) &&
              sw_i2c_write_byte(value);
    sw_i2c_stop();
    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    sw_i2c_start();
    bool ok = sw_i2c_write_byte((addr << 1) | 0) && sw_i2c_write_byte(reg);
    if (!ok) {
        sw_i2c_stop();
        return ESP_FAIL;
    }

    sw_i2c_start();
    ok = sw_i2c_write_byte((addr << 1) | 1);
    if (!ok) {
        sw_i2c_stop();
        return ESP_FAIL;
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = sw_i2c_read_byte(i + 1 < len);
    }
    sw_i2c_stop();
    return ESP_OK;
}

static bool mpu_probe_addr(uint8_t addr)
{
    uint8_t who = 0;
    if (read_regs(addr, 0x75, &who, 1) != ESP_OK) {
        return false;
    }
    printf("MPU addr=0x%02X WHO=0x%02X\n", addr, who);
    if (who == 0x68 || who == 0x70 || who == addr) {
        mpu_addr = addr;
        return true;
    }
    if (addr == 0x68) {
        printf("MPU unknown WHO, trying anyway\n");
        mpu_addr = addr;
        return true;
    }
    return false;
}

static int mpu_scan_all(uint8_t *first_addr)
{
    int found = 0;
    if (first_addr) *first_addr = 0;
    printf("I2C scan:");
    for (uint8_t addr = 3; addr < 0x78; addr++) {
        sw_i2c_start();
        bool ack = sw_i2c_write_byte((addr << 1) | 0);
        sw_i2c_stop();
        if (ack) {
            printf(" 0x%02X", addr);
            if (found == 0 && first_addr) *first_addr = addr;
            found++;
        }
        if ((addr & 0x0F) == 0) sleep_ms(1);
    }
    printf(" found=%d\n", found);
    return found;
}

static bool mpu_init(void)
{
    const int pins[][2] = {
        {47, 21},
        {21, 47},
    };

    for (int p = 0; p < 2; p++) {
        sw_i2c_pin_init(pins[p][0], pins[p][1]);
        printf("MPU soft I2C try SDA=%d SCL=%d\n", pins[p][0], pins[p][1]);
        mpu_print_bus_idle("MPU bus");

        if (mpu_probe_addr(MPU_ADDRS[0]) || mpu_probe_addr(MPU_ADDRS[1])) {
            write_reg(mpu_addr, 0x6B, 0x00);
            sleep_ms(100);
            write_reg(mpu_addr, 0x19, 0x04);
            write_reg(mpu_addr, 0x1A, 0x03);
            write_reg(mpu_addr, 0x1B, 0x00);
            write_reg(mpu_addr, 0x1C, 0x00);
            sleep_ms(100);
            printf("MPU6050 init OK addr=0x%02X SDA=%d SCL=%d soft-i2c\n",
                   mpu_addr, pins[p][0], pins[p][1]);
            return true;
        }

        uint8_t first = 0;
        int found = mpu_scan_all(&first);
        if (found > 0 && first != MPU_ADDRS[0] && first != MPU_ADDRS[1]) {
            printf("I2C device found at 0x%02X, probing it as MPU candidate\n", first);
            if (mpu_probe_addr(first)) {
                write_reg(mpu_addr, 0x6B, 0x00);
                sleep_ms(100);
                write_reg(mpu_addr, 0x19, 0x04);
                write_reg(mpu_addr, 0x1A, 0x03);
                write_reg(mpu_addr, 0x1B, 0x00);
                write_reg(mpu_addr, 0x1C, 0x00);
                sleep_ms(100);
                printf("MPU6050 init OK addr=0x%02X SDA=%d SCL=%d soft-i2c scan\n",
                       mpu_addr, pins[p][0], pins[p][1]);
                return true;
            }
        }
        printf("No MPU on soft I2C SDA=%d SCL=%d\n", pins[p][0], pins[p][1]);
    }
    return false;
}
static int16_t read_i16_be(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

static bool read_motion(MotionData *m)
{
    if (!mpu_ready) return false;
    uint8_t data[14] = {};
    if (read_regs(mpu_addr, 0x3B, data, sizeof(data)) != ESP_OK) return false;
    int16_t gz_raw = read_i16_be(&data[12]);
    m->gz = gz_raw / 131.0f;
    return true;
}

static void send_gyro(float gz)
{
    if (!link_ready) return;
    char line[32];
    int n = snprintf(line, sizeof(line), "GZ %.2f\n", (double)gz);
    if (n > 0) uart_write_bytes(LINK_UART, line, n);
}

static inline uint8_t pixel_luma(const camera_fb_t *fb, int x, int y)
{
    if (fb->format == PIXFORMAT_GRAYSCALE) {
        return fb->buf[y * fb->width + x];
    }

    const uint8_t *p = fb->buf + ((y * fb->width + x) * 2);
    uint16_t rgb = ((uint16_t)p[0] << 8) | p[1];
    uint8_t r = (uint8_t)(((rgb >> 11) & 0x1F) << 3);
    uint8_t g = (uint8_t)(((rgb >> 5) & 0x3F) << 2);
    uint8_t b = (uint8_t)((rgb & 0x1F) << 3);
    return (uint8_t)(((uint16_t)r * 30 + (uint16_t)g * 59 + (uint16_t)b * 11) / 100);
}

// 最长黑列法：逐列统计黑像素，找最长黑列位置
static int find_longest_black_column(const camera_fb_t *fb, int threshold,
                                      int y_start, int y_end, int x_start, int x_end)
{
    int best_x = x_start;
    int best_cnt = 0;

    for (int x = x_start; x < x_end; x++) {
        int dark = 0;
        for (int y = y_start; y < y_end; y += 2) {
            if (pixel_luma(fb, x, y) < threshold) dark++;
        }
        if (dark > best_cnt) {
            best_cnt = dark;
            best_x = x;
        }
    }
    return best_x;
}

static CamData analyze_frame(const camera_fb_t *fb)
{
    CamData out = {};
    out.lost = true;
    if (!fb ||
        (fb->format != PIXFORMAT_GRAYSCALE && fb->format != PIXFORMAT_RGB565) ||
        fb->width < 40 || fb->height < 40) {
        return out;
    }

    const int width = (int)fb->width;
    const int height = (int)fb->height;
    // Analyze only the forward track area.
    // The lower part contains the car nose / LEDs and must not affect line detection.
    const int row_start = height / 10;
    const int row_end = height * 62 / 100;
    const int col_start = width / 12;
    const int col_end = width - width / 12;
    const int row_step = 3;
    const int col_step = 1;

    int vmin = 255;
    int vmax = 0;
    int sampled = 0;

    for (int y = row_start; y < row_end; y += row_step) {
        for (int x = col_start; x < col_end; x += 2) {
            int v = pixel_luma(fb, x, y);
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
            sampled++;
        }
    }

    int contrast = vmax - vmin;
    out.contrast = contrast;

    if (sampled <= 0 || contrast < 12) {
        return out;
    }

    int threshold = (vmax + vmin) / 2;
    const int min_run_len = 3;
    const int max_run_frac_num = 7;
    const int max_run_frac_den = 10;

    float group_sum[3] = {};
    float group_weight[3] = {};
    int valid_rows = 0;
    int center_rows = 0;
    int total_black_px = 0;
    int total_px = 0;
    const int rows_span = row_end - row_start;

    for (int y = row_start; y < row_end; y += row_step) {
        int best_start = -1;
        int best_end = -1;
        int best_len = 0;
        int cur_start = -1;
        int cur_len = 0;
        int row_px = 0;
        int row_black = 0;

        for (int x = col_start; x < col_end; x += col_step) {
            row_px++;
            bool is_black = pixel_luma(fb, x, y) < threshold;

            if (is_black) {
                row_black++;
                if (cur_start < 0) {
                    cur_start = x;
                    cur_len = 1;
                } else {
                    cur_len++;
                }
            } else if (cur_start >= 0) {
                if (cur_len > best_len) {
                    best_len = cur_len;
                    best_start = cur_start;
                    best_end = x - col_step;
                }
                cur_start = -1;
                cur_len = 0;
            }
        }

        if (cur_start >= 0 && cur_len > best_len) {
            best_len = cur_len;
            best_start = cur_start;
            best_end = col_end - col_step;
        }

        total_black_px += row_black;
        total_px += row_px;

        int row_width = col_end - col_start;
        int max_run_len = row_width * max_run_frac_num / max_run_frac_den;
        if (best_len < min_run_len || best_len > max_run_len) {
            continue;
        }

        float center_x = ((float)best_start + (float)best_end) * 0.5f;
        float norm = (center_x - (float)(width - 1) * 0.5f) /
                     ((float)(width - 1) * 0.5f);
        norm = clampf_local(norm, -1.0f, 1.0f);

        if (fabsf(norm) <= 0.26f) {
            center_rows++;
        }

        int rel = y - row_start;
        int group = 0;
        if (rel > rows_span * 2 / 3) {
            group = 2;
        } else if (rel > rows_span / 3) {
            group = 1;
        }

        float y_weight = 1.0f + (float)(y - row_start) / (float)rows_span;
        float len_weight = clampf_local((float)best_len / 16.0f, 0.5f, 2.0f);
        float weight = y_weight * len_weight;

        group_sum[group] += norm * weight;
        group_weight[group] += weight;
        valid_rows++;
    }

    out.valid_rows = valid_rows;
    out.w = total_px > 0 ? (float)total_black_px / (float)total_px : 0.0f;

    if (valid_rows < 4) {
        return out;
    }

    float all_sum = group_sum[0] + group_sum[1] + group_sum[2];
    float all_weight = group_weight[0] + group_weight[1] + group_weight[2];
    if (all_weight <= 0.0f) {
        return out;
    }

    float all_p = all_sum / all_weight;
    float far_p = group_weight[0] > 0.0f ? group_sum[0] / group_weight[0] : all_p;
    float mid_p = group_weight[1] > 0.0f ? group_sum[1] / group_weight[1] : all_p;
    float near_p = group_weight[2] > 0.0f ? group_sum[2] / group_weight[2] : all_p;

    out.p = clampf_local(group_weight[2] > 0.0f ? (near_p * 0.75f + mid_p * 0.25f) : all_p,
                         -1.0f, 1.0f);
    out.a = clampf_local(far_p - near_p, -1.0f, 1.0f);
    out.c = clampf_local(0.70f * (far_p - near_p) + 0.30f * (mid_p - near_p),
                         -1.0f, 1.0f);

    // quality
    float expected_rows = (float)((row_end - row_start + row_step - 1) / row_step);
    float row_q = clampf_local((float)valid_rows / (expected_rows * 0.65f), 0.0f, 1.0f);
    float contrast_q = clampf_local((float)(contrast - 12) / 70.0f, 0.0f, 1.0f);
    float width_q = (out.w >= 0.010f && out.w <= 0.50f) ? 1.0f : 0.35f;
    out.q = clampf_local(row_q * 0.55f + contrast_q * 0.35f + width_q * 0.10f,
                         0.0f, 1.0f);

    out.lost = out.q < 0.22f;
    if (out.lost) { out.p = 0.0f; out.a = 0.0f; out.c = 0.0f; }

    // Whole-screen straight trigger for the main controller.
    // For this car, the useful signal is a continuous centered black stripe
    // from far to near, like the WiFi preview image.
    float center_row_ratio = valid_rows > 0 ? (float)center_rows / (float)valid_rows : 0.0f;
    bool whole_screen_straight =
        !out.lost &&
        out.q >= 0.42f &&
        row_q >= 0.58f &&
        center_row_ratio >= 0.70f &&
        out.valid_rows >= 9 &&
        out.w >= 0.020f &&
        out.w <= 0.52f &&
        fabsf(out.p) <= 0.65f &&
        fabsf(out.a) <= 0.35f &&
        fabsf(out.c) <= 0.45f;

    if (whole_screen_straight) {
        if (straight_frames < 100) straight_frames++;
    } else {
        straight_frames = 0;
    }

    out.boost = straight_frames >= 10;

    int raw_turn_dir = 0;
    bool sharp_turn =
        !out.lost &&
        out.q >= 0.38f &&
        row_q >= 0.43f &&
        out.valid_rows >= 6 &&
        out.w >= 0.015f &&
        out.w <= 0.58f &&
        (fabsf(out.c) >= 0.48f || fabsf(out.a) >= 0.50f || fabsf(far_p) >= 0.60f);

    if (sharp_turn) {
        float turn_signal = out.c * 0.55f + out.a * 0.30f + far_p * 0.15f;
        raw_turn_dir = (turn_signal >= 0.0f) ? -1 : 1;
    }

    if (raw_turn_dir != 0) {
        if (turn_dir_hold == raw_turn_dir) {
            if (turn_frames < 100) turn_frames++;
        } else {
            turn_dir_hold = raw_turn_dir;
            turn_frames = 1;
        }
    } else {
        turn_frames = 0;
        turn_dir_hold = 0;
    }

    out.turn_dir = (turn_frames >= 4) ? turn_dir_hold : 0;

    return out;
}

static void send_cam(const CamData &cam)
{
    if (!link_ready) return;
    char line[128];
    int n = snprintf(line, sizeof(line),
                     "CAM P=%.2f A=%.2f C=%.2f Q=%.2f W=%.2f L=%d\nBOOST %d\n",
                     (double)cam.p, (double)cam.a, (double)cam.c,
                     (double)cam.q, (double)cam.w, cam.lost ? 1 : 0,
                     cam.boost ? 1 : 0);
    if (n > 0) uart_write_bytes(LINK_UART, line, n);
}

extern "C" void app_main(void)
{
    printf("\n===== S3 camera line assist =====\n");
    printf("UART link TX=GPIO%d RX=GPIO%d baud=%d\n", LINK_TX_PIN, LINK_RX_PIN, LINK_BAUD);
    led_init();
    link_init();
    send_boot_status(-0.90f, 0.10f);
    sleep_ms(200);

    send_boot_status(-0.70f, 0.12f);
    printf("Init MPU before camera...\n");
    mpu_ready = mpu_init();
    send_boot_status(-0.45f, mpu_ready ? 0.30f : 0.08f);

    printf("Init camera after MPU...\n");
    bool cam_ok = (camera_init() == ESP_OK);
    send_boot_status(cam_ok ? 0.00f : -0.50f, cam_ok ? 0.25f : 0.05f);
    printf("INIT cam=%d mpu=%d\n", cam_ok ? 1 : 0, mpu_ready ? 1 : 0);

    TickType_t last_print = 0;
    TickType_t led_until = 0;

    while (1) {
        MotionData motion = {};
        if (read_motion(&motion)) {
            send_gyro(motion.gz);
        }

        CamData cam = {};
        cam.lost = true;
        if (cam_ok) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                cam = analyze_frame(fb);
                esp_camera_fb_return(fb);
            }
        }

        send_cam(cam);

        TickType_t now = xTaskGetTickCount();
        if (cam.boost) {
            led_until = now + pdMS_TO_TICKS(80);
        }
        gpio_set_level((gpio_num_t)LED_PIN, now < led_until ? 1 : 0);

        if ((now - last_print) >= pdMS_TO_TICKS(1000)) {
            printf("CAM P=%.2f A=%.2f C=%.2f Q=%.2f W=%.2f L=%d B=%d contrast=%d rows=%d MPU=%d GZ=%.2f\n",
                   (double)cam.p, (double)cam.a, (double)cam.c, (double)cam.q,
                   (double)cam.w, cam.lost ? 1 : 0, cam.boost ? 1 : 0,
                   cam.contrast, cam.valid_rows,
                   mpu_ready ? 1 : 0, (double)motion.gz);
            last_print = now;
        }

        sleep_ms(20);
    }
}

#endif // APP_HD_STREAM
