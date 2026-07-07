#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const i2c_port_t I2C_PORT = I2C_NUM_0;
static const int I2C_FREQ_HZ = 400000;
static const uint8_t MPU_ADDRS[] = {0x68, 0x69};

static const uart_port_t LINK_UART = UART_NUM_1;
static const int LINK_TX_PIN = 45;
static const int LINK_RX_PIN = 46;
static const int LINK_BAUD = 115200;
static const int LINK_PERIOD_MS = 20;
static const int GYRO_CALIB_SAMPLES = 80;
static const float STRAIGHT_GZ_DPS_TH = 5.0f;
static const float STRAIGHT_GZ_DPS_EXIT_TH = 8.0f;
static const int STRAIGHT_CONFIRM_CYCLES = 12;
static const int STRAIGHT_LOST_CYCLES = 5;

static const int STATUS_LED_PIN = 2;
static const int STATUS_LED_ACTIVE_LEVEL = 1;
static const int BOOST_HINT_TIMEOUT_MS = 300;
static const int BOOST_BLINK_PERIOD_MS = 160;

static int active_sda = -1;
static int active_scl = -1;
static uint8_t active_addr = 0;
static bool i2c_ready = false;
static bool boost_hint = false;
static TickType_t boost_hint_last_tick = 0;
static char link_rx_line[32];
static int link_rx_len = 0;
static float gz_bias = 0.0f;
static bool straight_state = false;
static int straight_count = 0;
static int curve_count = 0;

struct MotionData {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float temp;
};

static void sleepMs(int ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

static void statusLedSet(bool on) {
  gpio_set_level((gpio_num_t)STATUS_LED_PIN,
                 on ? STATUS_LED_ACTIVE_LEVEL : !STATUS_LED_ACTIVE_LEVEL);
}

static void statusLedInit() {
  gpio_config_t config = {};
  config.intr_type = GPIO_INTR_DISABLE;
  config.mode = GPIO_MODE_OUTPUT;
  config.pin_bit_mask = (1ULL << STATUS_LED_PIN);
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&config);
  statusLedSet(false);
}

static void parseMainCommand(const char *line) {
  if (line[0] == 'B' && line[1] == 'O' && line[2] == 'O' &&
      line[3] == 'S' && line[4] == 'T') {
    const char *p = line + 5;
    while (*p == ' ' || *p == '=' || *p == ':') p++;
    boost_hint = (*p == '1');
    boost_hint_last_tick = xTaskGetTickCount();
  }
}

static void receiveMainCommands() {
  uint8_t buf[64];
  int n = uart_read_bytes(LINK_UART, buf, sizeof(buf), 0);
  for (int i = 0; i < n; i++) {
    char ch = (char)buf[i];
    if (ch == '\n' || ch == '\r') {
      if (link_rx_len > 0) {
        link_rx_line[link_rx_len] = '\0';
        parseMainCommand(link_rx_line);
        link_rx_len = 0;
      }
    } else if (link_rx_len < 31) {
      link_rx_line[link_rx_len++] = ch;
    } else {
      link_rx_len = 0;
    }
  }
}

static void statusLedUpdate() {
  TickType_t now = xTaskGetTickCount();
  if (boost_hint &&
      (now - boost_hint_last_tick) > pdMS_TO_TICKS(BOOST_HINT_TIMEOUT_MS)) {
    boost_hint = false;
  }

  if (!boost_hint) {
    statusLedSet(false);
    return;
  }

  TickType_t phase = now % pdMS_TO_TICKS(BOOST_BLINK_PERIOD_MS);
  statusLedSet(phase < pdMS_TO_TICKS(BOOST_BLINK_PERIOD_MS / 2));
}

static int16_t readI16BE(const uint8_t *p) {
  return (int16_t)((p[0] << 8) | p[1]);
}

static void linkInit() {
  uart_config_t config = {};
  config.baud_rate = LINK_BAUD;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_DEFAULT;

  ESP_ERROR_CHECK(uart_param_config(LINK_UART, &config));
  ESP_ERROR_CHECK(uart_set_pin(LINK_UART, LINK_TX_PIN, LINK_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(LINK_UART, 512, 0, 0, NULL, 0));
}

static void i2cStopDriver() {
  if (i2c_ready) {
    i2c_driver_delete(I2C_PORT);
    i2c_ready = false;
    sleepMs(50);
  }
}

static bool i2cStartDriver(int sda, int scl) {
  i2cStopDriver();

  i2c_config_t config = {};
  config.mode = I2C_MODE_MASTER;
  config.sda_io_num = sda;
  config.scl_io_num = scl;
  config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  config.master.clk_speed = I2C_FREQ_HZ;

  printf("--------------------------------\n");
  printf("I2C try: SDA=GPIO%d SCL=GPIO%d freq=%d\n", sda, scl, I2C_FREQ_HZ);

  esp_err_t err = i2c_param_config(I2C_PORT, &config);
  if (err != ESP_OK) {
    printf("i2c_param_config failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = i2c_driver_install(I2C_PORT, config.mode, 0, 0, 0);
  if (err != ESP_OK) {
    printf("i2c_driver_install failed: %s\n", esp_err_to_name(err));
    return false;
  }

  i2c_ready = true;
  active_sda = sda;
  active_scl = scl;
  return true;
}

static bool probeAddr(uint8_t addr) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(cmd);
  return err == ESP_OK;
}

static esp_err_t writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  return i2c_master_write_to_device(I2C_PORT, addr, data, sizeof(data), pdMS_TO_TICKS(50));
}

static esp_err_t readRegs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
  return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, data, len, pdMS_TO_TICKS(50));
}

static bool findMpu() {
  const int pin_sets[][2] = {
    {21, 47},
    {47, 21},
  };

  for (int p = 0; p < 2; p++) {
    int sda = pin_sets[p][0];
    int scl = pin_sets[p][1];
    if (!i2cStartDriver(sda, scl)) continue;

    printf("I2C scan:");
    bool any = false;
    for (int addr = 1; addr < 127; addr++) {
      if (probeAddr((uint8_t)addr)) {
        printf(" 0x%02X", addr);
        any = true;
      }
    }
    if (!any) printf(" none");
    printf("\n");

    for (uint8_t addr : MPU_ADDRS) {
      if (!probeAddr(addr)) {
        printf("ADDR 0x%02X NACK\n", addr);
        continue;
      }

      uint8_t who = 0;
      esp_err_t err = readRegs(addr, 0x75, &who, 1);
      if (err != ESP_OK) {
        printf("ADDR 0x%02X ACK but WHO_AM_I read failed: %s\n", addr, esp_err_to_name(err));
        continue;
      }

      printf("ADDR 0x%02X ACK WHO_AM_I=0x%02X\n", addr, who);
      if (who == 0x68 || who == 0x69) {
        active_addr = addr;
        return true;
      }
    }
  }

  return false;
}

static bool initMpu() {
  if (writeReg(active_addr, 0x6B, 0x00) != ESP_OK) return false;
  sleepMs(100);
  if (writeReg(active_addr, 0x1B, 0x00) != ESP_OK) return false;
  if (writeReg(active_addr, 0x1C, 0x00) != ESP_OK) return false;
  if (writeReg(active_addr, 0x1A, 0x03) != ESP_OK) return false;
  sleepMs(100);
  return true;
}

static bool readMotion(MotionData *motion) {
  uint8_t data[14] = {};
  esp_err_t err = readRegs(active_addr, 0x3B, data, sizeof(data));
  if (err != ESP_OK) {
    printf("read 0x3B failed: %s\n", esp_err_to_name(err));
    return false;
  }

  int16_t ax_raw = readI16BE(&data[0]);
  int16_t ay_raw = readI16BE(&data[2]);
  int16_t az_raw = readI16BE(&data[4]);
  int16_t temp_raw = readI16BE(&data[6]);
  int16_t gx_raw = readI16BE(&data[8]);
  int16_t gy_raw = readI16BE(&data[10]);
  int16_t gz_raw = readI16BE(&data[12]);

  motion->ax = ax_raw / 16384.0f;
  motion->ay = ay_raw / 16384.0f;
  motion->az = az_raw / 16384.0f;
  motion->gx = gx_raw / 131.0f;
  motion->gy = gy_raw / 131.0f;
  motion->gz = gz_raw / 131.0f;
  motion->temp = temp_raw / 340.0f + 36.53f;

  return true;
}

static void sendGyroToMain(const MotionData &motion) {
  float gz = motion.gz - gz_bias;

  if (fabsf(gz) < STRAIGHT_GZ_DPS_TH) {
    straight_count++;
    curve_count = 0;
  } else if (fabsf(gz) > STRAIGHT_GZ_DPS_EXIT_TH) {
    curve_count++;
    straight_count = 0;
  }

  if (straight_count >= STRAIGHT_CONFIRM_CYCLES) {
    straight_state = true;
  }
  if (curve_count >= STRAIGHT_LOST_CYCLES) {
    straight_state = false;
  }

  char line[8];
  int n = snprintf(line, sizeof(line), "%d\n", straight_state ? 1 : 0);
  if (n > 0) {
    uart_write_bytes(LINK_UART, line, n);
  }
}

static void printMotion(const MotionData &motion) {
  printf("AX=%.3fg  AY=%.3fg  AZ=%.3fg  GX=%.2f  GY=%.2f  GZ=%.2f  GZ0=%.2f  STR=%d  TEMP=%.2fC\n",
         motion.ax, motion.ay, motion.az, motion.gx, motion.gy,
         motion.gz, motion.gz - gz_bias, straight_state ? 1 : 0, motion.temp);
}

static void calibrateGyroBias() {
  printf("Calibrating gyro bias, keep S3 still...\n");
  float sum = 0.0f;
  int ok = 0;
  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    MotionData motion = {};
    if (readMotion(&motion)) {
      sum += motion.gz;
      ok++;
    }
    sleepMs(LINK_PERIOD_MS);
  }
  if (ok > 0) {
    gz_bias = sum / (float)ok;
  }
  printf("Gyro bias OK: gz_bias=%.2f from %d samples\n", gz_bias, ok);
}

extern "C" void app_main(void) {
  printf("\n===== ESP32-S3 MPU6050 C detect =====\n");
  printf("Priority: SDA=GPIO21 SCL=GPIO47, then swapped. Freq=%d\n", I2C_FREQ_HZ);
  printf("UART link: TX=GPIO%d RX=GPIO%d baud=%d, sending \"GZ value\" every %d ms\n",
         LINK_TX_PIN, LINK_RX_PIN, LINK_BAUD, LINK_PERIOD_MS);
  printf("Status LED: GPIO%d blinks when main sends BOOST 1\n", STATUS_LED_PIN);
  statusLedInit();
  linkInit();

  if (!findMpu()) {
    printf("MPU6050 NOT FOUND.\n");
    while (1) sleepMs(1000);
  }

  printf("FOUND MPU: SDA=GPIO%d SCL=GPIO%d addr=0x%02X\n", active_sda, active_scl, active_addr);
  if (!initMpu()) {
    printf("MPU init failed.\n");
    while (1) sleepMs(1000);
  }

  calibrateGyroBias();
  printf("MPU init OK. Sending 1/0 straight flag to main every %d ms, USB prints every 200 ms.\n", LINK_PERIOD_MS);
  int print_divider = 0;
  while (1) {
    receiveMainCommands();
    statusLedUpdate();

    MotionData motion = {};
    if (readMotion(&motion)) {
      sendGyroToMain(motion);
      if (++print_divider >= 10) {
        printMotion(motion);
        print_divider = 0;
      }
    }
    receiveMainCommands();
    statusLedUpdate();
    sleepMs(LINK_PERIOD_MS);
  }
}
