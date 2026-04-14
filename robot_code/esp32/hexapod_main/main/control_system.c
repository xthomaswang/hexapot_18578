/*
 * Hexapod main firmware: Pi UART bridge -> 18 servos (2x PCA9685) + BNO055 IMU.
 *
 * Pi -> ESP wire contract (all lines terminated by '\n' or '\r\n'):
 *   M:f0,f1,...,f17          full-body move (18 servo angles in degrees)
 *   N:f0,..f17               same payload, neutral direction tag
 *   F:/B:/L:/R:/LL:/RR:      same 18-float payload with direction tag (logged)
 *   H:                        home -> all legs to (90,90,90)
 *   S:                        stop -> all legs to (90,90,90), PWM stays active
 *   V:<percent>               speed setting (0..100), recorded only
 *   Q:                        query status -> ESP replies with ST: line
 *
 * ESP -> Pi uplink:
 *   ST:heading,roll,pitch,uptime_ms,last_cmd_age_ms  (reply to Q:)
 *
 * Boot policy:
 *   At power-on PWM outputs are left at 0 (no pulse). Servos stay inert
 *   until the first valid frame (or H:/S:) arrives from the Pi. This
 *   prevents any involuntary snap during boot.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "driver/i2c.h"

static const char *TAG = "HEXAPOD";

/* ==============================================================
 * Configuration
 * ============================================================== */

/* ---- UART (Pi <-> ESP32) ---- */
#define PI_UART_NUM         UART_NUM_2
#define PI_UART_TX_PIN      17
#define PI_UART_RX_PIN      16
#define PI_UART_BAUD        115200
#define UART_RX_BUF_SIZE    1024
#define UART_TX_BUF_SIZE    1024
#define RX_LINE_MAX         512

/* ---- I2C ---- */
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_TIMEOUT_MS          100

/* ---- PCA9685 ---- */
#define PCA9685_ADDR_0          0x40    /* legs 1,2,3 */
#define PCA9685_ADDR_1          0x41    /* legs 4,5,6 (A0 soldered) */

#define PCA9685_MODE1           0x00
#define PCA9685_PRESCALE_REG    0xFE
#define PCA9685_LED0_ON_L       0x06
#define PCA9685_MODE1_RESTART   0x80
#define PCA9685_MODE1_AI        0x20
#define PCA9685_MODE1_SLEEP     0x10

/* Prescale value for 50 Hz with the internal 25 MHz oscillator:
 *   round(25e6 / (4096 * 50)) - 1 = 121
 */
#define PCA9685_PRESCALE_50HZ   121

/* ---- BNO055 ---- */
#define BNO055_ADDR             0x28    /* default; 0x29 if ADR pin high */
#define BNO055_CHIP_ID_REG      0x00
#define BNO055_CHIP_ID_VAL      0xA0
#define BNO055_PAGE_ID_REG      0x07
#define BNO055_OPR_MODE_REG     0x3D
#define BNO055_PWR_MODE_REG     0x3E
#define BNO055_SYS_TRIGGER_REG  0x3F
#define BNO055_UNIT_SEL_REG     0x3B
#define BNO055_EULER_H_LSB      0x1A

#define BNO055_MODE_CONFIG      0x00
#define BNO055_MODE_NDOF        0x0C
#define BNO055_PWR_NORMAL       0x00

/* ---- Servo hard limits ---- *
 * Conservative mechanical envelope for the S6510 at 50 Hz.
 * Pi side emits degrees; ESP clamps to [30, 150] before mapping to PWM.
 */
#define SERVO_MIN_DEG           30
#define SERVO_MAX_DEG           150
#define SERVO_MIN_US            1000
#define SERVO_MAX_US            2000
#define SERVO_PWM_FREQ_HZ       50

/* ---- Robot layout ---- */
#define NUM_LEGS                6
#define JOINTS_PER_LEG          3
#define NUM_SERVOS              (NUM_LEGS * JOINTS_PER_LEG)

/* ---- Safe poses (joint midpoint, used by H: and S:) ---- */
#define SAFE_POSE_COXA          90
#define SAFE_POSE_FEMUR         90
#define SAFE_POSE_TIBIA         90

/* ==============================================================
 * Per-leg channel map
 *
 * Each PCA9685 drives 3 legs using channels {0,1,2}, {4,5,6}, {8,9,10}.
 * Legs 1..3 live on 0x40, legs 4..6 on 0x41.
 * ============================================================== */
typedef struct {
    uint8_t pca_addr;
    uint8_t coxa_ch;
    uint8_t femur_ch;
    uint8_t tibia_ch;
} leg_map_t;

static const leg_map_t g_leg_map[NUM_LEGS] = {
    {PCA9685_ADDR_0,  0,  1,  2},   /* leg 1 (L1) */
    {PCA9685_ADDR_0,  4,  5,  6},   /* leg 2 (L2) */
    {PCA9685_ADDR_0,  8,  9, 10},   /* leg 3 (L3) */
    {PCA9685_ADDR_1,  0,  1,  2},   /* leg 4 (R1) */
    {PCA9685_ADDR_1,  4,  5,  6},   /* leg 5 (R2) */
    {PCA9685_ADDR_1,  8,  9, 10},   /* leg 6 (R3) */
};

/* ==============================================================
 * Runtime state
 * ============================================================== */
static volatile bool     g_pwm_unlocked   = false; /* false until first Pi frame */
static volatile int64_t  g_last_cmd_us    = 0;     /* esp_timer at last accepted frame */
static volatile int      g_speed_percent  = 100;   /* from V: — advisory only */

typedef struct {
    float heading_deg;
    float roll_deg;
    float pitch_deg;
} bno055_euler_t;

static volatile bool     g_bno_ready      = false;

/* ==============================================================
 * Helpers
 * ============================================================== */
static inline int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float clamp_float(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ==============================================================
 * I2C helpers
 * ============================================================== */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM,
                                      dev_addr,
                                      buf,
                                      sizeof(buf),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        dev_addr,
                                        &reg,
                                        1,
                                        value,
                                        1,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        dev_addr,
                                        &reg,
                                        1,
                                        out,
                                        len,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/* ==============================================================
 * PCA9685
 * ============================================================== */
static esp_err_t pca9685_init_one(uint8_t addr)
{
    uint8_t oldmode = 0;

    /* Wake + known state */
    ESP_ERROR_CHECK(i2c_write_reg(addr, PCA9685_MODE1, 0x00));
    ESP_ERROR_CHECK(i2c_read_reg(addr, PCA9685_MODE1, &oldmode));

    /* Enter sleep to allow PRESCALE write */
    uint8_t sleep_mode = (oldmode & 0x7F) | PCA9685_MODE1_SLEEP;
    ESP_ERROR_CHECK(i2c_write_reg(addr, PCA9685_MODE1, sleep_mode));
    ESP_ERROR_CHECK(i2c_write_reg(addr, PCA9685_PRESCALE_REG, PCA9685_PRESCALE_50HZ));
    ESP_ERROR_CHECK(i2c_write_reg(addr, PCA9685_MODE1, oldmode));

    vTaskDelay(pdMS_TO_TICKS(5));

    /* Restart + auto-increment for multi-byte register writes */
    ESP_ERROR_CHECK(i2c_write_reg(addr, PCA9685_MODE1,
                                  PCA9685_MODE1_RESTART | PCA9685_MODE1_AI));

    ESP_LOGI(TAG, "PCA9685 @ 0x%02X initialized (50 Hz)", addr);
    return ESP_OK;
}

static esp_err_t pca9685_set_pwm(uint8_t addr, uint8_t channel,
                                 uint16_t on_count, uint16_t off_count)
{
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t buf[5];

    buf[0] = reg;
    buf[1] = on_count & 0xFF;
    buf[2] = (on_count >> 8) & 0x0F;
    buf[3] = off_count & 0xFF;
    buf[4] = (off_count >> 8) & 0x0F;

    return i2c_master_write_to_device(I2C_MASTER_NUM,
                                      addr,
                                      buf,
                                      sizeof(buf),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/* ==============================================================
 * Servo math (degree -> pulse -> 12-bit count)
 * ============================================================== */
static uint16_t angle_to_pulse_us(int angle_deg)
{
    int a = clamp_int(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    float t = (float)(a - SERVO_MIN_DEG) /
              (float)(SERVO_MAX_DEG - SERVO_MIN_DEG);
    return (uint16_t)(SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US));
}

static uint16_t pulse_us_to_counts(uint16_t pulse_us)
{
    /* 50 Hz => 20000 us period, 12-bit resolution */
    float counts = ((float)pulse_us * 4096.0f) / 20000.0f;
    if (counts < 0.0f)    counts = 0.0f;
    if (counts > 4095.0f) counts = 4095.0f;
    return (uint16_t)counts;
}

static esp_err_t set_servo_angle_raw(uint8_t pca_addr, uint8_t channel, int angle_deg)
{
    uint16_t pulse_us = angle_to_pulse_us(angle_deg);
    uint16_t counts   = pulse_us_to_counts(pulse_us);
    return pca9685_set_pwm(pca_addr, channel, 0, counts);
}

static esp_err_t set_leg_angles(uint8_t leg_idx, int coxa_deg, int femur_deg, int tibia_deg)
{
    if (leg_idx >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }

    const leg_map_t *leg = &g_leg_map[leg_idx];
    esp_err_t err;

    err = set_servo_angle_raw(leg->pca_addr, leg->coxa_ch,  coxa_deg);
    if (err != ESP_OK) return err;
    err = set_servo_angle_raw(leg->pca_addr, leg->femur_ch, femur_deg);
    if (err != ESP_OK) return err;
    err = set_servo_angle_raw(leg->pca_addr, leg->tibia_ch, tibia_deg);
    return err;
}

static esp_err_t apply_safe_pose(void)
{
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        esp_err_t err = set_leg_angles(leg, SAFE_POSE_COXA, SAFE_POSE_FEMUR, SAFE_POSE_TIBIA);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "apply_safe_pose failed on leg %d: %s", leg + 1, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}

/* ==============================================================
 * BNO055 IMU
 * ============================================================== */
static esp_err_t bno055_init(void)
{
    uint8_t chip_id = 0;

    if (i2c_write_reg(BNO055_ADDR, BNO055_PAGE_ID_REG, 0x00) != ESP_OK) {
        ESP_LOGW(TAG, "BNO055 not responding at 0x%02X", BNO055_ADDR);
        return ESP_FAIL;
    }
    if (i2c_read_reg(BNO055_ADDR, BNO055_CHIP_ID_REG, &chip_id) != ESP_OK ||
        chip_id != BNO055_CHIP_ID_VAL) {
        ESP_LOGW(TAG, "BNO055 chip ID mismatch: got 0x%02X", chip_id);
        return ESP_FAIL;
    }

    /* Config mode -> reset -> normal power -> NDOF */
    i2c_write_reg(BNO055_ADDR, BNO055_OPR_MODE_REG, BNO055_MODE_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(25));

    i2c_write_reg(BNO055_ADDR, BNO055_SYS_TRIGGER_REG, 0x20);   /* RST_SYS */
    vTaskDelay(pdMS_TO_TICKS(700));

    if (i2c_read_reg(BNO055_ADDR, BNO055_CHIP_ID_REG, &chip_id) != ESP_OK ||
        chip_id != BNO055_CHIP_ID_VAL) {
        ESP_LOGW(TAG, "BNO055 chip ID mismatch after reset: 0x%02X", chip_id);
        return ESP_FAIL;
    }

    i2c_write_reg(BNO055_ADDR, BNO055_PWR_MODE_REG,    BNO055_PWR_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_reg(BNO055_ADDR, BNO055_PAGE_ID_REG,     0x00);
    i2c_write_reg(BNO055_ADDR, BNO055_UNIT_SEL_REG,    0x00);   /* deg, m/s^2, dps */
    i2c_write_reg(BNO055_ADDR, BNO055_SYS_TRIGGER_REG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_write_reg(BNO055_ADDR, BNO055_OPR_MODE_REG, BNO055_MODE_NDOF);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "BNO055 initialized in NDOF mode");
    g_bno_ready = true;
    return ESP_OK;
}

static esp_err_t bno055_read_euler_deg(bno055_euler_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    if (!g_bno_ready) {
        out->heading_deg = 0.0f;
        out->roll_deg = 0.0f;
        out->pitch_deg = 0.0f;
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[6] = {0};
    esp_err_t err = i2c_read_bytes(BNO055_ADDR, BNO055_EULER_H_LSB, data, sizeof(data));
    if (err != ESP_OK) return err;

    int16_t heading_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t roll_raw    = (int16_t)((data[3] << 8) | data[2]);
    int16_t pitch_raw   = (int16_t)((data[5] << 8) | data[4]);

    /* BNO055 Euler values: 1 LSB = 1/16 deg */
    out->heading_deg = heading_raw / 16.0f;
    out->roll_deg    = roll_raw    / 16.0f;
    out->pitch_deg   = pitch_raw   / 16.0f;
    return ESP_OK;
}

/* ==============================================================
 * UART
 * ============================================================== */
static void pi_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate  = PI_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(PI_UART_NUM,
                                        UART_RX_BUF_SIZE,
                                        UART_TX_BUF_SIZE,
                                        0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PI_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PI_UART_NUM,
                                 PI_UART_TX_PIN, PI_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialized: baud=%d tx=%d rx=%d",
             PI_UART_BAUD, PI_UART_TX_PIN, PI_UART_RX_PIN);
}

static void pi_uart_write_line(const char *line, size_t len)
{
    uart_write_bytes(PI_UART_NUM, line, len);
}

/* ==============================================================
 * Command dispatch
 * ============================================================== */
static bool parse_18_floats(const char *payload, float out[NUM_SERVOS])
{
    int parsed = sscanf(payload,
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
        &out[0],  &out[1],  &out[2],
        &out[3],  &out[4],  &out[5],
        &out[6],  &out[7],  &out[8],
        &out[9],  &out[10], &out[11],
        &out[12], &out[13], &out[14],
        &out[15], &out[16], &out[17]);
    return (parsed == NUM_SERVOS);
}

static void drive_full_frame(const char *tag_label, float angles_f[NUM_SERVOS])
{
    int angles_int[NUM_SERVOS];
    for (int i = 0; i < NUM_SERVOS; ++i) {
        /* Clamp in float domain first so negative or NaN-ish inputs
         * never reach the uint16 cast, then round to nearest degree. */
        float a = clamp_float(angles_f[i], 0.0f, 180.0f);
        angles_int[i] = (int)(a + 0.5f);
    }

    bool ok = true;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        int base = leg * JOINTS_PER_LEG;
        esp_err_t err = set_leg_angles(leg,
                                       angles_int[base + 0],
                                       angles_int[base + 1],
                                       angles_int[base + 2]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "servo write fail leg=%d tag=%s err=%s",
                     leg + 1, tag_label, esp_err_to_name(err));
            ok = false;
            break;
        }
    }

    if (ok) {
        g_last_cmd_us = esp_timer_get_time();
        if (!g_pwm_unlocked) {
            g_pwm_unlocked = true;
            ESP_LOGI(TAG, "PWM unlocked by first Pi frame (tag=%s)", tag_label);
        }
    }
}

static void handle_full_frame(const char *tag_label, const char *payload)
{
    float angles_f[NUM_SERVOS];
    if (!parse_18_floats(payload, angles_f)) {
        ESP_LOGW(TAG, "bad float count for tag=%s payload=%.40s", tag_label, payload);
        return;
    }
    drive_full_frame(tag_label, angles_f);
}

static void handle_home(void)
{
    ESP_LOGI(TAG, "H: home -> safe pose (90,90,90)x6");
    if (apply_safe_pose() == ESP_OK) {
        g_last_cmd_us = esp_timer_get_time();
        g_pwm_unlocked = true;
    }
}

static void handle_stop(void)
{
    ESP_LOGW(TAG, "S: stop -> safe pose (PWM kept active)");
    if (apply_safe_pose() == ESP_OK) {
        g_last_cmd_us = esp_timer_get_time();
        g_pwm_unlocked = true;
    }
}

static void handle_speed(const char *payload)
{
    int percent = 0;
    if (sscanf(payload, "%d", &percent) != 1) {
        ESP_LOGW(TAG, "V: bad percent payload=%.16s", payload);
        return;
    }
    percent = clamp_int(percent, 0, 100);
    g_speed_percent = percent;
    ESP_LOGI(TAG, "V: speed = %d%% (advisory)", percent);
}

static void handle_query(void)
{
    bno055_euler_t euler = { 0.0f, 0.0f, 0.0f };
    bno055_read_euler_deg(&euler);  /* ignores error, zeros on failure */

    int64_t now_us = esp_timer_get_time();
    uint32_t uptime_ms = (uint32_t)(now_us / 1000);
    int32_t  age_ms    = (g_last_cmd_us != 0)
                         ? (int32_t)((now_us - g_last_cmd_us) / 1000)
                         : -1;

    char buf[160];
    int n = snprintf(buf, sizeof(buf),
                     "ST:%.2f,%.2f,%.2f,%u,%d\n",
                     euler.heading_deg, euler.roll_deg, euler.pitch_deg,
                     (unsigned)uptime_ms, (int)age_ms);
    if (n > 0 && n < (int)sizeof(buf)) {
        pi_uart_write_line(buf, (size_t)n);
    }
}

/* Returns true if the 1- or 2-char prefix matches a known direction tag. */
static bool is_direction_tag(const char *s, size_t len)
{
    if (len == 1) {
        return (s[0] == 'N' || s[0] == 'F' || s[0] == 'B' ||
                s[0] == 'L' || s[0] == 'R');
    }
    if (len == 2) {
        return (s[0] == 'L' && s[1] == 'L') ||
               (s[0] == 'R' && s[1] == 'R');
    }
    return false;
}

static void dispatch_line(char *line)
{
    /* Trim trailing whitespace just in case. */
    size_t slen = strlen(line);
    while (slen > 0 && (line[slen - 1] == '\r' || line[slen - 1] == ' ' ||
                        line[slen - 1] == '\t')) {
        line[--slen] = '\0';
    }
    if (slen == 0) return;

    const char *colon = strchr(line, ':');
    if (colon == NULL) {
        ESP_LOGW(TAG, "no colon: %s", line);
        return;
    }

    size_t prefix_len = (size_t)(colon - line);
    const char *payload = colon + 1;

    if (prefix_len == 1) {
        char c = line[0];
        switch (c) {
            case 'M':
                handle_full_frame("M", payload);
                return;
            case 'H':
                handle_home();
                return;
            case 'S':
                handle_stop();
                return;
            case 'V':
                handle_speed(payload);
                return;
            case 'Q':
                handle_query();
                return;
            default:
                break;
        }
    }

    if (is_direction_tag(line, prefix_len)) {
        char tag[3] = {0};
        tag[0] = line[0];
        if (prefix_len == 2) tag[1] = line[1];
        handle_full_frame(tag, payload);
        return;
    }

    ESP_LOGW(TAG, "unknown prefix (len=%u): %.*s",
             (unsigned)prefix_len, (int)prefix_len, line);
}

/* ==============================================================
 * UART RX task
 * ============================================================== */
static void pi_uart_rx_task(void *arg)
{
    static char    rx_line[RX_LINE_MAX];
    static uint8_t rx_buf[128];
    int idx = 0;

    ESP_LOGI(TAG, "UART RX task started");

    while (1) {
        int n = uart_read_bytes(PI_UART_NUM, rx_buf, sizeof(rx_buf),
                                pdMS_TO_TICKS(20));
        if (n <= 0) continue;

        for (int i = 0; i < n; ++i) {
            char ch = (char)rx_buf[i];

            if (ch == '\n' || ch == '\r') {
                if (idx == 0) continue;
                rx_line[idx] = '\0';
                dispatch_line(rx_line);
                idx = 0;
            } else if (idx < (int)sizeof(rx_line) - 1) {
                rx_line[idx++] = ch;
            } else {
                ESP_LOGW(TAG, "rx line overflow, dropping");
                idx = 0;
            }
        }
    }
}

/* ==============================================================
 * IMU debug task (log-only, no uplink in this revision)
 * ============================================================== */
static void imu_debug_task(void *arg)
{
    bno055_euler_t euler;

    while (1) {
        if (g_bno_ready && bno055_read_euler_deg(&euler) == ESP_OK) {
            ESP_LOGD(TAG, "IMU heading=%.2f roll=%.2f pitch=%.2f",
                     euler.heading_deg, euler.roll_deg, euler.pitch_deg);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ==============================================================
 * app_main
 * ============================================================== */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Shared I2C bus for both PCA9685 + BNO055 */
    ESP_ERROR_CHECK(i2c_master_init());

    /* UART to Pi */
    pi_uart_init();

    /* PWM drivers — initialize but do NOT drive any channel.
     * Per boot policy, servos stay inert until the first Pi frame arrives. */
    ESP_ERROR_CHECK(pca9685_init_one(PCA9685_ADDR_0));
    ESP_ERROR_CHECK(pca9685_init_one(PCA9685_ADDR_1));

    /* BNO055 is best-effort: the robot should still run if the IMU is
     * absent or fails to come up. handle_query() will report zeros. */
    if (bno055_init() != ESP_OK) {
        ESP_LOGW(TAG, "BNO055 init failed; IMU fields will report zero");
    }

    xTaskCreate(pi_uart_rx_task, "pi_uart_rx", 4096, NULL, 6, NULL);
    xTaskCreate(imu_debug_task,  "imu_debug",  3072, NULL, 3, NULL);

    ESP_LOGI(TAG, "Ready. Waiting for first Pi frame to unlock PWM.");
}
