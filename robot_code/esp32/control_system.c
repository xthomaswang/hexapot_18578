#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "driver/i2c.h"

static const char *TAG = "HEXAPOD_RX";

/* ---------------- UART config (Pi <-> ESP32) ---------------- */
#define PI_UART_NUM         UART_NUM_2
#define PI_UART_TX_PIN      17
#define PI_UART_RX_PIN      16
#define PI_UART_BAUD        115200
#define UART_BUF_SIZE       512

/* ---------------- I2C config ---------------- */
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_TIMEOUT_MS          100

/* ---------------- PCA9685 config ---------------- */
#define PCA9685_ADDR_0          0x40    // legs 1,2,3
#define PCA9685_ADDR_1          0x41    // legs 4,5,6 (A0 soldered)

#define PCA9685_MODE1           0x00
#define PCA9685_PRESCALE        0xFE
#define PCA9685_LED0_ON_L       0x06
#define PCA9685_RESTART_AI      0xA1
#define PCA9685_SLEEP_BIT       0x10

/* ---------------- BNO055 config ---------------- */
#define BNO055_ADDR             0x28    // default address; use 0x29 if ADR is high
#define BNO055_CHIP_ID_REG      0x00
#define BNO055_CHIP_ID_VAL      0xA0
#define BNO055_OPR_MODE_REG     0x3D
#define BNO055_PWR_MODE_REG     0x3E
#define BNO055_SYS_TRIGGER_REG  0x3F
#define BNO055_UNIT_SEL_REG     0x3B
#define BNO055_PAGE_ID_REG      0x07
#define BNO055_EULER_H_LSB      0x1A

#define BNO055_PWR_NORMAL       0x00
#define BNO055_MODE_CONFIG      0x00
#define BNO055_MODE_NDOF        0x0C

/* ---------------- Servo config ---------------- */
#define SERVO_MIN_DEG           30
#define SERVO_MAX_DEG           150
#define SERVO_MIN_US            1000
#define SERVO_MAX_US            2000
#define SERVO_PWM_FREQ_HZ       50

#define NUM_LEGS                6
#define JOINTS_PER_LEG          3
#define NUM_SERVOS              (NUM_LEGS * JOINTS_PER_LEG)

/* ---------------- Per-leg channel mapping ----------------
   Each PCA9685 handles 3 legs using channels 0-2, 4-6, 8-10.
   Legs 1-3 on 0x40, legs 4-6 on 0x41.
*/
typedef struct {
    uint8_t pca_addr;
    uint8_t coxa_ch;
    uint8_t femur_ch;
    uint8_t tibia_ch;
} leg_map_t;

static const leg_map_t g_leg_map[NUM_LEGS] = {
    {PCA9685_ADDR_0,  0,  1,  2},   // leg 1
    {PCA9685_ADDR_0,  4,  5,  6},   // leg 2
    {PCA9685_ADDR_0,  8,  9, 10},   // leg 3
    {PCA9685_ADDR_1,  0,  1,  2},   // leg 4
    {PCA9685_ADDR_1,  4,  5,  6},   // leg 5
    {PCA9685_ADDR_1,  8,  9, 10},   // leg 6
};

/* ---------------- Optional per-servo neutral pose ---------------- */
static const uint16_t g_neutral_pose_deg[NUM_LEGS][JOINTS_PER_LEG] = {
    {45, 45, 45},
    {45, 45, 45},
    {45, 45, 45},
    {45, 45, 45},
    {45, 45, 45},
    {45, 45, 45},
};

/* ---------------- Helper functions ---------------- */
static uint16_t clamp_u16(uint16_t value, uint16_t min_val, uint16_t max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

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

static esp_err_t pca9685_init_one(uint8_t addr)
{
    uint8_t oldmode = 0;

    ESP_ERROR_CHECK(i2c_write_reg(addr, PCA9685_MODE1, 0x00));
    ESP_ERROR_CHECK(i2c_read_reg(addr, PCA9685_MODE1, &oldmode));

    uint8_t sleep_mode = (oldmode & 0x7F) | PCA9685_SLEEP_BIT;
    ESP_ERROR_CHECK(i2c_write_reg(addr, PCA9685_MODE1, sleep_mode));

    /* 25 MHz / (4096 * 50 Hz) - 1 ~= 121 */
    ESP_ERROR_CHECK(i2c_write_reg(addr, PCA9685_PRESCALE, 121));
    ESP_ERROR_CHECK(i2c_write_reg(addr, PCA9685_MODE1, oldmode));

    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_ERROR_CHECK(i2c_write_reg(addr, PCA9685_MODE1, PCA9685_RESTART_AI));

    ESP_LOGI(TAG, "PCA9685 initialized at address 0x%02X (%d Hz)", addr, SERVO_PWM_FREQ_HZ);
    return ESP_OK;
}

static uint16_t angle_to_pulse_us(uint16_t angle_deg)
{
    angle_deg = clamp_u16(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);

    float t = (float)(angle_deg - SERVO_MIN_DEG) /
              (float)(SERVO_MAX_DEG - SERVO_MIN_DEG);

    return (uint16_t)(SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US));
}

static uint16_t pulse_us_to_counts(uint16_t pulse_us)
{
    float counts = ((float)pulse_us * 4096.0f) / 20000.0f;
    if (counts < 0.0f) counts = 0.0f;
    if (counts > 4095.0f) counts = 4095.0f;
    return (uint16_t)counts;
}

static esp_err_t pca9685_set_pwm(uint8_t addr, uint8_t channel, uint16_t on_count, uint16_t off_count)
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

static esp_err_t set_servo_angle_raw(uint8_t addr, uint8_t channel, uint16_t angle_deg)
{
    uint16_t pulse_us = angle_to_pulse_us(angle_deg);
    uint16_t counts = pulse_us_to_counts(pulse_us);
    return pca9685_set_pwm(addr, channel, 0, counts);
}

static esp_err_t set_leg_angles(uint8_t leg_idx, uint16_t coxa_deg, uint16_t femur_deg, uint16_t tibia_deg)
{
    if (leg_idx >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }

    const leg_map_t *leg = &g_leg_map[leg_idx];

    esp_err_t err0 = set_servo_angle_raw(leg->pca_addr, leg->coxa_ch,  coxa_deg);
    esp_err_t err1 = set_servo_angle_raw(leg->pca_addr, leg->femur_ch, femur_deg);
    esp_err_t err2 = set_servo_angle_raw(leg->pca_addr, leg->tibia_ch, tibia_deg);

    if (err0 != ESP_OK) return err0;
    if (err1 != ESP_OK) return err1;
    if (err2 != ESP_OK) return err2;
    return ESP_OK;
}

static esp_err_t initialize_neutral_pose(void)
{
    for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        esp_err_t err = set_leg_angles(leg,
                                       g_neutral_pose_deg[leg][0],
                                       g_neutral_pose_deg[leg][1],
                                       g_neutral_pose_deg[leg][2]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize leg %u", (unsigned)(leg + 1));
            return err;
        }
    }

    ESP_LOGI(TAG, "All legs initialized to neutral pose");
    return ESP_OK;
}

/* ---------------- BNO055 ---------------- */
static esp_err_t bno055_init(void)
{
    uint8_t chip_id = 0;

    ESP_ERROR_CHECK(i2c_write_reg(BNO055_ADDR, BNO055_PAGE_ID_REG, 0x00));
    ESP_ERROR_CHECK(i2c_read_reg(BNO055_ADDR, BNO055_CHIP_ID_REG, &chip_id));

    if (chip_id != BNO055_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "BNO055 chip ID mismatch: got 0x%02X, expected 0x%02X", chip_id, BNO055_CHIP_ID_VAL);
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(i2c_write_reg(BNO055_ADDR, BNO055_OPR_MODE_REG, BNO055_MODE_CONFIG));
    vTaskDelay(pdMS_TO_TICKS(25));

    ESP_ERROR_CHECK(i2c_write_reg(BNO055_ADDR, BNO055_SYS_TRIGGER_REG, 0x20));
    vTaskDelay(pdMS_TO_TICKS(700));

    ESP_ERROR_CHECK(i2c_read_reg(BNO055_ADDR, BNO055_CHIP_ID_REG, &chip_id));
    if (chip_id != BNO055_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "BNO055 chip ID mismatch after reset: got 0x%02X", chip_id);
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(i2c_write_reg(BNO055_ADDR, BNO055_PWR_MODE_REG, BNO055_PWR_NORMAL));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(i2c_write_reg(BNO055_ADDR, BNO055_PAGE_ID_REG, 0x00));
    ESP_ERROR_CHECK(i2c_write_reg(BNO055_ADDR, BNO055_UNIT_SEL_REG, 0x00));
    ESP_ERROR_CHECK(i2c_write_reg(BNO055_ADDR, BNO055_SYS_TRIGGER_REG, 0x00));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(i2c_write_reg(BNO055_ADDR, BNO055_OPR_MODE_REG, BNO055_MODE_NDOF));
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "BNO055 initialized at address 0x%02X in NDOF mode", BNO055_ADDR);
    return ESP_OK;
}

typedef struct {
    float heading_deg;
    float roll_deg;
    float pitch_deg;
} bno055_euler_t;

static esp_err_t bno055_read_euler_deg(bno055_euler_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = BNO055_EULER_H_LSB;
    uint8_t data[6] = {0};

    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM,
                                                 BNO055_ADDR,
                                                 &reg,
                                                 1,
                                                 data,
                                                 sizeof(data),
                                                 pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }

    int16_t heading_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t roll_raw    = (int16_t)((data[3] << 8) | data[2]);
    int16_t pitch_raw   = (int16_t)((data[5] << 8) | data[4]);

    out->heading_deg = heading_raw / 16.0f;
    out->roll_deg    = roll_raw / 16.0f;
    out->pitch_deg   = pitch_raw / 16.0f;
    return ESP_OK;
}

/* ---------------- UART ---------------- */
static void pi_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = PI_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(PI_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PI_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PI_UART_NUM, PI_UART_TX_PIN, PI_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialized for Raspberry Pi link");
}

/* ---------------- Task: receive 6-leg servo commands from Pi ----------------
   Expected format:
   L:a0,a1,a2,b0,b1,b2,c0,c1,c2,d0,d1,d2,e0,e1,e2,f0,f1,f2
   where each consecutive 3-tuple is one leg's [coxa,femur,tibia].
*/
static void pi_uart_rx_task(void *arg)
{
    static char rx_line[256];
    int idx = 0;
    uint8_t ch;

    while (1) {
        int len = uart_read_bytes(PI_UART_NUM, &ch, 1, pdMS_TO_TICKS(20));

        if (len <= 0) {
            continue;
        }

        if (ch == '
' || ch == '
') {
            if (idx == 0) {
                continue;
            }

            rx_line[idx] = '␀';
            idx = 0;

            char *payload = strchr(rx_line, ':');
            if (payload == NULL) {
                ESP_LOGW(TAG, "UART parse failed, no colon found: %s", rx_line);
                continue;
            }

            payload++;

            float angle_f[NUM_SERVOS] = {0};
            int parsed = sscanf(payload,
                                "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                                &angle_f[0],  &angle_f[1],  &angle_f[2],
                                &angle_f[3],  &angle_f[4],  &angle_f[5],
                                &angle_f[6],  &angle_f[7],  &angle_f[8],
                                &angle_f[9],  &angle_f[10], &angle_f[11],
                                &angle_f[12], &angle_f[13], &angle_f[14],
                                &angle_f[15], &angle_f[16], &angle_f[17]);

            if (parsed != NUM_SERVOS) {
                ESP_LOGW(TAG, "UART parse failed, expected %d floats but got %d | raw=%s",
                         NUM_SERVOS, parsed, rx_line);
                continue;
            }

            uint16_t angle_deg[NUM_SERVOS];
            for (int i = 0; i < NUM_SERVOS; ++i) {
                angle_deg[i] = clamp_u16((uint16_t)(angle_f[i] + 0.5f), SERVO_MIN_DEG, SERVO_MAX_DEG);
            }

            bool ok = true;
            for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
                esp_err_t err = set_leg_angles(leg,
                                               angle_deg[leg * 3 + 0],
                                               angle_deg[leg * 3 + 1],
                                               angle_deg[leg * 3 + 2]);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Servo write failed for leg %u | raw=%s", (unsigned)(leg + 1), rx_line);
                    ok = false;
                    break;
                }
            }

            if (ok) {
                ESP_LOGI(TAG,
                         "Applied angles | L1=[%u,%u,%u] L2=[%u,%u,%u] L3=[%u,%u,%u] L4=[%u,%u,%u] L5=[%u,%u,%u] L6=[%u,%u,%u]",
                         angle_deg[0], angle_deg[1], angle_deg[2],
                         angle_deg[3], angle_deg[4], angle_deg[5],
                         angle_deg[6], angle_deg[7], angle_deg[8],
                         angle_deg[9], angle_deg[10], angle_deg[11],
                         angle_deg[12], angle_deg[13], angle_deg[14],
                         angle_deg[15], angle_deg[16], angle_deg[17]);
            }
        } else {
            if (idx < (int)sizeof(rx_line) - 1) {
                rx_line[idx++] = (char)ch;
            } else {
                ESP_LOGW(TAG, "UART line overflow, dropping frame");
                idx = 0;
            }
        }
    }
}

/* ---------------- Optional IMU debug task ---------------- */
static void imu_debug_task(void *arg)
{
    bno055_euler_t euler;

    while (1) {
        esp_err_t err = bno055_read_euler_deg(&euler);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "BNO055 Euler | heading=%.2f roll=%.2f pitch=%.2f",
                     euler.heading_deg, euler.roll_deg, euler.pitch_deg);
        } else {
            ESP_LOGW(TAG, "Failed to read BNO055 Euler data");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Shared I2C bus for both PCA9685 drivers + BNO055 */
    ESP_ERROR_CHECK(i2c_master_init());

    /* UART link from Raspberry Pi -> ESP32 */
    pi_uart_init();

    /* Initialize chained PWM drivers */
    ESP_ERROR_CHECK(pca9685_init_one(PCA9685_ADDR_0));
    ESP_ERROR_CHECK(pca9685_init_one(PCA9685_ADDR_1));

    /* Initialize BNO055 on same I2C bus */
    ESP_ERROR_CHECK(bno055_init());

    /* Move all six legs to neutral pose */
    ESP_ERROR_CHECK(initialize_neutral_pose());

    xTaskCreate(pi_uart_rx_task, "pi_uart_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(imu_debug_task,  "imu_debug_task",  4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Ready: Pi UART -> ESP32 -> dual PCA9685, with BNO055 initialized on I2C");
}
