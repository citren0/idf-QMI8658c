
#ifndef QMI8658C_H_
#define QMI8658C_H_

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "math.h"

#define PI                          3.14159265

#define RAD_TO_DEG(x)               (x * (360.0 / (2.0 * PI)))
#define DEG_TO_RAD(x)               (x * ((2.0 * PI) / 360.0))
#define MS_TO_S(x)                  ((float)(x) / 1000000.0)

#define QMI_I2C_SCL                 GPIO_NUM_10
#define QMI_I2C_SDA                 GPIO_NUM_11
#define QMI_I2C_NUM                 I2C_NUM_0
#define QMI_I2C_FREQ_HZ             200000
#define QMI_I2C_TIMEOUT_MS          1000
#define QMI8658C_ADDR               0x6B
#define QMI_INTERRUPT1_PIN          GPIO_NUM_38
#define QMI_FIFO_QUEUE_TIMEOUT      100
#define QMI_FIFO_WATERMARK_LEVEL    64 // 64 samples of 12 bytes each, ~700 bytes to read per interrupt.
#define QMI_FIFO_TOTAL_SIZE         1536
#define QMI_COMP_RATIO              0.98

/* General purpose registers */
#define QMI8658_WHO_AM_I 0x00
#define QMI8658_REVISION 0x01
#define QMI8658_RESET 0x60
#define QMI8658_CTRL1 0x02
#define QMI8658_CTRL2 0x03
#define QMI8658_CTRL3 0x04
#define QMI8658_CTRL4 0x05
#define QMI8658_CTRL5 0x06
#define QMI8658_CTRL7 0x08
#define QMI8658_CTRL8 0x09
#define QMI8658_CTRL9 0x0A
#define QMI8658_STATUS1 0x2F
/* FIFO Registers */
#define QMI8658_FIFO_WTM_TH 0x13
#define QMI8658_FIFO_CTRL 0x14
#define QMI8658_FIFO_SAMPLE_COUNT_L 0x15
#define QMI8658_FIFO_STATUS 0x16
#define QMI8658_FIFO_DATA 0x17
/* Accelerometer Registers */
#define QMI8658_ACC_X_L 0x35
#define QMI8658_ACC_X_H 0x36
#define QMI8658_ACC_Y_L 0x37
#define QMI8658_ACC_Y_H 0x38
#define QMI8658_ACC_Z_L 0x39
#define QMI8658_ACC_Z_H 0x3A
/* Gyroscope Registers */
#define QMI8658_GYR_X_L 0x3B
#define QMI8658_GYR_X_H 0x3C
#define QMI8658_GYR_Y_L 0x3D
#define QMI8658_GYR_Y_H 0x3E
#define QMI8658_GYR_Z_L 0x3F
#define QMI8658_GYR_Z_H 0x40
/* Temperature Sensor Registers */
#define QMI8658_TEMP_L 0x33
#define QMI8658_TEMP_H 0x34
#define TEMPERATURE_SENSOR_RESOLUTION (1 << 8)

// CTRL Commands
#define CTRL_CMD_REQ_FIFO 0x05
// Accelerometer scale sensitivity values.
#define ACC_SCALE_SENSITIVITY_2G  (1 << 14)
#define ACC_SCALE_SENSITIVITY_4G  (1 << 13)
#define ACC_SCALE_SENSITIVITY_8G  (1 << 12)
#define ACC_SCALE_SENSITIVITY_16G (1 << 11)
// Gyroscope scale sensitivity values.
#define GYRO_SCALE_SENSITIVITY_16DPS   (1 << 11)
#define GYRO_SCALE_SENSITIVITY_32DPS   (1 << 10)
#define GYRO_SCALE_SENSITIVITY_64DPS   (1 << 9)
#define GYRO_SCALE_SENSITIVITY_128DPS  (1 << 8)
#define GYRO_SCALE_SENSITIVITY_256DPS  (1 << 7)
#define GYRO_SCALE_SENSITIVITY_512DPS  (1 << 6)
#define GYRO_SCALE_SENSITIVITY_1024DPS (1 << 5)
#define GYRO_SCALE_SENSITIVITY_2048DPS (1 << 4)
// A+G low pass filter settings.
#define LPF_MODE_2P66 0b00
#define LPF_MODE_3P63 0b01
#define LPF_MODE_5P39 0b10
#define LPF_MODE_13P37 0b11
// FIFO Size Settings
#define FIFO_SIZE_16 0b00
#define FIFO_SIZE_32 0b01
#define FIFO_SIZE_64 0b10
#define FIFO_SIZE_128 0b11
// FIFO Mode Settings
#define FIFO_MODE_BYPASS 0b00
#define FIFO_MODE_FIFO 0b01
#define FIFO_MODE_STREAM 0b10
#define FIFO_MODE_RESERVED 0b11


typedef enum
{
    QMI8658C_MODE_ACC_ONLY = 1,
    QMI8658C_MODE_GYRO_ONLY,
    QMI8658C_MODE_DUAL,
} qmi8658c_mode_t;

typedef enum
{
    QMI8658C_ACC_ODR_8000,
    QMI8658C_ACC_ODR_4000,
    QMI8658C_ACC_ODR_2000,
    QMI8658C_ACC_ODR_1000,
    QMI8658C_ACC_ODR_500,
    QMI8658C_ACC_ODR_250,
    QMI8658C_ACC_ODR_125,
    QMI8658C_ACC_ODR_62_5,
    QMI8658C_ACC_ODR_31_25,
    QMI8658C_ACC_ODR_128 = 12,
    QMI8658C_ACC_ODR_21,
    QMI8658C_ACC_ODR_11,
    QMI8658C_ACC_ODR_3,
} qmi8658c_acc_odr_t;

typedef enum
{
    QMI8658C_GYRO_ODR_8000,
    QMI8658C_GYRO_ODR_4000,
    QMI8658C_GYRO_ODR_2000,
    QMI8658C_GYRO_ODR_1000,
    QMI8658C_GYRO_ODR_500,
    QMI8658C_GYRO_ODR_250,
    QMI8658C_GYRO_ODR_125,
    QMI8658C_GYRO_ODR_62_5,
    QMI8658C_GYRO_ODR_31_25,
} qmi8658c_gyro_odr_t;

typedef enum
{
    QMI8658C_ACC_SCALE_2G,
    QMI8658C_ACC_SCALE_4G,
    QMI8658C_ACC_SCALE_8G,
    QMI8658C_ACC_SCALE_16G,
} qmi8658c_acc_scale_t;

typedef enum
{
    QMI8658C_GYRO_SCALE_16DPS,
    QMI8658C_GYRO_SCALE_32DPS,
    QMI8658C_GYRO_SCALE_64DPS,
    QMI8658C_GYRO_SCALE_128DPS,
    QMI8658C_GYRO_SCALE_256DPS,
    QMI8658C_GYRO_SCALE_512DPS,
    QMI8658C_GYRO_SCALE_1024DPS,
    QMI8658C_GYRO_SCALE_2048DPS,
} qmi8658c_gyro_scale_t;

typedef struct
{
    float x;
    float y;
    float z;
} qmi8658c_axes_t;

typedef struct
{
    uint16_t acc_sensitivity;
    uint8_t acc_scale;
    uint16_t gyro_sensitivity;
    uint8_t gyro_scale;
    uint8_t who_am_i;
    uint8_t revision;
} qmi_ctx_t;

typedef struct
{
    qmi8658c_axes_t acc;
    qmi8658c_axes_t gyro;
    float temperature;
} qmi8658c_data_t;

typedef struct
{
    qmi8658c_mode_t mode;
    qmi8658c_acc_scale_t acc_scale;
    qmi8658c_acc_odr_t acc_odr;
    qmi8658c_gyro_scale_t gyro_scale;
    qmi8658c_gyro_odr_t gyro_odr;
} qmi8658c_config_t;

typedef struct
{
    float pitch;
    float yaw;
    float roll;
    int64_t last_update;
} qmi8658c_complimentary_t;

typedef struct
{
    uint8_t ax_l;
    uint8_t ax_h;
    uint8_t ay_l;
    uint8_t ay_h;
    uint8_t az_l;
    uint8_t az_h;
    uint8_t gx_l;
    uint8_t gx_h;
    uint8_t gy_l;
    uint8_t gy_h;
    uint8_t gz_l;
    uint8_t gz_h;
} qmi8658c_fifo_reading_t;



void init_qmi(i2c_master_bus_handle_t bus_handle);
void qmi_init_complimentary(qmi8658c_complimentary_t * comp);

esp_err_t qmi_fifo_setup(void);
void qmi_deinit_fifo(void);
uint8_t qmi_fifo_is_read_ready(void);
esp_err_t qmi_fifo_consume(qmi8658c_fifo_reading_t * * buf, uint16_t * readings_available);
void qmi_fifo_update_complimentary_with_readings(qmi8658c_complimentary_t * complimentary, qmi8658c_fifo_reading_t * readings, uint16_t num_readings);
void qmi_calibrate_with_fifo(void);

esp_err_t qmi_get_accel(float * x, float * y, float * z);
esp_err_t qmi_get_gyro(float * x, float * y, float * z);
esp_err_t qmi_get_temp(float * temperature);
esp_err_t qmi_read_all_data(qmi8658c_data_t * data);

#endif // QMI8658C_H_
