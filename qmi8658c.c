
#include "qmi8658c.h"


static char * TAG = "PCF85063";
static i2c_master_dev_handle_t dev_handle;
static qmi_ctx_t qmi_ctx;
static QueueHandle_t qmi_int_queue = NULL;
static uint8_t * fifo_buffer = NULL;
static float gx_cal = 0.0, gy_cal = 0.0, gz_cal = 0.0;
static uint16_t acc_scale_sensitivity_table[4] =
{
    ACC_SCALE_SENSITIVITY_2G,
    ACC_SCALE_SENSITIVITY_4G,
    ACC_SCALE_SENSITIVITY_8G,
    ACC_SCALE_SENSITIVITY_16G
};
static uint16_t gyro_scale_sensitivity_table[8] =
{
    GYRO_SCALE_SENSITIVITY_16DPS,
    GYRO_SCALE_SENSITIVITY_32DPS,
    GYRO_SCALE_SENSITIVITY_64DPS,
    GYRO_SCALE_SENSITIVITY_128DPS,
    GYRO_SCALE_SENSITIVITY_256DPS,
    GYRO_SCALE_SENSITIVITY_512DPS,
    GYRO_SCALE_SENSITIVITY_1024DPS,
    GYRO_SCALE_SENSITIVITY_2048DPS
};
static float acc_odr_table[16] =
{
    8000, // QMI8658C_ACC_ODR_8000
    4000, // QMI8658C_ACC_ODR_4000
    2000, // QMI8658C_ACC_ODR_2000
    1000, // QMI8658C_ACC_ODR_1000
    500, // QMI8658C_ACC_ODR_500
    250, // QMI8658C_ACC_ODR_250
    125, // QMI8658C_ACC_ODR_125
    62.5, // QMI8658C_ACC_ODR_62_5
    31.25, // QMI8658C_ACC_ODR_31_25
    0, // N/A
    0, // N/A
    0, // N/A
    128, // QMI8658C_ACC_ODR_128 = 12
    21, // QMI8658C_ACC_ODR_21
    11, // QMI8658C_ACC_ODR_11
    3, // QMI8658C_ACC_ODR_3
};
static float gyro_odr_table[9] =
{
    8000, // QMI8658C_GYRO_ODR_8000
    4000, // QMI8658C_GYRO_ODR_4000
    2000, // QMI8658C_GYRO_ODR_2000
    1000, // QMI8658C_GYRO_ODR_1000
    500, // QMI8658C_GYRO_ODR_500
    250, // QMI8658C_GYRO_ODR_250
    125, // QMI8658C_GYRO_ODR_125
    62.5, // QMI8658C_GYRO_ODR_62_5
    31.25, // QMI8658C_GYRO_ODR_31_25
};


static esp_err_t qmi_read_reg(uint8_t reg_addr, uint8_t * data, size_t len)
{
    uint8_t send_buf[1] = { reg_addr };

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, send_buf, sizeof(send_buf), data, len, pdMS_TO_TICKS(QMI_I2C_TIMEOUT_MS));
    
    return ret;
}


static esp_err_t qmi_write(uint8_t * data, size_t len)
{
    esp_err_t ret = i2c_master_transmit(dev_handle, data, len, pdMS_TO_TICKS(QMI_I2C_TIMEOUT_MS));
    return ret;
}


static esp_err_t send_command(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    return qmi_write(data, 2);
}


static esp_err_t read_register(uint8_t reg, uint8_t * value)
{
    return qmi_read_reg(reg, value, 1);
}


static esp_err_t qmi_reset(void)
{
    uint8_t cmd = 0xB0;
    esp_err_t err = send_command(QMI8658_RESET, cmd);
    return err;
}


static esp_err_t qmi_check_whoami(void)
{
    uint8_t who_am_i = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_WHO_AM_I, &who_am_i));
    if (who_am_i != 0x05)
    {
        ESP_LOGE(TAG, "This isn't a QMI8658.");
        return ESP_FAIL;
    }

    uint8_t revision = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_REVISION, &revision));
    if (revision != 0x7C)
    {
        ESP_LOGE(TAG, "This QMI revision is not supported by this driver.");
        return ESP_FAIL;
    }

    return ESP_OK;
}


static void IRAM_ATTR qmi_ae_interrupt(void * arg)
{
    // Dummy value to signify read is ready.
    uint8_t data = 1;
    if (qmi_int_queue)
    {
        xQueueSendFromISR(qmi_int_queue, &data, NULL);
    }
}


static uint16_t qmi_get_fifo_bytes_available(void)
{
    uint8_t samples_l = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_FIFO_SAMPLE_COUNT_L, &samples_l));

    uint8_t samples_h = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_FIFO_STATUS, &samples_h));

    return 2 * (((samples_h & 0x03) << 8) | samples_l);
}


static esp_err_t qmi_enable_fifo_read_mode(void)
{
    esp_err_t err = send_command(QMI8658_CTRL9, CTRL_CMD_REQ_FIFO);
    return err;
}


static esp_err_t qmi_disable_fifo_read_mode(void)
{
    esp_err_t err = 0;

    uint8_t fifo_ctrl = 0;
    err |= read_register(QMI8658_FIFO_CTRL, &fifo_ctrl);
    fifo_ctrl &= ~(1 << 7); // Set FIFO to read mode.
    err |= send_command(QMI8658_FIFO_CTRL, fifo_ctrl);

    return err;
}


/* ----- Public General Purpose Setup ----- */


void init_qmi(i2c_master_bus_handle_t bus_handle)
{
    // Add device to I2C bus.
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMI8658C_ADDR,
        .scl_speed_hz = QMI_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
}


void qmi_init_complimentary(qmi8658c_complimentary_t * comp)
{
    comp->last_update = esp_timer_get_time();
    comp->pitch = 0.0;
    comp->yaw = 0.0;
    comp->roll = 0.0;
}


/* ----- Public FIFO Related Functions ----- */


esp_err_t qmi_fifo_setup(qmi8658c_config_t * config)
{
    if (acc_odr_table[config->acc_odr] != gyro_odr_table[config->gyro_odr])
    {
        ESP_LOGE(TAG, "Accelerometer ODR and gyrometer ODR must be the same.");
        return ESP_ERR_INVALID_ARG;
    }

    // Save for later.
    qmi_ctx.acc_scale = config->acc_scale;
    qmi_ctx.acc_sensitivity = acc_scale_sensitivity_table[config->acc_scale];
    qmi_ctx.acc_odr = acc_odr_table[config->acc_odr];

    qmi_ctx.gyro_scale = config->gyro_scale;
    qmi_ctx.gyro_sensitivity = gyro_scale_sensitivity_table[config->gyro_scale];
    qmi_ctx.gyro_odr = gyro_odr_table[config->gyro_odr];

    // Interrupt will push to queue to signal read is ready.
    if (!qmi_int_queue)
    {
        qmi_int_queue = xQueueCreate(10, sizeof(uint8_t));
    }

    // Reset and give time for boot-up.
    ESP_ERROR_CHECK(qmi_reset());

    vTaskDelay(pdMS_TO_TICKS(150));

    // Verify we have the correct chip.
    ESP_ERROR_CHECK(qmi_check_whoami());

    // Issue config command set.
    uint8_t ctrl1 = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_CTRL1, &ctrl1));
    ctrl1 &= ~(1 << 0); // Enable internal oscillator.
    ctrl1 &= ~(1 << 5); // Little endian.
    ctrl1 |=  (1 << 6); // Enable auto increment.
    ctrl1 |=  (1 << 3); // Enable interrupt 1.
    ctrl1 |=  (1 << 2); // FIFO interrupt to INT1.
    ESP_ERROR_CHECK(send_command(QMI8658_CTRL1, ctrl1));

    uint8_t ctrl2 = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_CTRL2, &ctrl2));
    ctrl2 &= ~(0x0F); // Clear ODR bits.
    ctrl2 |= config->acc_odr; // Set ODR bits.
    ctrl2 &= ~(0x70); // Clear scale bits.
    ctrl2 |= (config->acc_scale << 4); // Set scale bits.
    ESP_ERROR_CHECK(send_command(QMI8658_CTRL2, ctrl2));

    uint8_t ctrl3 = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_CTRL3, &ctrl3));
    ctrl3 &= ~(0x0F); // Clear ODR bits.
    ctrl3 |= config->gyro_odr; // Set ODR bits.
    ctrl3 &= ~(0x70); // Clear scale bits.
    ctrl3 |= (config->gyro_scale << 4); // Set scale bits.
    ESP_ERROR_CHECK(send_command(QMI8658_CTRL3, ctrl3));

    uint8_t ctrl5 = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_CTRL5, &ctrl5));
    ctrl5 |= (1 << 0); // Enable accelerometer low pass filter.
    ctrl5 &= ~(0b11 << 1); // Clear LPF bits.
    ctrl5 |= (LPF_MODE_5P39 << 1); // Set mode for accelerometer low pass filter.
    ctrl5 |= (1 << 4); // Enable gyro low pass filter.
    ctrl5 &= ~(0b11 << 5); // Clear LPF bits.
    ctrl5 |= (LPF_MODE_5P39 << 5); // Set mode for gyro low pass filter.
    ESP_ERROR_CHECK(send_command(QMI8658_CTRL5, ctrl5));

    uint8_t ctrl7 = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_CTRL7, &ctrl7));
    ctrl7 |= (1 << 1); // Enable gyrometer.
    ctrl7 |= (1 << 0); // Enable Accelerometer.
    ctrl7 |= (1 << 5); // Disable DRDY.
    ctrl7 &= ~(1 << 4); // Set gyrometer to full mode.
    ctrl7 &= ~(1 << 7); // Disable syncsample.
    ESP_ERROR_CHECK(send_command(QMI8658_CTRL7, ctrl7));

    uint8_t ctrl8 = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_CTRL8, &ctrl8));
    ctrl8 &= ~(1 << 0); // Disable Pedometer Engine.
    ctrl8 &= ~(1 << 1); // Disable Significant Motion Engine.
    ctrl8 &= ~(1 << 2); // Disable No Motion Engine.
    ctrl8 &= ~(1 << 3); // Disable Any Motion Engine.
    ctrl8 &= ~(1 << 4); // Disable Tap Motion Engine.
    ESP_ERROR_CHECK(send_command(QMI8658_CTRL8, ctrl8));

    uint8_t fifo_wtm_th = QMI_FIFO_WATERMARK_LEVEL; // Enable watermark interrupts.
    ESP_ERROR_CHECK(send_command(QMI8658_FIFO_WTM_TH, fifo_wtm_th));

    uint8_t fifo_ctrl = 0;
    ESP_ERROR_CHECK(read_register(QMI8658_FIFO_CTRL, &fifo_ctrl));
    fifo_ctrl &= ~(1 << 7); // Set FIFO to write mode.
    fifo_ctrl &= ~(0b11 << 2); // Clear FIFO depth bits.
    fifo_ctrl |= (FIFO_SIZE_128 << 2); // Set sample depth of FIFO.
    fifo_ctrl &= ~(0b11 << 0); // Clear FIFO mode bits.
    fifo_ctrl |= (FIFO_MODE_STREAM << 0); // Set mode of FIFO.
    ESP_ERROR_CHECK(send_command(QMI8658_FIFO_CTRL, fifo_ctrl));

    // Set up interrupt.
    gpio_config_t gpio_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << QMI_INTERRUPT1_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    gpio_install_isr_service(0); // Might already be configured. Don't error check in case it is.
    ESP_ERROR_CHECK(gpio_isr_handler_add(QMI_INTERRUPT1_PIN, qmi_ae_interrupt, NULL));

    // No memory leaks.
    if (!fifo_buffer)
    {
        fifo_buffer = (uint8_t *)malloc(QMI_FIFO_TOTAL_SIZE);
    }

    return ESP_OK;
}

esp_err_t qmi_fifo_consume(qmi8658c_fifo_reading_t * * buf, uint16_t * readings_available)
{
    uint8_t recv;
    if (qmi_int_queue && (xQueueReceive(qmi_int_queue, &recv, pdMS_TO_TICKS(QMI_FIFO_QUEUE_TIMEOUT)) == pdPASS))
    {
        esp_err_t err = 0;

        uint16_t fifo_bytes_available = qmi_get_fifo_bytes_available();

        // Make sure there are bytes to read, then do 2 basic sanity checks.
        if (
            fifo_bytes_available == 0 ||
            fifo_bytes_available % sizeof(qmi8658c_fifo_reading_t) != 0 ||
            fifo_bytes_available > QMI_FIFO_TOTAL_SIZE
        ) {
            return ESP_FAIL;
        }

        ESP_ERROR_CHECK(qmi_enable_fifo_read_mode());

        err |= qmi_read_reg(QMI8658_FIFO_DATA, fifo_buffer, fifo_bytes_available);

        ESP_ERROR_CHECK(qmi_disable_fifo_read_mode());

        *readings_available = fifo_bytes_available / sizeof(qmi8658c_fifo_reading_t);
        *buf = (qmi8658c_fifo_reading_t *)fifo_buffer;

        return err;
    }
    else
    {
        return ESP_FAIL;
    }
}

uint8_t qmi_fifo_is_read_ready(void)
{
    uint8_t peek;
    if (qmi_int_queue && (xQueuePeek(qmi_int_queue, &peek, pdMS_TO_TICKS(QMI_FIFO_QUEUE_TIMEOUT)) == pdPASS))
    {
        return 1;
    }
    return 0;
}

void qmi_fifo_update_complimentary_with_readings(qmi8658c_complimentary_t * complimentary, qmi8658c_fifo_reading_t * readings, uint16_t num_readings)
{
    float ax, ay, az, gx, gy, gz, apitch, aroll;

    float deltaT = 1.0 / qmi_ctx.acc_odr; // Assume acc and gyro odr are the same.

    for (int i = 0; i < num_readings; i++)
    {
        gx = ((float)((int16_t)((readings[i].gx_h << 8) | readings[i].gx_l))) / qmi_ctx.gyro_sensitivity;
        gx -= gx_cal;
        gy = ((float)((int16_t)((readings[i].gy_h << 8) | readings[i].gy_l))) / qmi_ctx.gyro_sensitivity;
        gy -= gy_cal;
        gz = ((float)((int16_t)((readings[i].gz_h << 8) | readings[i].gz_l))) / qmi_ctx.gyro_sensitivity;
        gz -= gz_cal;

        ax = ((float)(int16_t)((readings[i].ax_h << 8) | readings[i].ax_l)) / qmi_ctx.acc_sensitivity;
        ay = ((float)(int16_t)((readings[i].ay_h << 8) | readings[i].ay_l)) / qmi_ctx.acc_sensitivity;
        az = ((float)(int16_t)((readings[i].az_h << 8) | readings[i].az_l)) / qmi_ctx.acc_sensitivity;

        aroll = RAD_TO_DEG(atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))));
        apitch = RAD_TO_DEG(atan2(ax, sqrt(pow(ay, 2) + pow(az, 2))));

        complimentary->pitch = (QMI_COMP_RATIO * ((deltaT * gx) + complimentary->pitch)) + ((1.0 - QMI_COMP_RATIO) * apitch);
        complimentary->roll = ((QMI_COMP_RATIO * ((deltaT * gy) + complimentary->roll)) + ((1.0 - QMI_COMP_RATIO) * aroll));
        complimentary->yaw += -(deltaT * gz);
    }

    complimentary->last_update = esp_timer_get_time();
}

void qmi_calibrate_with_fifo(void)
{
    // Allow data to accumulate.
    while (qmi_fifo_is_read_ready() == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    uint16_t readings_available = 0;
    qmi8658c_fifo_reading_t * readings;
    ESP_ERROR_CHECK(qmi_fifo_consume(&readings, &readings_available));

    float gx_total = 0.0, gy_total = 0.0, gz_total = 0.0;

    for (int i = 0; i < readings_available; i++)
    {
        gx_total += -((float)((int16_t)((readings[i].gx_h << 8) | readings[i].gx_l))) / qmi_ctx.gyro_sensitivity;
        gy_total += ((float)((int16_t)((readings[i].gy_h << 8) | readings[i].gy_l))) / qmi_ctx.gyro_sensitivity;
        gz_total += ((float)((int16_t)((readings[i].gz_h << 8) | readings[i].gz_l))) / qmi_ctx.gyro_sensitivity;
    }

    gx_cal = gx_total / readings_available;
    gy_cal = gy_total / readings_available;
    gz_cal = gz_total / readings_available;
}

void qmi_deinit_fifo(void)
{
    vQueueDelete(qmi_int_queue);
    ESP_ERROR_CHECK(gpio_isr_handler_remove(QMI_INTERRUPT1_PIN));
    if (fifo_buffer)
    {
        free(fifo_buffer);
    }
}


/* ----- Public General Purpose Reads ----- */


esp_err_t qmi_get_temp(float * temperature)
{
    esp_err_t err = 0;

    uint8_t temp_l, temp_h;
    err |= read_register(QMI8658_TEMP_L, &temp_l);
    err |= read_register(QMI8658_TEMP_H, &temp_h);

    int16_t temp = (int16_t)((temp_h << 8) | temp_l);
    *temperature = (float)temp / TEMPERATURE_SENSOR_RESOLUTION;

    return err;
}

esp_err_t qmi_get_accel(float * x, float * y, float * z)
{
    esp_err_t err = 0;

    uint8_t acc_x_l, acc_x_h, acc_y_l, acc_y_h, acc_z_l, acc_z_h;
    err |= read_register(QMI8658_ACC_X_L, &acc_x_l);
    err |= read_register(QMI8658_ACC_X_H, &acc_x_h);
    err |= read_register(QMI8658_ACC_Y_L, &acc_y_l);
    err |= read_register(QMI8658_ACC_Y_H, &acc_y_h);
    err |= read_register(QMI8658_ACC_Z_L, &acc_z_l);
    err |= read_register(QMI8658_ACC_Z_H, &acc_z_h);

    int16_t acc_x = (int16_t)((acc_x_h << 8) | acc_x_l);
    int16_t acc_y = (int16_t)((acc_y_h << 8) | acc_y_l);
    int16_t acc_z = (int16_t)((acc_z_h << 8) | acc_z_l);

    *x = (float)acc_x / qmi_ctx.acc_sensitivity;
    *y = (float)acc_y / qmi_ctx.acc_sensitivity;
    *z = (float)acc_z / qmi_ctx.acc_sensitivity;

    return err;
}

esp_err_t qmi_get_gyro(float * x, float * y, float * z)
{
    esp_err_t err = 0;

    uint8_t gyr_x_l, gyr_x_h, gyr_y_l, gyr_y_h, gyr_z_l, gyr_z_h;
    err |= read_register(QMI8658_GYR_X_L, &gyr_x_l);
    err |= read_register(QMI8658_GYR_X_H, &gyr_x_h);
    err |= read_register(QMI8658_GYR_Y_L, &gyr_y_l);
    err |= read_register(QMI8658_GYR_Y_H, &gyr_y_h);
    err |= read_register(QMI8658_GYR_Z_L, &gyr_z_l);
    err |= read_register(QMI8658_GYR_Z_H, &gyr_z_h);

    int16_t gyr_x = (int16_t)((gyr_x_h << 8) | gyr_x_l) - gx_cal;
    int16_t gyr_y = (int16_t)((gyr_y_h << 8) | gyr_y_l) - gy_cal;
    int16_t gyr_z = (int16_t)((gyr_z_h << 8) | gyr_z_l) - gz_cal;

    *x = (float)gyr_x / qmi_ctx.gyro_sensitivity;
    *y = (float)gyr_y / qmi_ctx.gyro_sensitivity;
    *z = (float)gyr_z / qmi_ctx.gyro_sensitivity;

    return err;
}

esp_err_t qmi_read_all_data(qmi8658c_data_t * data)
{
    esp_err_t err = 0;

    err |= qmi_get_accel(&data->acc.x, &data->acc.y, &data->acc.z);
    err |= qmi_get_gyro(&data->gyro.x, &data->gyro.y, &data->gyro.z);
    err |= qmi_get_temp(&data->temperature);

    return err;
}
