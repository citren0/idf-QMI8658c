# QMI8658 Driver for ESP IDF

An ESP IDF component for the QMI8658 Rev A, supporting standard reads, complimentary filtering, and FIFO buffering.

## Note on compatibility

This driver will only work with the "Rev A" version of the qmi. Looking in the datasheet for rev 9, it's clear many of the register bit fields changed and certain features were disabled/enabled. This could still be a guideline on how to configure other revisions of this chip, but no guarantees are made that any of the FIFO tech will be the same.

## Planned Features

- Standard initialization function without FIFO buffering.

## Usage

This driver is intended to be used with the FIFO buffer of the QMI so that the host can spend less time waiting for I2C responses and batch process the IMU data.

To fetch this repo, add it as a submodule in your idf project's components folder or use the component manager.

Inform the QMI component of your I2C bus configuration with the following function:

    void init_qmi(i2c_master_bus_handle_t bus_handle);

Note: The QMI8658 expects pullups on scl and sda and an I2C frequency less than or equal to 400kHz.

Then, initialize a complimentary filter structure:

    qmi8658c_complimentary_t complimentary;
    qmi_init_complimentary(&complimentary);

To complete initialization, initialize your QMI with the config object and setup function:

    qmi8658c_config_t config =
    {
        .mode = QMI8658C_MODE_DUAL, // Enable both Accelerometer and Gyrometer.
        .acc_scale = QMI8658C_ACC_SCALE_4G, // 4Gs maximum acceleration.
        .acc_odr = QMI8658C_ACC_ODR_500, // 500 Hz output data rate.
        .gyro_scale = QMI8658C_GYRO_SCALE_256DPS, // 256 deg/s maximum angular velocity.
        .gyro_odr = QMI8658C_GYRO_ODR_500, // 500 Hz output data rate.
    };
    qmi_fifo_setup(&config);

_Recommended_: Run calibration.

    qmi_calibrate_with_fifo();

Now wait for a reading to become available, and consume the data.

    while (qmi_fifo_is_read_ready() == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    qmi8658c_fifo_reading_t * readings = NULL;
    uint16_t readings_available = 0;

    qmi_fifo_consume(&readings, &readings_available);

    qmi_fifo_update_complimentary_with_readings(&complimentary, readings, readings_available);

Note: If FIFO data is not received promptly once it's available, it will be overwritten once the FIFO buffer runs out of free space. It's recommended to calculate how often you need to read the buffer with this formula:

    Max seconds between reads = (1536 Byte FIFO Size / 12 Bytes per reading) / ODR of Acc and Gyro

Once data is read, access it via the qmi8658c_complimentary_t structure:

    float pitch = complimentary.pitch;
    float yaw = complimentary.yaw;
    float roll = complimentary.roll;