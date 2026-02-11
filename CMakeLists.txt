
idf_component_register(
    SRCS "qmi8658.c"
    REQUIRES driver
    PRIV_REQUIRES esp_timer i2c-lock
    INCLUDE_DIRS "include"
)