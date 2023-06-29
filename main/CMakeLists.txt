idf_component_register(
    SRCS "main.c"
    SRCS "ble.c"
    SRCS "config.c"
    SRCS "wifi.c"
    SRCS "motor.c"
    SRCS "mqtt.c"
    SRCS "sensor.c"
    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
