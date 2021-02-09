
include(${COMPONENT_DIR}/cmake/api_path.cmake)

set(sources
    ${VL53L0X_API_SRCS}
    "platform/esp32/src/vl53l0x_i2c_platform.c"
    "platform/esp32/src/vl53l0x_platform_log.c"
    "platform/esp32/src/vl53l0x_platform.c"
    "src/vl53l0x.c"
)

set(includes
    ${VL53L0X_API_INCLUDES}
    "platform/esp32/inc"
    "include/"
)

idf_component_register(SRCS ${sources}
                    INCLUDE_DIRS ${includes})

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-maybe-uninitialized")

