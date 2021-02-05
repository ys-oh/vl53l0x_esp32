#! /bin/bash

API_PATH="$1"
camke_template="

file(GLOB VL53L0X_API_SRCS
    \"${API_PATH}/core/src/vl53l0x_api_core.c\"
    \"${API_PATH}/core/src/vl53l0x_api_calibration.c\"
    \"${API_PATH}/core/src/vl53l0x_api_ranging.c\"
    \"${API_PATH}/core/src/vl53l0x_api_strings.c\"
    \"${API_PATH}/core/src/vl53l0x_api.c\"
)

set(VL53L0X_API_INCLUDES
    \"${API_PATH}/core/inc\"
    \"${API_PATH}/platform/inc\"
)
"

rm -f ./cmake/api_path.cmake

echo "${camke_template}" >> ./cmake/api_path.cmake
