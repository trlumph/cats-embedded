// Host test stub for flight_computer/src/target/VEGA/target.hpp
#pragma once
#include <cstdint>
#include <cstring>  // the STM32 HAL headers provide string.h to the firmware
#include "arm_math.h"
inline constexpr uint8_t NUM_IMU = 1;
inline constexpr uint8_t NUM_BARO = 1;
inline constexpr uint8_t NUM_ACCELEROMETER = 1;
inline constexpr uint8_t NUM_MAGNETO = 0;
