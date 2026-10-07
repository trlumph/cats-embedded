// Host test stub for CMSIS-DSP
#pragma once
#include <cstdint>
using float32_t = float;
struct arm_matrix_instance_f32 {
  uint16_t numRows;
  uint16_t numCols;
  float32_t* pData;
};
