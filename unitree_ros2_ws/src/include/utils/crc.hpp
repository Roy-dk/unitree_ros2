#pragma once

#include <cstdint>

namespace unitree::common {

uint32_t crc32_core(const uint32_t* ptr, uint32_t len);

}
