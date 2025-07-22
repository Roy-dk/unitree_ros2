#pragma once

namespace unitree::common {

double clamp(double value, double low, double high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

}  // namespace unitree::common
