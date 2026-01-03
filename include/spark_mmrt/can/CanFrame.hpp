#ifndef CANFRAME_HPP
#define CANFRAME_HPP
#include <array>
#include <cstdint>
#include <chrono>

namespace spark_mmrt::can {

constexpr uint32_t kExtIdMask = 0x1FFFFFFF; // 29 ones to ensure 29 bits only

struct CanFrame {
  uint32_t arbId = 0;               // 29-bit extended arbitration id 
  bool isRTR = false;               // remote transmission request
  uint8_t dlc = 0;                   // data length code (payload bytes)
  std::array<uint8_t, 8> data{};    // payload (ignored if isRTR)
  std::chrono::steady_clock::time_point timestamp{};

  // constructors 
  // Data frame 
  static CanFrame Data(uint32_t arbId, uint8_t len) {
    CanFrame f;
    f.arbId = arbId & kExtIdMask; // ensure 29 bit 
    f.isRTR = false;
    f.dlc = len;
    return f;
  }
  // Remote request frame 
  static CanFrame RTR(uint32_t arbId) {
    CanFrame f;
    f.arbId = arbId & kExtIdMask; // ensure 29 bit 
    f.isRTR = true;
    f.dlc = 0;
    return f;
  }

  bool valid() const {
    if ((arbId & ~kExtIdMask) != 0) return false; // must be 29-bit
    if (dlc > 8) return false;
    if (isRTR && dlc != 0) return false;
    return true;
  }
};

} // namespace spark_mmrt::can
#endif
