#ifndef SPARK_MMRT_FRAMES_SPARKFRAMES_HPP
#define SPARK_MMRT_FRAMES_SPARKFRAMES_HPP

#include "spark_mmrt/can/CanFrame.hpp"
#include "spark_mmrt/frames/SparkFrames.hpp"
#include <array>
#include <cstdint>
#include <cstring>


constexpr uint8_t DEVICE_TYPE = 0x02; // Device type for SPARK controllers
constexpr uint8_t MANUFACTURER=  0x05; // Manufacturer ID for REV Robotics

// apiClass , apiIndex from spark_mmrt/docs/spark-frames-2.0.0-dev.11
struct Api { uint8_t cls; uint8_t idx; };

// more to be added when more functions are implemented from doc 
namespace api {
  constexpr Api Heartbeat{11, 2};
  constexpr Api DutyCycle{0, 2};
}

// ArbID Layout: 
//| DeviceType  | Manufactuer | apiClass | apiIndex | Device ID (canID) |
constexpr uint32_t makeArbID(uint8_t deviceType, uint8_t manufacturer, Api api, uint8_t deviceID){
    return 
        (uint32_t(deviceType) << 24) | 
        (uint32_t(manufacturer) << 16) |
        (uint32_t(api.cls) << 10) |
        (uint32_t(api.idx) << 6) |
        uint32_t(deviceID  & 0x3F);

}
//Bitmask where a single bit corresponds to the CAN ID.
// Currently using class objects for each CAN ID idk if it hlps 
constexpr uint64_t sparkMaxDeviceIDMask(uint8_t canID) {
  return (canID < 64) ? (1ull << canID) : 0ull;
}

spark_mmrt::can::CanFrame heartbeatFrame();

spark_mmrt::can::CanFrame setDutyCycleFrame(float dutyCycle, uint8_t deviceID);




#endif
