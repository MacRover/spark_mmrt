#ifndef SPARK_MMRT_FRAMES_SPARKFRAMES_HPP
#define SPARK_MMRT_FRAMES_SPARKFRAMES_HPP

#include "spark_mmrt/can/CanFrame.hpp"
#include "spark_mmrt/frames/SparkFrames.hpp"
#include "spark_mmrt/frames/StatusFrames.hpp"
#include <array>
#include <cstdint>
#include <cstring>


constexpr uint8_t DEVICE_TYPE = 0x02; // Device type for SPARK controllers
constexpr uint8_t MANUFACTURER=  0x05; // Manufacturer ID for REV Robotics

//Status Frames ArbID 
constexpr uint32_t STATUS0_BASE = 0x0205B800; 
constexpr uint32_t STATUS1_BASE = 0x0205B840;
constexpr uint32_t STATUS2_BASE = 0x0205B880;
constexpr uint32_t STATUS3_BASE = 0x0205B8C0; 
constexpr uint32_t STATUS4_BASE = 0x0205B900;
constexpr uint32_t STATUS5_BASE = 0x0205B940;
constexpr uint32_t STATUS6_BASE = 0x0205B980; 
constexpr uint32_t STATUS7_BASE = 0x0205B9C0; 
constexpr uint32_t STATUS8_BASE = 0x0205BA00;
constexpr uint32_t STATUS9_BASE = 0x0205BA40;

// apiClass , apiIndex from spark_mmrt/docs/spark-frames-2.0.0-dev.11
struct Api { uint8_t cls; uint8_t idx; };

// more to be added when more functions are implemented from doc 
namespace api {
  constexpr Api Heartbeat{11, 2};
  constexpr Api DutyCycle{0, 2};
  constexpr Api VelocitySetpoint{0,0};
  constexpr Api MMVelocitySetpoint{0,9};
  constexpr Api PositionSetpoint{0,4};
  constexpr Api MMPositionSetpoint{0,8};
  constexpr Api voltageSetpoint{0,5};
  constexpr Api currentSetpoint{0,6};
  constexpr Api setEncoderPosition{10,0}; 

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


spark_mmrt::can::CanFrame setVelocityFrame(float setPoint, uint8_t deviceID); 
spark_mmrt::can::CanFrame setMMVelocityFrame(float setPoint, uint8_t deviceID);
spark_mmrt::can::CanFrame setPositionFrame(float setPoint, uint8_t deviceID);
spark_mmrt::can::CanFrame setMMPositionFrame(float setPoint, uint8_t deviceID);
spark_mmrt::can::CanFrame SetVoltageFrame(float setPoint, uint8_t deviceID);
spark_mmrt::can::CanFrame setCurrentFrame(float setPoint, uint8_t deviceID);
spark_mmrt::can::CanFrame setEncoderPositionFrame(float position, uint8_t deviceID); 



void status0Decoder(const std::array<uint8_t, 8> &data, Status0& s0);
void status1Decoder(const std::array<uint8_t, 8> &data, Status1& s1);
void status2Decoder(const std::array<uint8_t, 8> &data, Status2& s2);
void status3Decoder(const std::array<uint8_t, 8> &data, Status3& s3);
void status4Decoder(const std::array<uint8_t, 8> &data, Status4& s4);
void status5Decoder(const std::array<uint8_t, 8> &data, Status5& s5);
void status6Decoder(const std::array<uint8_t, 8> &data, Status6& s6);
void status7Decoder(const std::array<uint8_t, 8> &data, Status7& s7);
void status8Decoder(const std::array<uint8_t, 8> &data, Status8& s8);
void status9Decoder(const std::array<uint8_t, 8> &data, Status9& s9);




#endif
