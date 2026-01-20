#include "spark_mmrt/device/SparkMax.hpp"
#include "spark_mmrt/frames/SparkFrames.hpp"
#include "spark_mmrt/can/CanFrame.hpp"
#include "spark_mmrt/frames/StatusFrames.hpp"

#include <cmath>
#include <stdexcept>
#include <cstring>

SparkMax::SparkMax(spark_mmrt::can::SocketCanTransport& transport_, uint8_t ID_)
  : ID(ID_), transport(transport_){}


void SparkMax::heartbeat() {
  transport.send(heartbeatFrame());
}

void SparkMax::setDutyCycle(float val) {
  if (!std::isfinite(val)) throw std::invalid_argument("Invalid Duty Cycle");

  transport.send(setDutyCycleFrame(val, ID));
}
void SparkMax::setVelocity(float val){
  if (!std::isfinite(val)) throw std::invalid_argument("invalid Velocity Setpoint ");

  transport.send(setVelocityFrame(val, ID)); 

}
void SparkMax::setMMVelocity(float val){
  if (!std::isfinite(val)) throw std::invalid_argument("invalid MM Velocity Setpoint ");

  transport.send(setMMVelocityFrame(val, ID)); 
}
void SparkMax::setPosition(float val){
  if (!std::isfinite(val)) throw std::invalid_argument("invalid Position Setpoint ");

  transport.send(setPositionFrame(val, ID)); 
}
void SparkMax::setMMPosition(float val){
  if (!std::isfinite(val)) throw std::invalid_argument("invalid MM Position Setpoint ");

  transport.send(setMMPositionFrame(val, ID)); 
}
void SparkMax::setVoltage(float val){
  if (!std::isfinite(val)) throw std::invalid_argument("invalid Voltage Setpoint ");

  transport.send(setVoltageFrame(val, ID)); 
}
void SparkMax::setCurrent(float val){
  if (!std::isfinite(val)) throw std::invalid_argument("invalid Current  Setpoint ");

  transport.send(setCurrentFrame(val, ID)); 
}
void SparkMax::setEncoderPosition(float val){
  if (!std::isfinite(val)) throw std::invalid_argument("invalid encoder Position (in rotations) ");

  transport.send(setEncoderPositionFrame(val, ID)); 
}

uint8_t SparkMax::getID() const {
  return ID; 
}

std::optional<ParamWriteResponse>
SparkMax::writeParam(param::ParamID paramID, uint32_t value, std::chrono::milliseconds timeout)
{
  // 1) Send write
  transport.send(paramWriteFrame(value, uint8_t(paramID), ID));

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {     
    auto f = transport.recv(std::chrono::microseconds{200000});
    if (!f) continue;

    const auto& frame = *f;
    if (uint8_t(frame.arbId & 0x3F) != ID) continue;

    uint32_t base = frame.arbId & ~0x3Fu;
    if (base != PARAM_WRITE_RSP_BASE) continue; 
    if (frame.dlc < 7) continue;

    ParamWriteResponse rsp = paramWriteResponseDecode(frame.data);
    if (rsp.param_id != uint8_t(paramID)) continue;


    if (rsp.result_code == 0) { // write was successful 
      assignParam(p, paramID, rsp.value);
    }
    if (paramID == param::PARAM_CANID){
      ID = rsp.value; 
    }

    return rsp;
  }

  return std::nullopt;
}


void SparkMax::processFrame(const spark_mmrt::can::CanFrame& f) {
  uint32_t base = f.arbId & ~0x3Fu;  // mask out device bits 

  switch (base) {
    case STATUS0_BASE:  
      status0Decoder(f.data, s0);
      break;
    case STATUS1_BASE:
      status1Decoder(f.data, s1);
      break;
    case STATUS2_BASE:
      status2Decoder(f.data, s2);
      break; 
    case STATUS3_BASE:
      status3Decoder(f.data, s3); 
      break;
    case STATUS4_BASE:
      status4Decoder(f.data, s4); 
      break;
    case STATUS5_BASE:
      status5Decoder(f.data, s5); 
      break;
    case STATUS6_BASE:
      status6Decoder(f.data, s6); 
      break;
    case STATUS7_BASE:
      status7Decoder(f.data, s7); 
      break;
    case STATUS8_BASE:
      status8Decoder(f.data, s8); 
      break;
    case STATUS9_BASE:
      status9Decoder(f.data, s9); 
      break;
    default:
      break; 
  }
}

bool SparkMax::Flash(std::chrono::microseconds timeout){
  transport.send(persistParamFrame(ID));

  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    auto f = transport.recv(std::chrono::microseconds{20000});
    if (!f) continue;

    const auto& frame = *f;
    if (uint8_t(frame.arbId & 0x3F) != ID) continue;

    uint32_t base = frame.arbId & ~0x3Fu;
    if (base != PERSIST_PARAMS_RESPONSE_BASE) continue; 

    if (frame.dlc < 1) return false;
    return frame.data[0] == 0;
  }
  return false; // timed out
} 
 
uint32_t SparkMax::readParamBaseFor(param::ParamID paramID) {
  uint8_t pairIndex = uint8_t(paramID) / 2;
  Api readApi{15, pairIndex};
  return makeArbID(DEVICE_TYPE, MANUFACTURER, readApi, 0);
}

bool SparkMax::readParam(param::ParamID paramID, std::chrono::milliseconds timeout){
  return readParamWithType(paramID, timeout).has_value();
}

std::optional<ParamReadResponse>
SparkMax::readParamWithType(param::ParamID paramID, std::chrono::milliseconds timeout){
  // SparkBase-style read: send a 0-DLC data frame to the parameter's arbId,
  // then decode the response where data[4] is the parameter type.
  Api readApi{48, uint8_t(paramID)};
  const uint32_t requestArbId = makeArbID(DEVICE_TYPE, MANUFACTURER, readApi, ID);
  transport.send(spark_mmrt::can::CanFrame::Data(requestArbId, 0));

  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    auto f = transport.recv(std::chrono::microseconds{20000});
    if (!f) continue;

    const auto& frame = *f;
    if (uint8_t(frame.arbId & 0x3F) != ID) continue;
    if (frame.arbId != requestArbId) continue;
    if (frame.dlc < 5) continue;

    uint8_t type = frame.data[4];
    uint32_t raw = 0;
    switch (type) {
      case 0x01: { // uint32
        for (int i = 0; i < 4; ++i) {
          raw |= uint32_t(frame.data[i]) << (8 * i);
        }
        break;
      }
      case 0x02: { // float32
        std::memcpy(&raw, frame.data.data(), sizeof(raw));
        break;
      }
      case 0x03: { // bool
        raw = frame.data[0] ? 1u : 0u;
        break;
      }
      default:
        continue;
    }

    assignParam(p, paramID, raw);
    return ParamReadResponse{type, raw};
  }
  return std::nullopt; // timed out
}

bool SparkMax::readParam0And1(std::chrono::milliseconds timeout){
  transport.send(readParam0_1RTRFrame(ID));
  const uint32_t expectedBase = READ_PARAM_0_1_BASE;

  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    auto f = transport.recv(std::chrono::microseconds{20000});
    if (!f) continue;

    const auto& frame = *f;
    if (uint8_t(frame.arbId & 0x3F) != ID) continue;

    uint32_t base = frame.arbId & ~0x3Fu;
    if (base != expectedBase) continue;

    if (frame.dlc < 8) continue;

    uint32_t param0 = decodeParam(frame.data, param::PARAM_CANID);
    uint32_t param1 = decodeParam(frame.data, param::PARAM_InputMode);
    assignParam(p, param::PARAM_CANID, param0);
    assignParam(p, param::PARAM_InputMode, param1);
    return true;
  }
  return false; // timed out
}
void SparkMax::assignParam(param::Params& p, param::ParamID paramID, uint32_t value) {
  switch (paramID) {
    case param::PARAM_CANID:
      p.CANID = value;
      break;
    case param::PARAM_InputMode:
      p.InputMode = value;
      break;
    case param::PARAM_MotorType:
      p.MotorType = value;
      break;
    case param::PARAM_CommAdvance:
      p.CommAdvance = value;
      break;
    case param::PARAM_ControlType:
      p.ControlType = value;
      break;
    case param::PARAM_IdleMode:
      p.IdleMode = value;
      break;
    default:
      break;
  }
}

std::optional<ParamWriteResponse> SparkMax::setIdleMode(IdleMode mode, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_IdleMode, uint32_t(mode), timeout);

}
std::optional<ParamWriteResponse> SparkMax::setControlType(ControlType type, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_ControlType, uint32_t(type), timeout);
}



Status0 SparkMax::getStatus0() const {
  return s0; 
}
Status1 SparkMax::getStatus1() const {
  return s1; 
}
Status2 SparkMax::getStatus2() const {
  return s2; 
}
Status3 SparkMax::getStatus3() const {
  return s3; 
}
Status4 SparkMax::getStatus4() const {
  return s4; 
}
Status5 SparkMax::getStatus5() const {
  return s5; 
}
Status6 SparkMax::getStatus6() const {
  return s6; 
}
Status7 SparkMax::getStatus7() const {
  return s7; 
}
Status8 SparkMax::getStatus8() const {
  return s8; 
}
Status9 SparkMax::getStatus9() const {
  return s9; 
}
param::Params SparkMax::getParams() const{
  return p;  
}

InputMode SparkMax::getInputMode() const{
  return InputMode(p.InputMode); 
}

MotorType SparkMax::getMotorType() const{
  return MotorType(p.MotorType);
}
IdleMode SparkMax::getIdleMode() const{
  return IdleMode(p.IdleMode);
}
ControlType SparkMax::getControlType() const{
  return ControlType(p.ControlType);
}




