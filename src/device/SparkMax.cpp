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

std::optional<ParamWriteResponse> SparkMax::writeParam(param::ParamID paramID, uint32_t value, std::chrono::milliseconds timeout)
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

std::optional<ParamWriteResponse> SparkMax::writeParam(param::ParamID paramID, float value, std::chrono::milliseconds timeout){
  uint32_t x; 
  std::memcpy(&x, &value, sizeof(x));
  return writeParam(paramID, x, timeout);
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
 
bool SparkMax::readParam(param::ParamID paramID, std::chrono::milliseconds timeout){
  return readParamWithType(paramID, timeout).has_value(); // if readParamwith type returned a paramRead Response then return
}

std::optional<ParamReadResponse>
SparkMax::readParamWithType(param::ParamID paramID, std::chrono::milliseconds timeout){
  //  send a 0 DLC data frame to the parameter's arbId,
  // then decode the response where data[4] is parameter type
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
static float uInt32toFloat(uint32_t r){
  float f;
  std::memcpy(&f, &r, sizeof(f));
  return f;
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
    case param::PARAM_SensorType: 
      p.SensorType = value; 
      break; 
    case param::PARAM_P0:
      p.P = uInt32toFloat(value); 
      break;
    case param::PARAM_I0:
      p.I = uInt32toFloat(value); 
      break; 
    case param::PARAM_D0:
      p.D = uInt32toFloat(value); 
      break;
    case param::PARAM_F0:
      p.F = uInt32toFloat(value);
      break;
    case param::PARAM_IZ0:
      p.IZ = uInt32toFloat(value);
      break;
    case param::PARAM_DFilter0:
      p.DFilter = uInt32toFloat(value);
      break;
    case param::PARAM_OutputMin0:
      p.OutputMin = uInt32toFloat(value);
      break;
    case param::PARAM_OutputMax0:
      p.OutputMax = uInt32toFloat(value);
      break;
    case param::PARAM_StatusPeriod0:
      p.period0 = value; 
      break;  
    case param::PARAM_StatusPeriod1:
      p.period1 = value;
      break;
    case param::PARAM_StatusPeriod2:
      p.period2 = value;
      break;
    case param::PARAM_StatusPeriod3:
      p.period3 = value;
      break;
    case param::PARAM_StatusPeriod4:
      p.period4 = value;
      break;
    case param::PARAM_StatusPeriod5:
      p.period5 = value;
      break;
    case param::PARAM_StatusPeriod6:
      p.period6 = value;
      break;
    case param::PARAM_StatusPeriod7:
      p.period7 = value;
      break;
    case param::PARAM_MaxVelMM0:
      p.MaxVelMM = uInt32toFloat(value);
      break;
    case param::PARAM_MaxAccelMM0:
      p.MaxAccelMM = uInt32toFloat(value);
      break;
    case param::PARAM_AllowedClosedLoopErrorMM0:
      p.AllowedClosedLoopErrorMM = uInt32toFloat(value);
      break;
    case param::PARAM_ForceEnableStatus0:
      p.ForceEnableStatus0 = (value != 0u);
      break;
    case param::PARAM_ForceEnableStatus1:
      p.ForceEnableStatus1 = (value != 0u);
      break;
    case param::PARAM_ForceEnableStatus2:
      p.ForceEnableStatus2 = (value != 0u);
      break;
    case param::PARAM_ForceEnableStatus3:
      p.ForceEnableStatus3 = (value != 0u);
      break;
    case param::PARAM_ForceEnableStatus4:
      p.ForceEnableStatus4 = (value != 0u);
      break;
    case param::PARAM_ForceEnableStatus5:
      p.ForceEnableStatus5 = (value != 0u);
      break;
    case param::PARAM_ForceEnableStatus6:
      p.ForceEnableStatus6 = (value != 0u);
      break;
    case param::PARAM_ForceEnableStatus7:
      p.ForceEnableStatus7 = (value != 0u);
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
std::optional<ParamWriteResponse> SparkMax::setSensorType(SensorType type, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_SensorType, uint32_t(type), timeout); 
}

std::optional<ParamWriteResponse> SparkMax::setP(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_P0, val, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setI(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_I0, val, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setD(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_D0, val, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setF(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_F0, val, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setIZ(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_IZ0, val, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setDFilter(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_DFilter0, val, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setOutputMin(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_OutputMin0, val, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setOutputMax(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_OutputMax0, val, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setCANID(uint32_t ID,  std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_CANID, ID, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setStatus0(uint32_t val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_StatusPeriod0, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setStatus1(uint32_t val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_StatusPeriod1, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setStatus2(uint32_t val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_StatusPeriod2, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setStatus3(uint32_t val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_StatusPeriod3, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setStatus4(uint32_t val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_StatusPeriod4, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setStatus5(uint32_t val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_StatusPeriod5, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setStatus6(uint32_t val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_StatusPeriod6, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setStatus7(uint32_t val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_StatusPeriod7, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setMaxVelMM(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_MaxVelMM0, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setMaxAccelMM(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_MaxAccelMM0, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setAllowedClosedLoopErrorMM(float val, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_AllowedClosedLoopErrorMM0, val, timeout); 
}
std::optional<ParamWriteResponse> SparkMax::setForceEnableStatus0(bool enabled, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_ForceEnableStatus0, enabled ? 1u : 0u, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setForceEnableStatus1(bool enabled, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_ForceEnableStatus1, enabled ? 1u : 0u, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setForceEnableStatus2(bool enabled, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_ForceEnableStatus2, enabled ? 1u : 0u, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setForceEnableStatus3(bool enabled, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_ForceEnableStatus3, enabled ? 1u : 0u, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setForceEnableStatus4(bool enabled, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_ForceEnableStatus4, enabled ? 1u : 0u, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setForceEnableStatus5(bool enabled, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_ForceEnableStatus5, enabled ? 1u : 0u, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setForceEnableStatus6(bool enabled, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_ForceEnableStatus6, enabled ? 1u : 0u, timeout);
}
std::optional<ParamWriteResponse> SparkMax::setForceEnableStatus7(bool enabled, std::chrono::milliseconds timeout){
  return writeParam(param::PARAM_ForceEnableStatus7, enabled ? 1u : 0u, timeout);
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

SensorType SparkMax::getSensorType() const{
  return SensorType(p.SensorType);
}




