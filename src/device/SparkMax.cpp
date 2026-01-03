#include "spark_mmrt/device/SparkMax.hpp"
#include "spark_mmrt/frames/SparkFrames.hpp"
#include "spark_mmrt/can/CanFrame.hpp"
#include "spark_mmrt/frames/StatusFrames.hpp"

#include <cmath>
#include <stdexcept>

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

  transport.send(SetVoltageFrame(val, ID)); 
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


