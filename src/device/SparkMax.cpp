#include "spark_mmrt/device/SparkMax.hpp"
#include "spark_mmrt/frames/SparkFrames.hpp"
#include "spark_mmrt/can/CanFrame.hpp"
#include "spark_mmrt/frames/StatusFrames.hpp"

#include <cmath>
#include <stdexcept>

SparkMax::SparkMax(spark_mmrt::can::SocketCanTransport& transport_, uint8_t ID_)
  : ID(ID_), transport(transport_){}


void SparkMax::Heartbeat() {
  transport.send(heartbeatFrame());
}

void SparkMax::SetDutyCycle(float val) {
  if (!std::isfinite(val)) throw std::invalid_argument("duty must be finite");

  transport.send(setDutyCycleFrame(val, ID));
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

    default:
      break; 
  }
}

float SparkMax::getAppliedOutput() {return s0.appliedOutput;}
float SparkMax::getVoltage() {return s0.voltage;}
float SparkMax::getCurrent() { return s0.current;} 
float SparkMax::getTemp() {return s0.motorTempC;}


