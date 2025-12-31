#include "spark_mmrt/device/SparkMax.hpp"
#include "spark_mmrt/frames/SparkFrames.hpp"

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


