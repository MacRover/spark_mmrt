#ifndef SPARK_MMRT_FRAMES_STATUSFRAMES_HPP
#define SPARK_MMRT_FRAMES_STATUSFRAMES_HPP

#include <cstdint>

struct Status0 {
    float appliedOutput = 0.0f;   // [-1, 1]
    float voltage = 0.0f;        
    float current = 0.0f;         
    uint8_t motorTempC = 0;      

    // Flags
    bool hardForwardLimit = false;
    bool hardReverseLimit = false;
    bool softForwardLimit = false;
    bool softReverseLimit = false;
    bool inverted = false;
    bool primaryHeartbeatLock = false;
};

namespace Status0Scale {
  constexpr float appliedOutputScale = 0.00003082369457075716f;
  constexpr float voltageScale = 0.0073260073260073f;
  constexpr float currentScale = 0.0366300366300366f;

}

struct Status1
{
    bool hi = false; 
};






#endif