#ifndef SPARK_MMRT_FRAMES_STATUSFRAMES_HPP
#define SPARK_MMRT_FRAMES_STATUSFRAMES_HPP

#include <cstdint>

// ONLY STATUS 0 and 1 ARE ENABLED BY DEFAULT THE REST NEED TO BE ENABLED MANUALLY 

struct Status0 {
    double appliedOutput = 0.0;   // [-1, 1]
    double voltage = 0.0;        
    double current = 0.0;         
    uint8_t motorTempC = 0;      

    // Flags
    bool hardForwardLimit = false;
    bool hardReverseLimit = false;
    bool softForwardLimit = false;
    bool softReverseLimit = false;
    bool inverted = false;
    bool primaryHeartbeatLock = false;

    uint8_t sparkModel = 0; // in new doc idk if we will need 



};

namespace Status0Scale {
  constexpr double  appliedOutputScale = 0.00003082369457075716f;
  constexpr double voltageScale = 0.0073260073260073f;
  constexpr double  currentScale = 0.0366300366300366f;

}


struct Status1 {
  bool otherFault = false; // 0
  bool motorTypeFault = false; //1
  bool sensorFault = false; //  2
  bool canFault = false; //  3
  bool temperatureFault = false; //  4
  bool drvFault = false; //  5
  bool escEepromFault = false; //  6
  bool firmwareFault = false; //  7
  uint8_t reservedActives = 0;     //  8->15

  bool brownoutWarning = false; //  16
  bool overcurrentWarning = false; //  17
  bool escEepromWarning = false; //  18
  bool extEepromWarning = false; //  19
  bool sensorWarning = false; //  20
  bool stallWarning = false; //  21
  bool hasResetWarning = false; //  22
  bool otherWarning = false; //  23

  bool otherStickyFault = false; //  24
  bool motorTypeStickyFault = false; //  25
  bool sensorStickyFault = false; //  26
  bool canStickyFault = false; //  27
  bool temperatureStickyFault = false; //  28
  bool drvStickyFault = false; //  29
  bool escEepromStickyFault = false; //  30
  bool firmwareStickyFault = false; //  31
  uint8_t reservedStickies = 0;     //  32->39

  bool brownoutStickyWarning = false; //  40
  bool overcurrentStickyWarning = false; //  41
  bool escEepromStickyWarning = false; //  42
  bool extEepromStickyWarning = false; //  43
  bool sensorStickyWarning = false; //  44
  bool stallStickyWarning = false; //  45
  bool hasResetStickyWarning = false; //  46
  bool otherStickyWarning = false; //  47

  bool isFollower = false; //  48
  uint16_t reserved = 0;     //  49->63 
};


struct Status2{

  float primaryEncoderVelocity = 0.0f; //"By default, the unit is RPM, but it can be changed implicitly using the Velocity Conversion Factor parameter",
  float primaryEncoderPosition = 0.0f; //"By default, the unit is rotations, but it can be changed implicitly using the Position Conversion Factor parameter",


};

struct Status3{
  double analogVoltage = 0.0; 
  double analogVelocity = 0.0; //"By default, the unit is RPM, but it can be changed implicitly using the Analog Velocity Conversion Factor parameter",
  double analogPosition = 0.0; // "By default, the unit is rotations, but it can be changed implicitly using the Analog Position Conversion Factor parameter",




};
namespace Status3Scale {
  constexpr double analogVoltageScale = 0.0048973607038123f;
  constexpr double analogVelocityScale = 0.007812026887906498f;
}

struct Status4{
  double altEncoderVelocity = 0.0; 
  double altEncoderPosition = 0.0;
};

struct Status5{
  double dutyCycleEncVelocity = 0.0;
  double dutyCycleEncVPosition = 0.0; 
};

struct Status6{
  double unadjustedDutyCycle = 0.0;
  uint16_t dutyCyclePeriod = 0;
  bool dutyCycleNoSignal = false;

  uint32_t dutyCycleReserved = 0; 
};
namespace Status6Scale{
  constexpr double unadjustedDutyCycleScale = 0.00001541161211566339;
}

struct Status7{
  double IAccumalation = 0.0; 
  uint32_t reserved = 0; 
};

// in new doc idk if we will need 
struct Status8{
  double setPoint = 0.0;
  bool isAtSetpoint = false;
  uint8_t selectedPIDSlot = 0; 
  uint32_t reserved = 0;  
};

//LIKELY WILL NOT NEED IT SAYS 26.0.0 VERSION IMPLEMENTATION???
struct Status9{
  double MaxMotionPositionSetPoint = 0.0;
  double MaxMotionVelocitySetPoint = 0.0;

};



#endif