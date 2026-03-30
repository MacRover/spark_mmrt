#ifndef SPARK_MMRT_FRAMES_STATUSFRAMES_HPP
#define SPARK_MMRT_FRAMES_STATUSFRAMES_HPP

#include <cstdint>

// ONLY STATUS 0 and 1 ARE ENABLED BY DEFAULT THE REST NEED TO BE ENABLED MANUALLY 

struct Status0 {
    double appliedOutput;   // [-1, 1]
    double voltage;        
    double current;         
    uint8_t motorTempC;      

    // Flags
    bool hardForwardLimit;
    bool hardReverseLimit;
    bool softForwardLimit;
    bool softReverseLimit;
    bool inverted;
    bool primaryHeartbeatLock;

    uint8_t sparkModel; // in new doc idk if we will need 



};

namespace Status0Scale {
  constexpr double  appliedOutputScale = 0.00003082369457075716f;
  constexpr double voltageScale = 0.0073260073260073f;
  constexpr double  currentScale = 0.0366300366300366f;

}


struct Status1 {
  bool otherFault; // 0
  bool motorTypeFault; //1
  bool sensorFault; //  2
  bool canFault; //  3
  bool temperatureFault; //  4
  bool drvFault; //  5
  bool escEepromFault; //  6
  bool firmwareFault; //  7
  uint8_t reservedActives;     //  8->15

  bool brownoutWarning; //  16
  bool overcurrentWarning; //  17
  bool escEepromWarning; //  18
  bool extEepromWarning; //  19
  bool sensorWarning; //  20
  bool stallWarning; //  21
  bool hasResetWarning; //  22
  bool otherWarning; //  23

  bool otherStickyFault; //  24
  bool motorTypeStickyFault; //  25
  bool sensorStickyFault; //  26
  bool canStickyFault; //  27
  bool temperatureStickyFault; //  28
  bool drvStickyFault; //  29
  bool escEepromStickyFault; //  30
  bool firmwareStickyFault; //  31
  uint8_t reservedStickies;     //  32->39

  bool brownoutStickyWarning; //  40
  bool overcurrentStickyWarning; //  41
  bool escEepromStickyWarning; //  42
  bool extEepromStickyWarning; //  43
  bool sensorStickyWarning; //  44
  bool stallStickyWarning; //  45
  bool hasResetStickyWarning; //  46
  bool otherStickyWarning; //  47

  bool isFollower; //  48
  uint16_t reserved;     //  49->63 
};


struct Status2{

  float primaryEncoderVelocity; //"By default, the unit is RPM, but it can be changed implicitly using the Velocity Conversion Factor parameter",
  float primaryEncoderPosition; //"By default, the unit is rotations, but it can be changed implicitly using the Position Conversion Factor parameter",


};

struct Status3{
  double analogVoltage; 
  double analogVelocity; //"By default, the unit is RPM, but it can be changed implicitly using the Analog Velocity Conversion Factor parameter",
  double analogPosition; // "By default, the unit is rotations, but it can be changed implicitly using the Analog Position Conversion Factor parameter",
};

namespace Status3Scale {
  constexpr double analogVoltageScale = 0.0048973607038123f;
  constexpr double analogVelocityScale = 0.007812026887906498f;
}

struct Status4{
  double altEncoderVelocity; 
  double altEncoderPosition;
};

struct Status5{
  double dutyCycleEncVelocity;
  double dutyCycleEncPosition; 
};

struct Status6{
  double unadjustedDutyCycle;
  uint16_t dutyCyclePeriod;
  bool dutyCycleNoSignal;

  uint32_t dutyCycleReserved; 
};
namespace Status6Scale{
  constexpr double unadjustedDutyCycleScale = 0.00001541161211566339;
}

struct Status7{
  double IAccumalation; 
  uint32_t reserved; 
};

// in new doc idk if we will need 
struct Status8{
  double setPoint;
  bool isAtSetpoint;
  uint8_t selectedPIDSlot; 
  uint32_t reserved;  
};

//LIKELY WILL NOT NEED IT SAYS 26.0.0 VERSION IMPLEMENTATION???
struct Status9{
  double MaxMotionPositionSetPoint;
  double MaxMotionVelocitySetPoint;

};



#endif