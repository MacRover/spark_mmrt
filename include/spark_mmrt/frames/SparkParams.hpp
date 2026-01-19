#ifndef SPARK_MMRT_FRAMES_SPARKPARAMS_HPP
#define SPARK_MMRT_FRAMES_SPARKPARAMS_HPP

#include "spark_mmrt/can/CanFrame.hpp"
#include "spark_mmrt/frames/StatusFrames.hpp"
#include <array>
#include <cstdint>
#include <cstring>

namespace param{
    enum ParamID {
        PARAM_CANID = 0,
        PARAM_InputMode = 1,
        PARAM_MotorType = 2,
        PARAM_CommAdvance = 3,
        PARAM_ControlType = 5,
        PARAM_IdleMode = 6, 
    };

    struct Params{
        uint32_t CANID; 
        uint32_t InputMode;
        uint32_t MotorType;
        uint32_t CommAdvance;
        uint32_t ControlType;
        uint32_t IdleMode;
    };


}

enum InputMode : uint32_t { 
    PWM, 
    CAN, 
    USB 
};

enum MotorType : uint32_t{
    BRUSHED,
    BRUSHLESS
};
enum IdleMode : uint32_t{
    COAST,
    BRAKE
};

enum ControlType : uint32_t {
    DUTY_CYCLE = 0, 
    VELOCITY = 1, 
    VOLTAGE = 2, 
    POSITION = 3,
    SMARTMOTION = 4, 
    SMARTVELOCITY = 5, 
    MAXMOTION_POSITION = 6, 
    MAXMOTION_VELOCITY = 7
};



#endif
