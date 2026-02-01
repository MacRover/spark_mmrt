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
        PARAM_SensorType = 9,
        PARAM_P0 = 13,
        PARAM_I0 = 14,
        PARAM_D0 = 15, 
        PARAM_F0 = 16,
        PARAM_IZ0 = 17,
        PARAM_DFilter0 = 18,
        PARAM_OutputMin0 = 19,
        PARAM_OutputMax0 = 20,
        PARAM_StatusPeriod0 = 158,
        PARAM_StatusPeriod1 = 159,
        PARAM_StatusPeriod2 = 160,
        PARAM_StatusPeriod3 = 161,
        PARAM_StatusPeriod4 = 162,
        PARAM_StatusPeriod5 = 163,
        PARAM_StatusPeriod6 = 164,
        PARAM_StatusPeriod7 = 165,
        PARAM_MaxVelMM0 = 166,
        PARAM_MaxAccelMM0 = 167, 
        PARAM_AllowedClosedLoopErrorMM0 = 169,
        PARAM_ForceEnableStatus0 = 186,
        PARAM_ForceEnableStatus1 = 187,
        PARAM_ForceEnableStatus2 = 188,
        PARAM_ForceEnableStatus3 = 189,
        PARAM_ForceEnableStatus4 = 190,
        PARAM_ForceEnableStatus5 = 191,
        PARAM_ForceEnableStatus6 = 192,
        PARAM_ForceEnableStatus7 = 193
    };

    struct Params{
        uint32_t CANID; 
        uint32_t InputMode;
        uint32_t MotorType;
        uint32_t CommAdvance;
        uint32_t ControlType;
        uint32_t IdleMode;
        uint32_t SensorType;
        float P;
        float I;
        float D;
        float F;
        float IZ;
        float DFilter;
        float OutputMin;
        float OutputMax; 
        uint32_t period0;
        uint32_t period1;
        uint32_t period2;
        uint32_t period3;
        uint32_t period4;
        uint32_t period5;
        uint32_t period6;
        uint32_t period7;
        float MaxVelMM;
        float MaxAccelMM;
        float AllowedClosedLoopErrorMM; 
        bool ForceEnableStatus0;
        bool ForceEnableStatus1;
        bool ForceEnableStatus2;
        bool ForceEnableStatus3;
        bool ForceEnableStatus4;
        bool ForceEnableStatus5;
        bool ForceEnableStatus6;
        bool ForceEnableStatus7;

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

enum SensorType : uint32_t {
    NONE, 
    MAIN_ENCODER,
    ANALOG_ENCODER,
    ALT_ENCODER,
    DUTY_CYCLE_ENCODER
};

enum MAXMotionPositionMode : uint32_t{
    TRAPEZOIDAL
};





#endif
