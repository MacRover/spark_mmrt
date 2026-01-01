#ifndef SPARK_MMRT_DEVICE_SPARKMAX_HPP
#define SPARK_MMRT_DEVICE_SPARKMAX_HPP
#include "spark_mmrt/can/SocketCanTransport.hpp"
#include "spark_mmrt/frames/StatusFrames.hpp"
#include "spark_mmrt/can/CanFrame.hpp"
#include <cstdint>
#include <unordered_map>



class SparkMax{

    private:
        uint8_t ID;
        spark_mmrt::can::SocketCanTransport& transport;
        Status0 s0; 
        Status1 s1; 



    public:
        SparkMax(spark_mmrt::can::SocketCanTransport& transport_, uint8_t ID_);
        void Heartbeat();
        void SetDutyCycle(float val);
        void processFrame(const spark_mmrt::can::CanFrame & f); 
        float getAppliedOutput(); 
        float getVoltage();
        float getCurrent(); 
        float getTemp(); 

        uint8_t getID() const; 

};

#endif
