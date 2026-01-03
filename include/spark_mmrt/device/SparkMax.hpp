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
        Status2 s2;
        Status3 s3;
        Status4 s4;
        Status5 s5;
        Status6 s6;
        Status7 s7;
        Status8 s8;
        Status9 s9; 




    public:
        SparkMax(spark_mmrt::can::SocketCanTransport& transport_, uint8_t ID_);
        ~SparkMax() {}

       
        Status0 getStatus0() const;
        Status1 getStatus1() const; 
        Status2 getStatus2() const;
        Status3 getStatus3() const;
        Status4 getStatus4() const;
        Status5 getStatus5() const;
        Status6 getStatus6() const; 
        Status7 getStatus7() const; 
        Status8 getStatus8() const;
        Status9 getStatus9() const;

        void heartbeat();
        void setDutyCycle(float val);
        void setVelocity(float val); 
        void setMMVelocity(float val);
        void setPosition(float val);
        void setMMPosition(float val);
        void setVoltage(float val);
        void setCurrent(float val);
        void setEncoderPosition(float val); 

        void processFrame(const spark_mmrt::can::CanFrame & f); 


        uint8_t getID() const; 

};

#endif
