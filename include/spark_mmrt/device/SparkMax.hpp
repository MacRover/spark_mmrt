#ifndef SPARK_MMRT_DEVICE_SPARKMAX_HPP
#define SPARK_MMRT_DEVICE_SPARKMAX_HPP
#include "spark_mmrt/can/SocketCanTransport.hpp"
#include <cstdint>



class SparkMax{

    private:
        uint8_t ID;
        spark_mmrt::can::SocketCanTransport& transport;


    public:
        SparkMax(spark_mmrt::can::SocketCanTransport& transport_, uint8_t ID_);
        void Heartbeat();
        void SetDutyCycle(float val);

        uint8_t getID() const; 

};

#endif
