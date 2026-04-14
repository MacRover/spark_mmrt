#ifndef SPARK_MMRT_DEVICE_ROBORIO_HPP
#define SPARK_MMRT_DEVICE_ROBORIO_HPP
#include "spark_mmrt/can/SocketCanTransport.hpp"
#include "spark_mmrt/frames/SparkFrames.hpp"

const uint8_t RIO_DEVICE_TYPE = 0x01;
const uint8_t RIO_MANUFACTURER = 0x01;

class RoboRIO {

public:
    RoboRIO(spark_mmrt::can::SocketCanTransport& transport_);
    ~RoboRIO();
    void heartbeat(void);
private:
    spark_mmrt::can::CanFrame heartbeatFrame();
    spark_mmrt::can::SocketCanTransport& transport_;
};

#endif