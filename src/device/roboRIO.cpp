#include "spark_mmrt/device/roboRIO.hpp"

RoboRIO::RoboRIO(spark_mmrt::can::SocketCanTransport& transport) : transport_(transport) {}

RoboRIO::~RoboRIO() {
    transport_.close();
}

void RoboRIO::heartbeat(void) {
    transport_.send(heartbeatFrame());
}

spark_mmrt::can::CanFrame RoboRIO::heartbeatFrame() {
    const uint32_t arbID = makeArbID(RIO_DEVICE_TYPE, RIO_MANUFACTURER, api::UniveralHeartbeat, 0);
    // Set 8 byte payload with 0xFFs to enable controllers indefinitely until timeout occurs
    auto frame = spark_mmrt::can::CanFrame::Data(arbID, 8);
    frame.data.fill(0xFF);
    return frame;
}