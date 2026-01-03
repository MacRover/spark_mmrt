#include "spark_mmrt/device/SparkMax.hpp"
#include "spark_mmrt/frames/SparkFrames.hpp"
#include <iostream>
#include <chrono>
#include <thread>

static void printStatus1(const Status1& s1) {
  std::cout << "[S1] faults:"
            << " other=" << s1.otherFault
            << " motorType=" << s1.motorTypeFault
            << " sensor=" << s1.sensorFault
            << " can=" << s1.canFault
            << " temp=" << s1.temperatureFault
            << " drv=" << s1.drvFault
            << " escEep=" << s1.escEepromFault
            << " fw=" << s1.firmwareFault
            << " | warnings:"
            << " brownout=" << s1.brownoutWarning
            << " overcurrent=" << s1.overcurrentWarning
            << " stall=" << s1.stallWarning
            << " reset=" << s1.hasResetWarning
            << " follower=" << s1.isFollower
            << "\n";
}

static void printStatus0(const Status0& s0) {
  std::cout << "[S0] applied=" << s0.appliedOutput
            << " V=" << s0.voltage
            << " A=" << s0.current
            << " tempC=" << static_cast<int>(s0.motorTempC)
            << " HF=" << s0.hardForwardLimit
            << " HR=" << s0.hardReverseLimit
            << " SF=" << s0.softForwardLimit
            << " SR=" << s0.softReverseLimit
            << " inv=" << s0.inverted
            << " hbLock=" << s0.primaryHeartbeatLock
            << "\n";
}


int main()
{
  try {
    // Initialize SparkMax object with CAN interface (can0) and CAN ID 1 
    spark_mmrt::can::SocketCanTransport transport; 
    transport.open("vcan0");  // open/bind the socket to interface "can0"
    // transport.open("vcan0");  VCAN TESTING 
    SparkMax motor(transport, 1);

    auto start = std::chrono::high_resolution_clock::now();  // Starting timestamp
    // Loop for 10 seconds 
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count() < 10)
    {
      // Enable and run motor
      motor.Heartbeat();
      motor.SetDutyCycle(0.05); // 5% 

      auto f = transport.recv(std::chrono::microseconds{20000}); // Tested with CAN FRAME cansend vcan0 0205B801#5919667603140000
      if (!f) {
        // no frame this cycle 
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      auto &frame = *f; // currently in std::optional<canframe> need the canFrame part 
      uint8_t device = uint8_t(frame.arbId & 0x03F); 

      if(device == motor.getID()){
        motor.processFrame(frame); 
        uint32_t base = frame.arbId & ~0x3Fu;
        switch (base) {
          case STATUS1_BASE: 
            printStatus1(motor.getStatus1());
            break;

          case STATUS0_BASE: 
            printStatus0(motor.getStatus0());
            break;
      }
      // add other motors or might use a forloop or hashmap later idk 
    }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    }

  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
  return 0;
}
