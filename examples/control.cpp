#include "spark_mmrt/device/SparkMax.hpp"
#include <iostream>
#include <chrono>
#include <thread>

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

      if(device == motor.getID()) motor.processFrame(frame); 
      // add other motors or might use a forloop or hashmap later idk 
      
      std::cout << motor.getAppliedOutput() << std::endl; 
      std::cout << motor.getVoltage() << std::endl; 
      std::cout << motor.getCurrent() << std::endl; 
      std::cout << motor.getTemp() << std::endl; 

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      } 

  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
  return 0;
}
