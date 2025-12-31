#include "spark_mmrt/device/SparkMax.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
  try {
    // Initialize SparkMax object with CAN interface (can0) and CAN ID 1 
    spark_mmrt::can::SocketCanTransport transport; 
    transport.open("can0");  // open/bind the socket to interface "can0"
    SparkMax motor(transport, 1);

    auto start = std::chrono::high_resolution_clock::now();  // Starting timestamp
    // Loop for 10 seconds 
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count() < 10)
    {
      // Enable and run motor
      motor.Heartbeat();
      motor.SetDutyCycle(0.05); // 5% 
      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // sleep for 20 ms 

    }

  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
  return 0;
}
