#ifndef SOCKETCANTRANSPORT_HPP
#define SOCKETCANTRANSPORT_HPP

#include "spark_mmrt/can/CanFrame.hpp"

#include <chrono>
#include <optional>
#include <string>

#include <linux/can.h>
#include <net/if.h>

namespace spark_mmrt::can {

class SocketCanTransport
{
public:
  SocketCanTransport() {} // constructor
  ~SocketCanTransport(); // destructor 

  void open(const std::string & interface_name); // create a CAN RAW socket for interface (can0)
  void close(); 
  bool isOpen() const;

  void send(const CanFrame & frame); // Takes CAN frame and writes to CAN socket 
  std::optional<CanFrame> recv(std::chrono::microseconds timeout); // timed Recieve for a CAN frame 

private:
  inline static int socket_fd_ = -1; // Linux file descriptor for CAN socket, shared across all objects (-1 Not open)
  std::string interface_name_; // CAN network name (can0)
  struct sockaddr_can addr_; // SocketCAN Address info 
  struct ifreq ifr_; // to go from interface name to interface index 
};

} 
#endif
