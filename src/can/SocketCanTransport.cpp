#include "spark_mmrt/can/SocketCanTransport.hpp"
#include "spark_mmrt/frames/SparkFrames.hpp"


#include <stdexcept>
#include <system_error>

#include <cerrno>
#include <chrono>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <optional>

namespace spark_mmrt::can {

SocketCanTransport::~SocketCanTransport() // Close the network 
{
  try {
    close();
  } catch (...) {
  }
}

void SocketCanTransport::open(const std::string & interface_name) 
{
  if (isOpen()) {
    throw std::logic_error("SocketCanTransport is already open");
  }
  if (interface_name.empty()) {
    throw std::invalid_argument("interface_name must not be empty");
  }

  interface_name_ = interface_name;

  //Create SocketCAN socket 
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) { // socket() failed 
    throw std::system_error(
      errno, std::generic_category(),
      "Socket creation failed: " + std::string(std::strerror(errno)) +
      "\nPossible causes:\n"
      "1. CAN modules not loaded\n"
      "2. System resource limitations");
  }

  //clear ifr and copy interface name into it 
  std::memset(&ifr_, 0, sizeof(ifr_));
  std::strncpy(ifr_.ifr_name, interface_name_.c_str(), sizeof(ifr_.ifr_name) - 1);


  // Ask the kernel what the interface index for this interface name
  // 0 if success (populates ifr_ifrindex)  non zero if not 
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr_) < 0) {
    ::close(socket_fd_);
    throw std::runtime_error(
      "IOCTL failed: " + std::string(std::strerror(errno)) +
      "\nPossible causes:\n"
      "1. CAN interface does not exist\n"
      "2. CAN bus not initialized\n"
      "3. CAN interface is not up");
  }
  // Build the SocketCAN address to bind to the interface.
  // AF_CAN -> address family for CAN
  // ifindex -> which CAN interface 
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;


  // Bind the socket to the CAN interface. 
  // 0 for success non zero for fail 
  if (bind(socket_fd_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
    ::close(socket_fd_);
    throw std::runtime_error(
      "Binding to interface failed: Another program may be using this interface.");
  }

  //Filtering only sparkMax frames in the CAN network 
  can_filter f{};
  const uint32_t sparkPrefix = (uint32_t(DEVICE_TYPE) << 24) | (uint32_t(MANUFACTURER) << 16);
  f.can_id = CAN_EFF_FLAG | sparkPrefix; 
  f.can_mask = CAN_EFF_FLAG | (CAN_EFF_MASK & 0xFFFF0000u); // match device/manufacturer, ignore RTR/ERR flags

  if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &f, sizeof(f)) < 0) {
    ::close(socket_fd_); 
    socket_fd_ = -1; 
    throw std::system_error(errno, std::generic_category(), "setsockopt(CAN_RAW_FILTER) failed");
  }

}

void SocketCanTransport::close()
{
  if (socket_fd_ >= 0) { // check for open
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  interface_name_.clear();
}

// True if we currently have a valid open socket file descriptor
bool SocketCanTransport::isOpen() const
{
  return socket_fd_ >= 0;
}



// Send a project-level CanFrame through SocketCAN.
//  1.Convert CanFrame param to linux struct can_frame
//  2. write the raw frame bytes to the socket
void SocketCanTransport::send(const CanFrame & frame)
{
  if (!isOpen()) {
    throw std::logic_error("SocketCanTransport is not open");
  }
  if (!frame.valid()) {
    throw std::out_of_range("Invalid CAN frame");
  }

  // Linux raw CAN frame type
  can_frame raw{};

  // Mask the fram arbID to ensure 29 bits 
  raw.can_id = frame.arbId & kExtIdMask;
  // copy data length code
  raw.can_dlc = frame.dlc;
  // set CAN_EFF_FLAG for extended frame format (29 bits)
  raw.can_id |= CAN_EFF_FLAG;

  // check for remote transmission request and set flag 
  if (frame.isRTR) {
    raw.can_id |= CAN_RTR_FLAG;
  }

  // copy payload if not remote transmission request and data length > 0 
  if (!frame.isRTR && frame.dlc > 0) {
    std::memcpy(raw.data, frame.data.data(), frame.dlc);
  }

  // Write can_frame struct to the socket.
  ssize_t bytesSent = write(socket_fd_, &raw, sizeof(raw));   // write() should return sizeof(raw) on success.
  if (bytesSent != sizeof(raw)) {
    throw std::system_error(errno, std::generic_category(), "SocketCAN write() failed");
  }
}

// Receive one CAN frame, waiting up to 'timeout'.
// Returns std::nullopt if timeout expires or CanFrame if a frame was received
std::optional<CanFrame> SocketCanTransport::recv(std::chrono::microseconds timeout)
{
  if (!isOpen()) {
    throw std::logic_error("SocketCanTransport is not open");
  }

  fd_set read_fds; // set of FD's to monitor for reading 
  FD_ZERO(&read_fds); // clear set 
  FD_SET(socket_fd_, &read_fds); // add socket_fd to set and interrupt when its readable (CAN FRAME in socket recieve buffer)

  timeval tv{};
  tv.tv_sec = long(timeout.count() / 1000000);
  tv.tv_usec = long(timeout.count() % 1000000);

  // read socket_fd for time interval tv 
  // ret = 0 (timeout)
  // ret < 0 (error)
  int ret = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ret <= 0) {
    return std::nullopt;
  }

  // read linux CAN frame from socket 
  can_frame response{};
  ssize_t bytesRead = read(socket_fd_, &response, sizeof(response));
  if (bytesRead < 0) {
    throw std::system_error(errno, std::generic_category(), "SocketCAN read() failed");
  }
  if (bytesRead == 0) {
    return std::nullopt;
  }

  //Convert Linux CAN frame to SparkMAX CAN frame (CanFrame.hpp)
  CanFrame frame{};
  frame.isRTR = (response.can_id & CAN_RTR_FLAG) != 0;
  frame.dlc = response.can_dlc;
  frame.timestamp = std::chrono::steady_clock::now();
  // Extract 29-bit arbitration ID
  frame.arbId = response.can_id & CAN_EFF_MASK;

  // copy payload if not remote transmission request and data length > 0 
  if (!frame.isRTR && frame.dlc > 0) {
    std::memcpy(frame.data.data(), response.data, frame.dlc);
  }

  return frame;
}

}
