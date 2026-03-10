#include "spark_mmrt/device/SparkMax.hpp"

#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr uint8_t kMotorId = 17;
constexpr float kActiveDuty = 0.05f;  // 5% duty command

volatile std::sig_atomic_t g_running = 1;

void onSignal(int) {
  g_running = 0;
}

class TerminalRawMode {
 public:
  TerminalRawMode() {
    if (tcgetattr(STDIN_FILENO, &original_) != 0) {
      throw std::runtime_error("tcgetattr failed");
    }
    termios raw = original_;
    raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error("tcsetattr failed");
    }
  }

  ~TerminalRawMode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_);
  }

  TerminalRawMode(const TerminalRawMode&) = delete;
  TerminalRawMode& operator=(const TerminalRawMode&) = delete;

 private:
  termios original_{};
};

bool pollChar(char& out) {
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);

  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 0;

  const int ready = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
  if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &readfds)) {
    return false;
  }

  const ssize_t n = read(STDIN_FILENO, &out, 1);
  return n == 1;
}

}  // namespace

int main() {
  try {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    spark_mmrt::can::SocketCanTransport transport;
    transport.open("can0", SPARK_DRIVETRAIN);
    SparkMax motor(transport, kMotorId);

    TerminalRawMode rawModeGuard;

    std::cout << "Keyboard control started for motor CAN ID " << int(kMotorId) << ".\n";
    std::cout << "Hold W or D => duty " << kActiveDuty << ", release => duty 0.0\n";
    std::cout << "Press Q to quit.\n";

    auto activeUntil = std::chrono::steady_clock::now();
    auto lastPrint = std::chrono::steady_clock::now();

    while (g_running) {
      char c = '\0';
      while (pollChar(c)) {
        if (c == 'q' || c == 'Q') {
          g_running = 0;
          break;
        }
        if (c == 'w' || c == 'W' || c == 'd' || c == 'D') {
          // We cannot read true key-release in this simple terminal loop.
          // Extend active window when key-repeat chars arrive.
          activeUntil = std::chrono::steady_clock::now() + std::chrono::milliseconds(120);
        }
      }

      const bool active = std::chrono::steady_clock::now() < activeUntil;
      const float duty = active ? kActiveDuty : 0.0f;

      motor.heartbeat();
      motor.setDutyCycle(duty);

      const auto now = std::chrono::steady_clock::now();
      if (now - lastPrint >= std::chrono::milliseconds(200)) {
        std::cout << "\rDuty command: " << duty << "      " << std::flush;
        lastPrint = now;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    motor.setDutyCycle(0.0f);
    motor.heartbeat();
    std::cout << "\nStopped. Duty set to 0.0\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
