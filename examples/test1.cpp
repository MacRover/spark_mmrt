#include "spark_mmrt/device/SparkMax.hpp"
#include "spark_mmrt/frames/SparkFrames.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>


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
static const char* toString(InputMode m) {
  switch (m) {
    case PWM: return "PWM";
    case CAN: return "CAN";
    case USB: return "USB";
    default:  return "UNKNOWN";
  }
}
static const char* toString(MotorType m) {
  switch (m) {
    case BRUSHED: return "BRUSHED";
    case BRUSHLESS:return "BRUSHLESS";
    default: return "UNKNOWN";
  }
}
static const char* toString(IdleMode m) {
  switch (m) {
    case COAST: return "COAST";
    case BRAKE: return "BRAKE";
    default: return "UNKNOWN";
  }
}
static const char* toString(ControlType c) {
  switch (c) {
    case DUTY_CYCLE: return "DUTY_CYCLE";
    case VELOCITY: return "VELOCITY";
    case VOLTAGE: return "VOLTAGE";
    case POSITION: return "POSITION";
    case SMARTMOTION: return "SMARTMOTION";
    case SMARTVELOCITY: return "SMARTVELOCITY";
    case MAXMOTION_POSITION: return "MAXMOTION_POSITION";
    case MAXMOTION_VELOCITY: return "MAXMOTION_VELOCITY";
    default: return "UNKNOWN";
  }
}
static const char* toString(SensorType s) {
  switch (s) {
    case NONE: return "NONE";
    case MAIN_ENCODER: return "MAIN_ENCODER";
    case ANALOG: return "ANALOG";
    case ALT_ENCODER: return "ALT_ENCODER";
    case DUTY_CYCLE: return "DUTY_CYCLE";
    default: return "UNKNOWN";
  }
}

static void printParams(const SparkMax& m) {
  const auto p = m.getParams();
  std::cout << "[SparkMax ID=" << int(m.getID()) << "] Params:\n";
  std::cout << "  CANID        = " << p.CANID << "\n";
  std::cout << "  InputMode    = " << p.InputMode << " (" << toString(m.getInputMode()) << ")\n";
  std::cout << "  MotorType    = " << p.MotorType << " (" << toString(m.getMotorType()) << ")\n";
  std::cout << "  ControlType  = " << p.ControlType << " (" << toString(m.getControlType()) << ")\n";
  std::cout << "  IdleMode     = " << p.IdleMode << " (" << toString(m.getIdleMode()) << ")\n";
  std::cout << "  CommAdvance  = " << p.CommAdvance << "\n";
  std::cout << "  SensorType   = " << p.SensorType << " (" << toString(SensorType(p.SensorType)) << ")\n";
  std::cout << "  P            = " << p.P << "\n";
  std::cout << "  I            = " << p.I << "\n";
  std::cout << "  D            = " << p.D << "\n";
  std::cout << "  F            = " << p.F << "\n";
  std::cout << "  IZ           = " << p.IZ << "\n";
  std::cout << "  DFilter      = " << p.DFilter << "\n";
  std::cout << "  OutputMin    = " << p.OutputMin << "\n";
  std::cout << "  OutputMax    = " << p.OutputMax << "\n";
  std::cout << "  period0      = " << p.period0 << "\n";
  std::cout << "  period1      = " << p.period1 << "\n";
  std::cout << "  period2      = " << p.period2 << "\n";
  std::cout << "  period3      = " << p.period3 << "\n";
  std::cout << "  period4      = " << p.period4 << "\n";
  std::cout << "  period5      = " << p.period5 << "\n";
  std::cout << "  period6      = " << p.period6 << "\n";
  std::cout << "  period7      = " << p.period7 << "\n";
  std::cout << "  MaxVelMM     = " << p.MaxVelMM << "\n";
  std::cout << "  MaxAccelMM   = " << p.MaxAccelMM << "\n";
  std::cout << "  AllowedCLEMM = " << p.AllowedClosedLoopErrorMM << "\n";
  std::cout << "  ForceStatus0 = " << p.ForceEnableStatus0 << "\n";
  std::cout << "  ForceStatus1 = " << p.ForceEnableStatus1 << "\n";
  std::cout << "  ForceStatus2 = " << p.ForceEnableStatus2 << "\n";
  std::cout << "  ForceStatus3 = " << p.ForceEnableStatus3 << "\n";
  std::cout << "  ForceStatus4 = " << p.ForceEnableStatus4 << "\n";
  std::cout << "  ForceStatus5 = " << p.ForceEnableStatus5 << "\n";
  std::cout << "  ForceStatus6 = " << p.ForceEnableStatus6 << "\n";
  std::cout << "  ForceStatus7 = " << p.ForceEnableStatus7 << "\n";

}




int main()
{
  try {
    // Initialize SparkMax object with CAN interface (can0) and CAN ID 1 
    spark_mmrt::can::SocketCanTransport transport; 
    transport.open("can0", SPARK_DRIVETRAIN);  // open/bind the socket to interface "can0"
    // transport.open("vcan0");  VCAN TESTING 
    SparkMax motor1(transport, 16);
    SparkMax motor2(transport, 2);

    motor1.readParam(param::PARAM_CANID, std::chrono::milliseconds(200));
    motor1.readParam(param::PARAM_ControlType, std::chrono::milliseconds(200));
    motor1.readParam(param::PARAM_IdleMode, std::chrono::milliseconds(200));
    motor1.readParam(param::PARAM_InputMode, std::chrono::milliseconds(200));
    motor1.readParam(param::PARAM_MotorType, std::chrono::milliseconds(200));

    printParams(motor1);

    auto rsp = motor1.setCANID(1, std::chrono::milliseconds{200});
    if (!rsp || rsp->result_code != 0) {std::cout << u_int8_t(rsp->result_code )<< "FAIL ID ";}


       rsp = motor1.setIdleMode(IdleMode::BRAKE);
      if (!rsp) {
        std::cout << "FAIL idle (timeout)\n";
      } else {
        std::cout << "idle rsp: param_id=" << int(rsp->param_id) << " value=" << rsp->value << " code=" << int(rsp->result_code) << "\n";
      }



    // rsp = motor1.setControlType(ControlType::POSITION); 
    // if (!rsp || rsp->result_code != 0) {std::cout << rsp->result_code << "FAIL control" << std::endl;}


    motor1.Flash(std::chrono::microseconds(1000));

    printParams(motor1);



    auto start = std::chrono::high_resolution_clock::now();  // Starting timestamp
    // Loop for 10 seconds 
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count() < 10)
    {
      // Enable and run motor
      motor1.heartbeat();
      motor1.setDutyCycle(0.00); // 5% 
      motor2.setDutyCycle(0.20); // 5% 

      auto f = transport.recv(std::chrono::microseconds{20000}); // Tested with CAN FRAME cansend vcan0 0205B801#5919667603140000
      if (!f) {
        // no frame this cycle 
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      auto &frame = *f; // currently in std::optional<canframe> need the canFrame part 
      uint8_t device = uint8_t(frame.arbId & 0x03F); 

      if(device == motor1.getID()){
        motor1.processFrame(frame); 
        uint32_t base = frame.arbId & ~0x3Fu;
        switch (base) {
          case STATUS1_BASE: 
            printStatus1(motor1.getStatus1());
            break;

          case STATUS0_BASE: 
            printStatus0(motor1.getStatus0());
            break;
        }
      } else if (device == motor2.getID()) {
        motor2.processFrame(frame);
        uint32_t base = frame.arbId & ~0x3Fu;
        switch (base) {
          case STATUS1_BASE: 
            printStatus1(motor2.getStatus1());
            break;

          case STATUS0_BASE: 
            printStatus0(motor2.getStatus0());
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
