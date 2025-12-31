# Task List + Testing (spark_mmrt)
---

## Implemneted

### Transport (SocketCAN)
- [X] `SocketCanTransport::open(interface_name)` NEED TESTING 
- [x] `SocketCanTransport::close()`, `isOpen()` NEED TESTING 
- [x] `SocketCanTransport::send(const CanFrame&)` NEED TESTING 
- [x] `SocketCanTransport::recv(timeout)` NEED TESTING 

### frame Building (from spark-frames-2.0.0-dev.11)
- [x] `makeArbID(deviceType, manufacturer, apiClass/apiIndex, deviceID)` NEED TESTING 
- [x] Payload pack helpers in `SparkFrames.cpp`: 
  - [x] `packFloat32`  NEED TESTING 
  - [x] `packInt16` NEED TESTING 
  - [x] `setBits` NEED TESTING 
- [x] Frame builders:
  - [x] `heartbeatFrame()` — Secondary Heartbeat (`apiClass=11`, `apiIndex=2`) NEED TESTING 
  - [x] `setDutyCycleFrame(dutyCycle, deviceID)` — Duty Cycle Setpoint (`apiClass=0`, `apiIndex=2`) NEED TESTING 

### control.cpp
- [x] `examples/control.cpp` opens `can0`, creates motor for CAN ID 1, sends heartbeat + duty NEED TESTING 


##  Needs Implementation Next 
- [X] Enable Status Frames 
- [X] Add GET_FIRMWARE_VERSION (RTR) 
    -able to talk to a sparkMAX, arbID format matches, and  RX path works.
- [X] Start receiving + decoding STATUS_0 / STATUS_1 / STATUS_2
    - STATUS_0: applied output / voltage / current / temp
    - STATUS_1: faults/warnings (+ follower flag)
    - STATUS_2: encoder velocity/position

- [X] Enable the status frames 
    -Parameter:
      Write Status N Period params 158–165 (controls emit rate).
    -Command approach:
      Implement SET_STATUSES_ENABLED 
- [X] Implement parameter read/write “transaction” path
 



