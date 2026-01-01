# Task List + Testing (spark_mmrt)
---

## Implemneted

### Transport (SocketCAN)
- [X] `SocketCanTransport::open(interface_name)` TESTED THROUGH VCAN 
- [x] `SocketCanTransport::close()`, `isOpen()` TESTED THROUGH VCAN 
- [x] `SocketCanTransport::send(const CanFrame&)` TESTED THROUGH VCAN 
- [x] `SocketCanTransport::recv(timeout)` TESTED THROUGH VCAN 

### frame Building (from spark-frames-2.0.0-dev.11)
- [x] `makeArbID(deviceType, manufacturer, apiClass/apiIndex, deviceID)` TESTED THROUGH VCAN 
- [x] Payload pack helpers in `SparkFrames.cpp`: 
  - [x] `packFloat32`  TESTED THROUGH VCAN 
  - [x] `packInt16` TESTED THROUGH VCAN 
  - [x] `setBits` TESTED THROUGH VCAN 
  - [x] `GetBits` TESTED THROUGH VCAN 
  - [x] `SignExtend` TESTED THROUGH VCAN 

- [x] Frame builders:
  - [x] `heartbeatFrame()` — Secondary Heartbeat (`apiClass=11`, `apiIndex=2`) TESTED THROUGH VCAN 
  - [x] `setDutyCycleFrame(dutyCycle, deviceID)` — Duty Cycle Setpoint (`apiClass=0`, `apiIndex=2`) TESTED THROUGH VCAN 
  - [x] `processFrame` - Porcess recieved frame to determine StatusType TESTED THROUGH VCAN 
  - [x] `Status0Decoder` - Given CAN frame for status Frame 0 decode and put into status0 struct TESTED THROUGH VCAN 

- [x] Getters for StatusFrame0 TESTED THROUGH VCAN 


### control.cpp
- [x] `examples/control.cpp` opens `can0`, creates motor for CAN ID 1, sends heartbeat + duty TESTED THROUGH VCAN 


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
 



