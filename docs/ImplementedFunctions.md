# Implemented Functions

This document lists the currently implemented functions for sparkMMRT library 

## include/spark_mmrt/can

### `include/spark_mmrt/can/CanFrame.hpp`

**struct `spark_mmrt::can::CanFrame`**
- `static CanFrame Data(uint32_t arbId, uint8_t len)`: Build a data frame with a 29-bit arbitration ID and payload length.
- `static CanFrame RTR(uint32_t arbId)`: Build a remote transmission request frame with a 29-bit arbitration ID.
- `bool valid() const`: Validate frame.

### `include/spark_mmrt/can/SocketCanTransport.hpp`

**class `spark_mmrt::can::SocketCanTransport`**
- `SocketCanTransport()`: Construct a transport instance with no open socket.
- `~SocketCanTransport()`: Close the socket if it is open.
- `void open(const std::string& interface_name, SPARK_SUBSYSTEM_TYPE system_type)`: Open and bind a CAN RAW socket with a subsystem filter.
- `void close()`: Close the CAN socket and clear the interface name.
- `bool isOpen() const`: Check whether the socket is currently open.
- `void send(const CanFrame& frame)`: Send a CAN frame on the socket.
- `std::optional<CanFrame> recv(std::chrono::microseconds timeout)`: Receive one CAN frame or return empty on timeout.

## include/spark_mmrt/device

### `include/spark_mmrt/device/SparkMax.hpp`

**class `SparkMax`**
- `SparkMax(spark_mmrt::can::SocketCanTransport& transport_, uint8_t ID_)`: constructor
- `~SparkMax()`: destructor 
- `Status0 getStatus0() const`: Return the  Status 0 data.
- `Status1 getStatus1() const`: Return the  Status 1 data.
- `Status2 getStatus2() const`: Return the  Status 2 data.
- `Status3 getStatus3() const`: Return the  Status 3 data.
- `Status4 getStatus4() const`: Return the  Status 4 data.
- `Status5 getStatus5() const`: Return the  Status 5 data.
- `Status6 getStatus6() const`: Return the  Status 6 data.
- `Status7 getStatus7() const`: Return the  Status 7 data.
- `Status8 getStatus8() const`: Return the  Status 8 data.
- `Status9 getStatus9() const`: Return the  Status 9 data.
- `param::Params getParams() const`: Return the parameter.
- `InputMode getInputMode() const`: Return the input mode.
- `MotorType getMotorType() const`: Return the motor type.
- `IdleMode getIdleMode() const`: Return the idle mode.
- `ControlType getControlType() const`: Return the control type.
- `SensorType getSensorType() const`: Return the sensor type.
- `std::optional<ParamWriteResponse> setCANID(uint32_t ID, std::chrono::milliseconds timeout)`: Write the device CAN ID parameter.
- `std::optional<ParamWriteResponse> setIdleMode(IdleMode mode, std::chrono::milliseconds timeout)`: Write and apply idle mode.
- `std::optional<ParamWriteResponse> setControlType(ControlType type, std::chrono::milliseconds timeout)`: Write and apply control type.
- `std::optional<ParamWriteResponse> setSensorType(SensorType type, std::chrono::milliseconds timeout)`: Write and apply sensor type.
- `std::optional<ParamWriteResponse> setP(float val, std::chrono::milliseconds timeout)`: Write the P gain for slot 0.
- `std::optional<ParamWriteResponse> setI(float val, std::chrono::milliseconds timeout)`: Write the I gain for slot 0.
- `std::optional<ParamWriteResponse> setD(float val, std::chrono::milliseconds timeout)`: Write the D gain for slot 0.
- `std::optional<ParamWriteResponse> setF(float val, std::chrono::milliseconds timeout)`: Write the F gain for slot 0.
- `std::optional<ParamWriteResponse> setIZ(float val, std::chrono::milliseconds timeout)`: Write the I-zone for slot 0.
- `std::optional<ParamWriteResponse> setDFilter(float val, std::chrono::milliseconds timeout)`: Write the D filter for slot 0.
- `std::optional<ParamWriteResponse> setOutputMin(float val, std::chrono::milliseconds timeout)`: Write the minimum output for slot 0.
- `std::optional<ParamWriteResponse> setOutputMax(float val, std::chrono::milliseconds timeout)`: Write the maximum output for slot 0.
- `std::optional<ParamWriteResponse> setStatus0(uint32_t val, std::chrono::milliseconds timeout)`: Write the Status 0 period/enable parameter.
- `std::optional<ParamWriteResponse> setStatus1(uint32_t val, std::chrono::milliseconds timeout)`: Write the Status 1 period/enable parameter.
- `std::optional<ParamWriteResponse> setStatus2(uint32_t val, std::chrono::milliseconds timeout)`: Write the Status 2 period/enable parameter.
- `std::optional<ParamWriteResponse> setStatus3(uint32_t val, std::chrono::milliseconds timeout)`: Write the Status 3 period/enable parameter.
- `std::optional<ParamWriteResponse> setStatus4(uint32_t val, std::chrono::milliseconds timeout)`: Write the Status 4 period/enable parameter.
- `std::optional<ParamWriteResponse> setStatus5(uint32_t val, std::chrono::milliseconds timeout)`: Write the Status 5 period/enable parameter.
- `std::optional<ParamWriteResponse> setStatus6(uint32_t val, std::chrono::milliseconds timeout)`: Write the Status 6 period/enable parameter.
- `std::optional<ParamWriteResponse> setStatus7(uint32_t val, std::chrono::milliseconds timeout)`: Write the Status 7 period/enable parameter.
- `std::optional<ParamWriteResponse> setMaxVelMM(float val, std::chrono::milliseconds timeout)`: Write MaxMotion max velocity for slot 0.
- `std::optional<ParamWriteResponse> setMaxAccelMM(float val, std::chrono::milliseconds timeout)`: Write MaxMotion max acceleration for slot 0.
- `std::optional<ParamWriteResponse> setAllowedClosedLoopErrorMM(float val, std::chrono::milliseconds timeout)`: Write MaxMotion allowed closed-loop error for slot 0.
- `std::optional<ParamWriteResponse> setForceEnableStatus0(bool enabled, std::chrono::milliseconds timeout)`: Force-enable Status 0 frames.
- `std::optional<ParamWriteResponse> setForceEnableStatus1(bool enabled, std::chrono::milliseconds timeout)`: Force-enable Status 1 frames.
- `std::optional<ParamWriteResponse> setForceEnableStatus2(bool enabled, std::chrono::milliseconds timeout)`: Force-enable Status 2 frames.
- `std::optional<ParamWriteResponse> setForceEnableStatus3(bool enabled, std::chrono::milliseconds timeout)`: Force-enable Status 3 frames.
- `std::optional<ParamWriteResponse> setForceEnableStatus4(bool enabled, std::chrono::milliseconds timeout)`: Force-enable Status 4 frames.
- `std::optional<ParamWriteResponse> setForceEnableStatus5(bool enabled, std::chrono::milliseconds timeout)`: Force-enable Status 5 frames.
- `std::optional<ParamWriteResponse> setForceEnableStatus6(bool enabled, std::chrono::milliseconds timeout)`: Force-enable Status 6 frames.
- `std::optional<ParamWriteResponse> setForceEnableStatus7(bool enabled, std::chrono::milliseconds timeout)`: Force-enable Status 7 frames.
- `void heartbeat()`: Send a heartbeat .
- `void setDutyCycle(float val)`: Send a duty cycle command .
- `void setVelocity(float val)`: Send a velocity setpoint.
- `void setMMVelocity(float val)`: Send a MaxMotion velocity setpoint .
- `void setPosition(float val)`: Send a position setpoint .
- `void setMMPosition(float val)`: Send a MaxMotion position setpoint .
- `void setVoltage(float val)`: Send a voltage setpoint .
- `void setCurrent(float val)`: Send a current setpoint .
- `void setEncoderPosition(float val)`: Send a set-encoder-position .
- `void processFrame(const spark_mmrt::can::CanFrame& f)`: Decode a status frame into status structs.
- `bool Flash(std::chrono::microseconds timeout)`: Send persist-parameters command and wait for response.
- `bool readParam(param::ParamID paramID, std::chrono::milliseconds timeout)`: Read a parameter.
- `std::optional<ParamReadResponse> readParamWithType(param::ParamID paramID, std::chrono::milliseconds timeout)`: Read a parameter and return its type and raw value.
- `std::optional<ParamWriteResponse> writeParam(param::ParamID pid, uint32_t value, std::chrono::milliseconds timeout)`: Write a parameter and return the response.
- `uint8_t getID() const`: Return the device CAN ID.

## include/spark_mmrt/frames

### `include/spark_mmrt/frames/SparkFrames.hpp`

- `uint32_t makeArbID(uint8_t deviceType, uint8_t manufacturer, Api api, uint8_t deviceID)`: Build a 29-bit SparkMax arbitration ID.
- `uint64_t sparkMaxDeviceIDMask(uint8_t canID)`: Build a bitmask with a single CAN ID bit set.
- `spark_mmrt::can::CanFrame heartbeatFrame()`: Build a heartbeat frame.
- `spark_mmrt::can::CanFrame setDutyCycleFrame(float dutyCycle, uint8_t deviceID)`: Build a duty cycle command frame.
- `spark_mmrt::can::CanFrame setVelocityFrame(float setPoint, uint8_t deviceID)`: Build a velocity setpoint command frame.
- `spark_mmrt::can::CanFrame setMMVelocityFrame(float setPoint, uint8_t deviceID)`: Build a MaxMotion velocity setpoint command frame.
- `spark_mmrt::can::CanFrame setPositionFrame(float setPoint, uint8_t deviceID)`: Build a position setpoint command frame.
- `spark_mmrt::can::CanFrame setMMPositionFrame(float setPoint, uint8_t deviceID)`: Build a MaxMotion position setpoint command frame.
- `spark_mmrt::can::CanFrame setVoltageFrame(float setPoint, uint8_t deviceID)`: Build a voltage setpoint command frame.
- `spark_mmrt::can::CanFrame setCurrentFrame(float setPoint, uint8_t deviceID)`: Build a current setpoint command frame.
- `spark_mmrt::can::CanFrame setEncoderPositionFrame(float position, uint8_t deviceID)`: Build a set-encoder-position command frame.
- `spark_mmrt::can::CanFrame paramWriteFrame(uint32_t val, uint8_t paramID, uint8_t deviceID)`: Build a parameter write frame (raw 32-bit value).
- `spark_mmrt::can::CanFrame paramWriteFloatFrame(float val, uint8_t paramID, uint8_t deviceID)`: Build a parameter write frame from a float.
- `spark_mmrt::can::CanFrame persistParamFrame(uint8_t deviceID)`: Build a persist-parameters (flash) command frame.
- `ParamWriteResponse paramWriteResponseDecode(const std::array<uint8_t, 8>& data)`: Decode a parameter write response payload.
- `void status0Decoder(const std::array<uint8_t, 8>& data, Status0& s0)`: Decode Status 0 payload into `Status0`.
- `void status1Decoder(const std::array<uint8_t, 8>& data, Status1& s1)`: Decode Status 1 payload into `Status1`.
- `void status2Decoder(const std::array<uint8_t, 8>& data, Status2& s2)`: Decode Status 2 payload into `Status2`.
- `void status3Decoder(const std::array<uint8_t, 8>& data, Status3& s3)`: Decode Status 3 payload into `Status3`.
- `void status4Decoder(const std::array<uint8_t, 8>& data, Status4& s4)`: Decode Status 4 payload into `Status4`.
- `void status5Decoder(const std::array<uint8_t, 8>& data, Status5& s5)`: Decode Status 5 payload into `Status5`.
- `void status6Decoder(const std::array<uint8_t, 8>& data, Status6& s6)`: Decode Status 6 payload into `Status6`.
- `void status7Decoder(const std::array<uint8_t, 8>& data, Status7& s7)`: Decode Status 7 payload into `Status7`.
- `void status8Decoder(const std::array<uint8_t, 8>& data, Status8& s8)`: Decode Status 8 payload into `Status8`.
- `void status9Decoder(const std::array<uint8_t, 8>& data, Status9& s9)`: Decode Status 9 payload into `Status9`.
