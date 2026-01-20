#include "spark_mmrt/frames/SparkFrames.hpp"
#include "spark_mmrt/frames/StatusFrames.hpp"
 
#include <cstring> 
#include <cstdint>
#include <array>
#include <stdexcept>



//writes specific bytes from data into payload 
template <size_t T>
static void WriteBytes(std::array<uint8_t,T> & payload, int startByte, const uint8_t* bytes, int numBytes){
  if (startByte < 0 || numBytes < 0 || startByte + numBytes > (int)T) {
    throw std::out_of_range("WriteBytes out of bounds");
  }
  for(int i = 0; i < numBytes; i++){
    payload[i+startByte] = bytes[i];
  }
}

// BYTE ALINGNED VALUES
template <size_t T>
static void packFloat32(std::array<uint8_t, T> &data, int bitPos, float f){
  const int NUMBYTES = 4;
  int startByte =  bitPos / 8; 

  uint32_t floatBits = 0;

  std::memcpy(&floatBits, &f, NUMBYTES); // copy the raw bits of float f into 32 bit integer 

  // extract each byte from floatBits into int bytes[]
  uint8_t bytes[NUMBYTES]; 
  bytes[0] = uint8_t(floatBits & 0xFF);
  bytes[1] = uint8_t((floatBits >> 8) & 0xFF);
  bytes[2] = uint8_t((floatBits >> 16) & 0xFF);
  bytes[3] = uint8_t((floatBits >> 24) & 0xFF);

  WriteBytes(data, startByte, bytes, NUMBYTES); // write the float bits into data payload from a specific byte starting point 
}


// BYTE ALIGNED VALUES 
template <size_t T>
static void packInt16(std::array<uint8_t,T> & data, int bitPos, int16_t I){
  const int NUMBYTES = 2;  
  uint16_t intBits = uint16_t(I); 
  int startByte = bitPos / 8; 

  // extract each byte from intBits into int bytes[]
  uint8_t bytes[NUMBYTES]; 
  bytes[0] = uint8_t(intBits & 0xFF);
  bytes[1] = uint8_t((intBits >> 8) & 0xFF); 

  WriteBytes(data, startByte, bytes, NUMBYTES); // write the int bits into data payload from a specific byte starting point 
}

//NON BYTE ALIGNED VALUES 
template <size_t T>
static void setBits(std::array<uint8_t, T> & data, int bitPos, int bitLen, uint32_t val){
  if (bitPos < 0 || bitLen < 0 || (bitPos + bitLen) > int(T * 8)) {
  throw std::out_of_range("setBits out of bounds");
  }
  for(int i = 0; i < bitLen; i++){
    int currBit = bitPos + i;
    int byte = currBit / 8; 
    int bit = currBit % 8; 

    uint8_t bitMask = uint8_t(1u << bit); // mask the current bit 
    if((val & (1u << i)) != 0) data[byte] |= bitMask; // if the val bit is 1 set the data bit
    else data[byte] &= uint8_t(~bitMask); // else clear the data bit 


  }
}

// Simillar to setBits but builds a uint32 bit by bit from the list 

static uint32_t getBits(const std::array<uint8_t,8> &data , int bitPos, int bitlen){
  uint32_t out = 0; 
  for(int i = 0; i < bitlen; i++){
    int currBit = bitPos + i; 
    int byte = currBit / 8; 
    int bit = currBit % 8; 

    uint32_t bitVal = (data[byte] >> bit) & 1u; 
    out |= (bitVal << i); 
  }

  return out;  
}


//Sign extension from uint32 to int 32 
static int32_t signExtend32(uint32_t x, int bitLen) {
  uint32_t m = 1u << (bitLen - 1);
  return int32_t((x ^ m) - m);
}

static float uInt32toFloat(uint32_t r){
  float f;
  std::memcpy(&f, &r, sizeof(f));
  return f;
}

// Build a heartbeat CAN frame (8 bytes all 0xFF) using the Heartbeat arbitration ID
spark_mmrt::can::CanFrame heartbeatFrame(){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::Heartbeat, 0); // CAN ID = 0 for all sparkMAX
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 8);
  frame.data.fill(0xFF);
  return frame;   
}


// Build a duty cycle command frame for a given device ID
// Payload (8 bytes) layout:
// - bits 0..31 (bytes 0..3): float32 setpoint (dutyCycle)
// - bits 32..47 (bytes 4..5): int16 feed-forward
// - bits 48..49 (byte 6, bits 0..1): PID slot
// - bit 50 (byte 6, bit 2):  Feed Forward units 
// - bits 51..63  (byte 6 bit 3 ... end): reserved
spark_mmrt::can::CanFrame setDutyCycleFrame(float dutyCycle, uint8_t deviceID){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::DutyCycle, deviceID);
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 8);
  // Bound check and correction [-1, 1]
  if (dutyCycle > 1.0f) {dutyCycle = 1.0f;}
  else if (dutyCycle < -1.0f) {dutyCycle = -1.0f;}


  frame.data.fill(0); 
  // Setpoint 
  packFloat32(frame.data, 0, dutyCycle);

  //THESE ARE ALRDY zero might delete later 
  //Feed Forward 
  packInt16(frame.data, 32, 0);
  //PID SLOT
  setBits(frame.data, 48, 2, 0); 
  //Arbitrary FeedForward Units
  setBits(frame.data, 50, 1 , 0);
  //reserved Bits 
  setBits(frame.data, 51, 13, 0); 

  return frame;   
}


spark_mmrt::can::CanFrame setVelocityFrame(float setPoint, uint8_t deviceID){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::VelocitySetpoint, deviceID); 
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 8); 

  frame.data.fill(0);
  //setpoint
  packFloat32(frame.data, 0, setPoint);

  //THESE ARE ALRDY zero might delete later 
  //feedforward
  packInt16(frame.data, 32, 0);
  //PID SLOT
  setBits(frame.data, 48, 2, 0);
  //FeedFroward uints
  setBits(frame.data, 50, 1, 0); 
  //reserved alrdy 0
  
  return frame;
}

spark_mmrt::can::CanFrame setMMVelocityFrame(float setPoint, uint8_t deviceID){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::MMVelocitySetpoint, deviceID);
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 8); 

  frame.data.fill(0); 

  packFloat32(frame.data, 0, setPoint);

  //THESE ARE ALRDY zero might delete later 
  //feedforward
  packInt16(frame.data, 32, 0);
  //PID SLOT
  setBits(frame.data, 48, 2, 0);
  //FeedFroward uints
  setBits(frame.data, 50, 1, 0); 
  //reserved alrdy 0

  return frame;
}

spark_mmrt::can::CanFrame setPositionFrame(float setPoint, uint8_t deviceID){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::PositionSetpoint, deviceID); 
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 8); 

  frame.data.fill(0);
  //setpoint
  packFloat32(frame.data, 0, setPoint);

  //THESE ARE ALRDY zero might delete later 
  //feedforward
  packInt16(frame.data, 32, 0);
  //PID SLOT
  setBits(frame.data, 48, 2, 0);
  //FeedFroward uints
  setBits(frame.data, 50, 1, 0); 
  //reserved alrdy 0
  
  return frame;
}

spark_mmrt::can::CanFrame setMMPositionFrame(float setPoint, uint8_t deviceID){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::MMPositionSetpoint, deviceID);
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 8); 

  frame.data.fill(0); 

  packFloat32(frame.data, 0, setPoint);

  //THESE ARE ALRDY zero might delete later 
  //feedforward
  packInt16(frame.data, 32, 0);
  //PID SLOT
  setBits(frame.data, 48, 2, 0);
  //FeedFroward uints
  setBits(frame.data, 50, 1, 0); 
  //reserved alrdy 0

  return frame;
}

spark_mmrt::can::CanFrame setVoltageFrame(float setPoint, uint8_t deviceID){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::voltageSetpoint, deviceID);
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 8); 

  frame.data.fill(0); 

  packFloat32(frame.data, 0, setPoint);

  //THESE ARE ALRDY zero might delete later 
  //feedforward
  packInt16(frame.data, 32, 0);
  //PID SLOT
  setBits(frame.data, 48, 2, 0);
  //FeedFroward uints
  setBits(frame.data, 50, 1, 0); 
  //reserved alrdy 0

  return frame;
}

spark_mmrt::can::CanFrame setCurrentFrame(float setPoint, uint8_t deviceID){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::currentSetpoint, deviceID);
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 8); 

  frame.data.fill(0); 

  packFloat32(frame.data, 0, setPoint);

  //THESE ARE ALRDY zero might delete later 
  //feedforward
  packInt16(frame.data, 32, 0);
  //PID SLOT
  setBits(frame.data, 48, 2, 0);
  //FeedFroward uints
  setBits(frame.data, 50, 1, 0); 
  //reserved alrdy 0

  return frame;
}

spark_mmrt::can::CanFrame setEncoderPositionFrame(float position, uint8_t deviceID){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::setEncoderPosition, deviceID);
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 5); 

  frame.data.fill(0); 

  packFloat32(frame.data, 0, position);

  //uints 
  setBits(frame.data, 32, 8, 3);

  return frame;
}

spark_mmrt::can::CanFrame paramWriteFrame(uint32_t val, uint8_t paramID, uint8_t deviceID){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::parameterWrite, deviceID);
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 5); 

  frame.data.fill(0);

  setBits(frame.data, 0, 8, paramID); 

  setBits(frame.data, 8, 32, val); 

  return frame; 
}

spark_mmrt::can::CanFrame paramWriteFloatFrame(float val, uint8_t paramID, uint8_t deviceID){
  uint32_t bits = 0;
  std::memcpy(&bits, &val, sizeof(val));
  return paramWriteFrame(bits, paramID, deviceID);  
}

spark_mmrt::can::CanFrame persistParamFrame(uint8_t deviceID){
  const uint32_t ID = makeArbID(DEVICE_TYPE, MANUFACTURER, api::presistParam, deviceID);
  auto frame = spark_mmrt::can::CanFrame::Data(ID, 2); 

  frame.data.fill(0);

  setBits(frame.data, 0, 16, 15011);

  return frame; 
}


ParamWriteResponse paramWriteResponseDecode(const std::array<uint8_t, 8> &data){
  ParamWriteResponse pr{};
  pr.param_id = uint8_t(getBits(data, 0, 8));
  pr.param_type = uint8_t(getBits(data, 8, 8));
  pr.value = getBits(data, 16, 32);
  pr.result_code = uint8_t(getBits(data, 48, 8));
  return pr;
}






// Status 0 frame (SPARK MAX to program) for a given device ID
// Payload (8 bytes) layout:
// - bits 0..15: Applied Output  [-1, 1]
// - bits 16..27: Voltage [V]
// - bits 28..39: Current [A]
// - bits 40..47  Motor Temperature [°C]
// - bit  48: Hard Forward Limit Reached (bool)
// - bit  49: Hard Reverse Limit Reached (bool)
// - bit  50: Soft Forward Limit Reached (bool)
// - bit  51: Soft Reverse Limit Reached (bool)
// - bit  52: Inverted (bool)
// - bit  53: Primary Heartbeat Lock (bool)
// - bits 54..63: Reserved 
void status0Decoder(const std::array<uint8_t, 8> &data, Status0& s0){
  // data is uint but applied output is int need to sign extend after building 
  int32_t aoRaw = signExtend32(getBits(data, 0, 16), 16); 
  s0.appliedOutput = double(aoRaw) * Status0Scale::appliedOutputScale;

  uint32_t vRaw = getBits(data, 16, 12); 
  s0.voltage = double(vRaw) * Status0Scale::voltageScale; 

  uint32_t iRaw = getBits(data, 28, 12);
  s0.current = double(iRaw) * Status0Scale::currentScale; 

  s0.motorTempC = uint8_t(getBits(data, 40, 8)); 

  s0.hardForwardLimit = getBits(data, 48, 1); 
  s0.hardReverseLimit = getBits(data, 49, 1); 
  s0.softForwardLimit = getBits(data, 50 , 1); 
  s0.softReverseLimit = getBits(data, 51, 1);
  s0.inverted = getBits(data, 52, 1); 
  s0.primaryHeartbeatLock = getBits(data, 53, 1);

  //s0.sparkModel = getBits(data, 54, 4); // IDK IF ITS PUBLIC YET 


}

void status1Decoder(const std::array<uint8_t, 8> &data, Status1& s1){
  s1.otherFault = getBits(data, 0, 1); 
  s1.motorTypeFault = getBits(data, 1, 1);
  s1.sensorFault = getBits(data, 2, 1); 
  s1.canFault = getBits(data, 3, 1);
  s1.temperatureFault = getBits(data, 4, 1);
  s1.drvFault = getBits(data, 5, 1);
  s1.escEepromFault = getBits(data, 6, 1);
  s1.firmwareFault = getBits(data, 7, 1);

  s1.reservedActives = uint8_t(getBits(data, 8, 8)); 

  s1.brownoutWarning = getBits(data, 16, 1); 
  s1.overcurrentWarning = getBits(data, 17, 1); 
  s1.escEepromWarning = getBits(data, 18, 1);
  s1.extEepromWarning = getBits(data, 19, 1); 
  s1.sensorWarning = getBits(data, 20, 1); 
  s1.stallWarning = getBits(data, 21, 1); 
  s1.hasResetWarning = getBits(data, 22, 1); 
  s1.otherWarning = getBits(data, 23, 1); 

  s1.otherStickyFault = getBits(data ,24, 1); 
  s1.motorTypeStickyFault = getBits(data, 25, 1); 
  s1.sensorStickyFault = getBits(data, 26, 1); 
  s1.canStickyFault = getBits(data, 27, 1);
  s1.temperatureStickyFault = getBits(data, 28, 1);
  s1.drvStickyFault= getBits(data, 29, 1);
  s1.escEepromStickyFault = getBits(data, 30, 1); 
  s1.firmwareStickyFault = getBits(data, 31, 1); 

  s1.reservedStickies = uint8_t(getBits(data, 32, 8)); 

  s1.brownoutStickyWarning = getBits(data, 40, 1); 
  s1.overcurrentStickyWarning = getBits(data, 41, 1);
  s1.escEepromStickyWarning = getBits(data, 42, 1); 
  s1.extEepromStickyWarning = getBits(data, 43, 1); 
  s1.sensorStickyWarning = getBits(data, 44, 1);
  s1.stallStickyWarning = getBits(data, 45, 1);
  s1.hasResetStickyWarning = getBits(data, 46, 1);
  s1.otherStickyWarning = getBits(data, 47, 1); 
  s1.isFollower = getBits(data, 48, 1); 
  s1.reserved = uint16_t(getBits(data, 49, 15));
}


void status2Decoder(const std::array<uint8_t, 8> &data, Status2& s2){
  uint32_t vRaw = getBits(data, 0, 32);
  s2.primaryEncoderVelocity = uInt32toFloat(vRaw);

  uint32_t pRaw = getBits(data, 32, 32); 
  s2.primaryEncoderPosition = uInt32toFloat(pRaw); 
  
}

void status3Decoder(const std::array<uint8_t, 8> &data, Status3& s3){
  uint32_t aVoltageRaw = getBits(data, 0, 10);   
  s3.analogVoltage = double(aVoltageRaw) * Status3Scale::analogVoltageScale;

  int32_t aVelocityRaw = signExtend32(getBits(data, 10, 22), 22);
  s3.analogVelocity = double(aVelocityRaw) * Status3Scale::analogVelocityScale;

  uint32_t aPositionRaw = getBits(data, 32, 32); 

  s3.analogPosition = double(uInt32toFloat(aPositionRaw));
}

void status4Decoder(const std::array<uint8_t, 8> &data, Status4& s4){
  uint32_t altEncV = getBits(data, 0, 32); 
  s4.altEncoderVelocity = double(uInt32toFloat(altEncV));

  uint32_t altEncP = getBits(data, 32, 32); 
  s4.altEncoderPosition = double(uInt32toFloat(altEncP));
}
void status5Decoder(const std::array<uint8_t, 8> &data, Status5& s5){

  uint32_t dutyCycleEncV = getBits(data, 0, 32); 
  s5.dutyCycleEncVelocity = double(uInt32toFloat(dutyCycleEncV));

  uint32_t dutyCycleEncP = getBits(data, 32, 32); 
  s5.dutyCycleEncVPosition = double(uInt32toFloat(dutyCycleEncP)); 
}

void status6Decoder(const std::array<uint8_t, 8> &data, Status6& s6){

  uint32_t unadjustDCRaw = getBits(data, 0, 16); 
  s6.unadjustedDutyCycle = double(unadjustDCRaw) * Status6Scale::unadjustedDutyCycleScale;
  
  s6.dutyCyclePeriod = getBits(data, 16, 16); 
  s6.dutyCycleNoSignal = getBits(data, 32, 1); 
  s6.dutyCycleReserved = getBits(data, 33, 31); 

}

void status7Decoder(const std::array<uint8_t, 8> &data, Status7& s7){
  uint32_t IAccRaw = getBits(data, 0, 32);
  s7.IAccumalation = double(uInt32toFloat(IAccRaw));

  s7.reserved = getBits(data, 32, 32); 

}

void status8Decoder(const std::array<uint8_t, 8> &data, Status8& s8){

 uint32_t setPointRaw = getBits(data, 0, 32); 
 s8.setPoint = double(uInt32toFloat(setPointRaw));

 s8.isAtSetpoint = getBits(data, 32, 1); 
 s8.selectedPIDSlot = getBits(data, 33, 4); 
 s8.reserved = getBits(data, 37, 27); 

} 

void status9Decoder(const std::array<uint8_t, 8> &data, Status9& s9){
  uint32_t MMPositionSPRaw = getBits(data, 0, 32); 
  s9.MaxMotionPositionSetPoint = double(uInt32toFloat(MMPositionSPRaw));

  uint32_t MMVelocitySPRaw = getBits(data, 32, 32); 
  s9.MaxMotionVelocitySetPoint = double(uInt32toFloat(MMVelocitySPRaw));

}
