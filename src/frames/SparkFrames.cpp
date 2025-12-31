#include "spark_mmrt/frames/SparkFrames.hpp"
 
#include <cstring> 
#include <cstdint>
#include <array>




//writes specific bytes from data into payload 
static void WriteBytes(std::array<uint8_t,8> & payload, int startByte, const uint8_t* bytes, int numBytes){
  for(int i = 0; i < numBytes; i++){
    payload[i+startByte] = bytes[i];
  }
}

// BYTE ALINGNED VALUES
 static void packFloat32(std::array<uint8_t, 8> &data, int bitPos, float f){
  const int NUMBYTES = 4; // 32 bits = 4 bytes 
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
static void packInt16(std::array<uint8_t,8> & data, int bitPos, int16_t I){
  const int NUMBYTES = 2; // 2 bytes = 16 bits 
  uint16_t intBits = uint16_t(I); // cast integer to unsigned 
  int startByte = bitPos / 8; 

  // extract each byte from intBits into int bytes[]
  uint8_t bytes[NUMBYTES]; 
  bytes[0] = uint8_t(intBits & 0xFF);
  bytes[1] = uint8_t((intBits >> 8) & 0xFF); 

  WriteBytes(data, startByte, bytes, NUMBYTES); // write the int bits into data payload from a specific byte starting point 
}

//NON BYTE ALIGNED VALUES 
static void setBits(std::array<uint8_t, 8> & data, int bitPos, int bitLen, uint32_t val){
  for(int i = 0; i < bitLen; i++){
    int currBit = bitPos + i; // find current bit num of total (32) 
    int byte = currBit / 8; // find byte num (1, 2, 3, 4)
    int bit = currBit % 8; // find  bit num withn each byte (eg. first byte second bit)

    uint8_t bitMask = uint8_t(1u << bit); // mask the current bit 
    if((val & (1u << i)) != 0) data[byte] |= bitMask; // if the val bit is 1 set the data bit
    else data[byte] &= uint8_t(~bitMask); // else clear the data bit 


  }
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
// - bits 0..31  (bytes 0..3): float32 setpoint (dutyCycle)
// - bits 32..47  (bytes 4..5): int16 feed-forward
// - bits 48..49  (byte 6, bits 0..1): PID slot
// - bit 50   (byte 6, bit 2):  Feed Forward units 
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