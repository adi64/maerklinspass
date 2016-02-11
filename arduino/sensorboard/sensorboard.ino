#include "can.h"

constexpr int PinAdr0 = 14; // A0
constexpr int PinAdr1 = 15; // A1
constexpr int PinAdr2 = 16; // A2
constexpr int PinAdr3 = 17; // A3

void setup()
{
  uint16_t address = 0x3E0;
//  address |= digitalRead(PinAdr0)? 0x10 : 0x00;
//  address |= digitalRead(PinAdr1)? 0x20 : 0x00;
//  address |= digitalRead(PinAdr2)? 0x40 : 0x00;
//  address |= digitalRead(PinAdr3)? 0x80 : 0x00;
  uint16_t mask = 0x7F0; //take 16 addresses

  CAN::start(address, mask);
  CAN::setMsgHandler(&msgHandler);
  CAN::setRTRHandler(&rtrHandler);
  CAN::setErrorHandler(&errorHandler);
  
  Serial.begin(9600);
}

uint8_t addr = 0;

void loop()
{
  CAN::send(addr, millis(), 108);
  addr = (addr + 1) % 16;

  uint32_t ts = millis(); //TODO handle millis() overflow between ts and now
  while(millis() - ts < 5000)
  {
    CAN::processEvents();
    delay(1000);
  }
}

void msgHandler(CAN::CANAddress address, uint32_t timestamp, uint32_t duration)
{
  Serial.print("Msg from "); Serial.print(address, HEX);
  Serial.print(": "); Serial.print(timestamp); Serial.print(" - "); Serial.println(duration);
}

void rtrHandler(CAN::CANAddress address)
{
  Serial.print("RTR from: "); Serial.println(address, HEX);
}

void errorHandler(byte flags)
{
  Serial.print("ERROR: "); Serial.println(flags, HEX);
}


