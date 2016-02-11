#include "can.h"

constexpr int PinAdr0 = 14; // A0
constexpr int PinAdr1 = 15; // A1
constexpr int PinAdr2 = 16; // A2
constexpr int PinAdr3 = 17; // A3

// D3, D4 data in
// D5, D6, D7 multiplex selector D5 MSB (C) D7 LSB (A)

constexpr int MultiplexInputA = 3; // D3
constexpr int MultiplexInputB = 4; // D4

constexpr int MultiplexSelectA = 7; // D7 LSB
constexpr int MultiplexSelectB = 6; // D6
constexpr int MultiplexSelectC = 5; // D5 MSB

constexpr uint8_t pinRemapping[16] = {0, 1, 2, 3, 4, 5, 6, 7, 11, 10, 9, 8, 15, 14, 13, 12};

uint32_t timestamps[16] = {0};
uint16_t inputStates = 0; // bit field: input pin X is high
uint32_t debounceIn = 10; // time in milliseconds before input edge H/L is detected
uint32_t debounceOut = 100; // time in milliseconds before an input edge L/H after an edge H/L is detected

void setup()
{
  uint16_t address = 0x3E0;
//  address |= digitalRead(PinAdr0)? 0x10 : 0x00;
//  address |= digitalRead(PinAdr1)? 0x20 : 0x00;
//  address |= digitalRead(PinAdr2)? 0x40 : 0x00;
//  address |= digitalRead(PinAdr3)? 0x80 : 0x00;
  uint16_t mask = 0x7F0; //take 16 addresses

  CAN::inst().start(address, mask);
  CAN::inst().setMsgHandler(&msgHandler);
  CAN::inst().setRTRHandler(&rtrHandler);
  CAN::inst().setErrorHandler(&errorHandler);

  pinMode(MultiplexInputA, INPUT);
  pinMode(MultiplexInputB, INPUT);
  pinMode(MultiplexSelectA, OUTPUT);
  pinMode(MultiplexSelectB, OUTPUT);
  pinMode(MultiplexSelectC, OUTPUT);
  
  Serial.begin(9600);
}

uint8_t addr = 0;

uint8_t inputAddr = 0;

void loop()
{
//  CAN::inst().send(addr, millis(), 108);
//  addr = (addr + 1) % 16;
//
//  int ts = millis();
//  while(millis() - ts < 1000)
//    CAN::inst().processEvents();

for(inputAddr = 0; inputAddr < 16; inputAddr++)
{
  digitalWrite(MultiplexSelectA, inputAddr & 0b001 ? HIGH : LOW);
  digitalWrite(MultiplexSelectB, inputAddr & 0b010 ? HIGH : LOW);
  digitalWrite(MultiplexSelectC, inputAddr & 0b100 ? HIGH : LOW);

  int pinToRead = inputAddr < 8 ? MultiplexInputA : MultiplexInputB;
  uint16_t inputAddrMask = 1 << inputAddr;

  uint32_t now = millis();

  if(digitalRead(pinToRead) == LOW)
  {
    if((inputStates & inputAddrMask) == 0 && ((now - timestamps[inputAddr]) > debounceOut))
    {
      Serial.print(pinRemapping[inputAddr]);
      Serial.print(" ");
      Serial.print(now);
      Serial.print(" ");
      Serial.println(0);
      
      timestamps[inputAddr] = now;
      inputStates |= inputAddrMask;
    }
  }
  else
  {
    if((inputStates & inputAddrMask) > 0)
    {
      if((now - timestamps[inputAddr]) > debounceIn)
      {
        Serial.print(pinRemapping[inputAddr]);
        Serial.print(" ");
        uint32_t duration = now - timestamps[inputAddr];
        Serial.print(timestamps[inputAddr]);
        Serial.print(" ");
        Serial.println(duration);
        inputStates &= ~inputAddrMask;  

        // Send message with timestamps[inputAddr] and duration

        // reset timestamp for debounceOut
        timestamps[inputAddr] = now;
      }
    }
  }
 
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


