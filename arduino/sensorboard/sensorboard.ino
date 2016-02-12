#include "can.h"

constexpr int PinAdr0 = 14; // A0
constexpr int PinAdr1 = 15; // A1
constexpr int PinAdr2 = 16; // A2
constexpr int PinAdr3 = 17; // A3

constexpr int MultiplexInputA = 3; // D3
constexpr int MultiplexInputB = 4; // D4

constexpr int MultiplexSelectA = 7; // D7 LSB
constexpr int MultiplexSelectB = 6; // D6
constexpr int MultiplexSelectC = 5; // D5 MSB

// remapping to get consistent input pin numbering
constexpr uint8_t pinRemapping[16] = {0, 1, 2, 3, 4, 5, 6, 7, 11, 10, 9, 8, 15, 14, 13, 12};

uint32_t timestamps[16] = {0}; // last start of track signal
uint32_t durations[16] = {0}; // last duration of track signal
uint16_t inputStates = 0; // bit field: input pin X is currently high
uint32_t debounceIn = 10; // time in milliseconds before input edge H/L is detected
uint32_t debounceOut = 100; // time in milliseconds before an input edge L/H after an edge H/L is detected

void setup()
{
  uint16_t address = 0x300;
  address |= digitalRead(PinAdr0)? 0x10 : 0x00;
  address |= digitalRead(PinAdr1)? 0x20 : 0x00;
  address |= digitalRead(PinAdr2)? 0x40 : 0x00;
  address |= digitalRead(PinAdr3)? 0x80 : 0x00;
  uint16_t mask = 0x7F0; //take 16 addresses

  CAN::start(address, mask);
  CAN::setMsgHandler(&msgHandler);
  CAN::setRTRHandler(&rtrHandler);
  CAN::setErrorHandler(&errorHandler);

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

  for(inputAddr = 0; inputAddr < 16; inputAddr++)
  {
    digitalWrite(MultiplexSelectA, inputAddr & 0b001 ? HIGH : LOW);
    digitalWrite(MultiplexSelectB, inputAddr & 0b010 ? HIGH : LOW);
    digitalWrite(MultiplexSelectC, inputAddr & 0b100 ? HIGH : LOW);

    // the multiplexers are really quick -
    // the Arduino is slow enough that we don't have to delay reading the data
  
    int pinToRead = inputAddr < 8 ? MultiplexInputA : MultiplexInputB;
    uint16_t inputAddrMask = 1 << inputAddr;
  
    uint32_t now = millis();
  
    if(digitalRead(pinToRead) == LOW) // inverting input logic
    {
      if((inputStates & inputAddrMask) == 0 && ((now - (timestamps[inputAddr] + durations[inputAddr])) > debounceOut))
      {
        Serial.print(pinRemapping[inputAddr]);
        Serial.print(" ");
        Serial.print(now);
        Serial.print(" ");
        Serial.println(0);
              
        timestamps[inputAddr] = now;
        inputStates |= inputAddrMask;
  
        CAN::send(inputAddr, timestamps[inputAddr], 0);
      }
    }
    else
    {
      if((inputStates & inputAddrMask) > 0 && ((now - timestamps[inputAddr]) > debounceIn))
      {
        Serial.print(pinRemapping[inputAddr]);
        Serial.print(" ");
        durations[inputAddr] = now - timestamps[inputAddr];
        Serial.print(timestamps[inputAddr]);
        Serial.print(" ");
        Serial.println(durations[inputAddr]);
        
        inputStates &= ~inputAddrMask;  
  
        // Send message with timestamp and duration
        CAN::send(inputAddr, timestamps[inputAddr], durations[inputAddr]);
      }
    }
  
    CAN::processEvents();
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
  uint8_t inputAddr = (uint8_t)(address & 0xF);
  CAN::send(inputAddr, timestamps[inputAddr], durations[inputAddr]);
}

void errorHandler(byte flags)
{
  Serial.print("ERROR: "); Serial.println(flags, HEX);
}


