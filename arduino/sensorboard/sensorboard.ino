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
constexpr uint8_t pinToContactMap[16] = {0, 1, 2, 3, 4, 5, 6, 7, 11, 10, 9, 8, 15, 14, 13, 12};

uint32_t timestamps[16] = {0}; // last start of track signal
uint32_t durations[16] = {0}; // last duration of track signal
uint16_t inputStates = 0; // bit field: input pin X is currently high
uint32_t debounceIn = 10; // time in milliseconds before input edge H/L is detected
uint32_t debounceOut = 100; // time in milliseconds before an input edge L/H after an edge H/L is detected

CAN::StdIdentifier canAddress = 0x300;
CAN::StdIdentifier canAddressMask = 0x7F0; //take 16 addresses

void encodeLong(const uint32_t & value, uint8_t * buffer)
{
  buffer[0] = (uint8_t) (value & 0x000000FF);
  buffer[1] = (uint8_t)((value & 0x0000FF00) >>  8);
  buffer[2] = (uint8_t)((value & 0x0000FF00) >> 16);
  buffer[3] = (uint8_t)((value & 0x0000FF00) >> 24);
}

void send(uint8_t pin, uint32_t timestamp, uint32_t duration)
{
  CAN::MessageEvent * msg = CAN::prepareMessage();
  if(!msg)
    return;
  msg->hasExtIdentifier = false;
  msg->stdIdentifier = (canAddress & canAddressMask) |
                       (pin & ~canAddressMask);
  msg->isRTR = false;
  msg->length = 8;
  encodeLong(timestamp, msg->content);
  encodeLong(duration, msg->content + 4);
  CAN::commitMessage(msg);
}

void msgHandler(const CAN::MessageEvent * msg)
{
  if(msg->isRTR)
  {
    Serial.print("Request for 0x"); Serial.print(msg->stdIdentifier, HEX);
  }
  uint8_t contactNumber = msg->stdIdentifier & 0x00F;
  send(contactNumber, timestamps[contactNumber], durations[contactNumber]);
}

void errorHandler(const CAN::ErrorEvent * error)
{
  Serial.print("Error 0x"); Serial.println(error->flags, HEX);
}

void setup()
{
  canAddress |= digitalRead(PinAdr0)? 0x10 : 0x00;
  canAddress |= digitalRead(PinAdr1)? 0x20 : 0x00;
  canAddress |= digitalRead(PinAdr2)? 0x40 : 0x00;
  canAddress |= digitalRead(PinAdr3)? 0x80 : 0x00;

  CAN::start(&msgHandler, &errorHandler);
  CAN::setReceiveFilter(canAddress, canAddressMask);

  pinMode(MultiplexInputA, INPUT);
  pinMode(MultiplexInputB, INPUT);
  pinMode(MultiplexSelectA, OUTPUT);
  pinMode(MultiplexSelectB, OUTPUT);
  pinMode(MultiplexSelectC, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  for(uint8_t pinNumber = 0; pinNumber < 16; ++pinNumber)
  {
    digitalWrite(MultiplexSelectA, (pinNumber & 0b0001)? HIGH : LOW);
    digitalWrite(MultiplexSelectB, (pinNumber & 0b0010)? HIGH : LOW);
    digitalWrite(MultiplexSelectC, (pinNumber & 0b0100)? HIGH : LOW);
    int inputPin = (pinNumber & 0b1000)? MultiplexInputB : MultiplexInputA;
    uint8_t contactNumber = pinToContactMap[pinNumber];
    uint16_t contactMask = 1 << contactNumber;

    // the multiplexers are really quick -
    // the Arduino is slow enough that we don't have to delay reading the data

    uint32_t now = millis();
    if(digitalRead(inputPin) == LOW) // inverting input logic
    {
      uint32_t lastFallingEdge = timestamps[contactNumber] + durations[contactNumber];
      uint32_t timeSinceLastFallingEdge = now - lastFallingEdge;

      // handle timer overflow
      if((inputStates & contactMask) == 0 && (now < lastFallingEdge))
      {
        timeSinceLastFallingEdge = UINT32_MAX - lastFallingEdge + now;
      }

      if((inputStates & contactMask) == 0 && (timeSinceLastFallingEdge > debounceOut))
      {
        Serial.print(pinNumber);
        Serial.print(">");
        Serial.print(contactNumber);
        Serial.print(" ");
        Serial.print(now);
        Serial.print(" ");
        Serial.println(0);

        timestamps[contactNumber] = now;
        inputStates |= contactMask;

        send(contactNumber, timestamps[contactNumber], 0);
      }
    }
    else
    {
      uint32_t timeSinceLastRisingEdge = now - timestamps[contactNumber];

      // handle timer overflow
      if(now < timestamps[contactNumber])
      {
        timeSinceLastRisingEdge = UINT32_MAX - timestamps[contactNumber] + now;
      }

      if((inputStates & contactMask) > 0 && (timeSinceLastRisingEdge > debounceIn))
      {
        durations[contactNumber] = timeSinceLastRisingEdge;

        Serial.print(pinNumber);
        Serial.print(">");
        Serial.print(contactNumber);
        Serial.print(" ");
        Serial.print(timestamps[contactNumber]);
        Serial.print(" ");
        Serial.println(durations[contactNumber]);

        inputStates &= ~contactMask;

        // Send message with timestamp and duration
        send(contactNumber, timestamps[contactNumber], durations[contactNumber]);
      }
    }
  }
}


