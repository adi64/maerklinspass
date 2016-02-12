#include "can.h"
#include "motorola.h"

Motorola motorola;

const uint8_t address[] = {Motorola::IdleAddress, 1, 3, 65};
const uint8_t addressCount = 3;

constexpr uint8_t SwitchMsgSlot = 7;

constexpr uint8_t switchArrayAddress[] = {1, 3};
constexpr uint8_t switchArrayState[] = {15, 12, 3};

constexpr uint8_t sectionOccupants[4] = {0, 1, 2, 3};

ISR(TIMER1_OVF_vect)
{
  motorola.onTimerOverflow();
}

void setSwitchArray(uint8_t decoderAddress, uint8_t states) //first 4 bits: 0 = straight, 1 = diverging
{
  unsigned long ts;
  motorola.setMessageSpeed(SwitchMsgSlot, true);
  motorola.setMessageOneShot(SwitchMsgSlot, true);
  for(int i = 0; i < 4; ++i)
  {
    ts = millis();
    motorola.setMessage(SwitchMsgSlot, Motorola::switchMessage(decoderAddress, 2 * i + ((states & (0x1 << i))? 1 : 0), true));
    motorola.enableMessage(SwitchMsgSlot);
    while((millis()-ts) < 150 || motorola.messageEnabled(SwitchMsgSlot));
    ts = millis();
    motorola.setMessage(SwitchMsgSlot, Motorola::switchMessage(decoderAddress, 2 * i + ((states & (0x1 << i))? 1 : 0), false));
    motorola.enableMessage(SwitchMsgSlot);
    while((millis()-ts) < 50 || motorola.messageEnabled(SwitchMsgSlot));
  }
}


int incomingSerialByte;
int serialBytes[3];
int parsedSerialBytes[3];

void setup() {
  motorola.start(); 

  for(uint8_t i = 0; i < addressCount; ++i)
  {
    motorola.setMessage(i, Motorola::oldTrainMessage(address[i], true, 0));
    motorola.setMessageSpeed(i, false);
    motorola.setMessageOneShot(i, false);
    motorola.enableMessage(i);
  }
  
  uint16_t CANaddress = 0x300;
  uint16_t CANmask = 0x700;
  CAN::start(CANaddress, CANmask);
  CAN::setMsgHandler(&msgHandler);
  CAN::setErrorHandler(&errorHandler);

  Serial.begin(9600);
  Serial.setTimeout(60000);
}

void parseSerialInput()
{
  incomingSerialByte = Serial.read();
  if(incomingSerialByte == -1)
    return;

  serialBytes[0] = serialBytes[1];
  serialBytes[1] = serialBytes[2];
  serialBytes[2] = incomingSerialByte;

  for(int i=1; i<3; i++)
  {
    if(serialBytes[i] >= '0' && serialBytes[i] <= '9')
      parsedSerialBytes[i] = serialBytes[i] - '0';
    else if(serialBytes[i] >= 'A' && serialBytes[i] <= 'F')
      parsedSerialBytes[i] = serialBytes[i] - 'A' + 10;
    else
      parsedSerialBytes[i] = -1;
  }

  if(parsedSerialBytes[1] == -1 || parsedSerialBytes[2] == -1)
    return;

  if(serialBytes[0] == 'L')
  {
    long trainNo = parsedSerialBytes[1];
    long speed = parsedSerialBytes[2];
    Serial.print("Zug "); Serial.print(trainNo); Serial.print(": "); Serial.println(speed);
    
    if(0 <= trainNo && trainNo < addressCount &&
       0 <= speed && speed < 16)
    {
      motorola.setMessage(trainNo, Motorola::oldTrainMessage(address[(uint8_t)trainNo], true, (uint8_t)speed));
    }
  }
  else if(serialBytes[0] == 'W')
  {
    long swaAddr = parsedSerialBytes[1];
    long state = parsedSerialBytes[2];
    Serial.print("Weiche "); Serial.print(swaAddr); Serial.print(": "); Serial.println(state);
    
    if(0 <= state && state < 3)
    {
      setSwitchArray(switchArrayAddress[swaAddr], switchArrayState[state]);
    }    
  }
}


void loop() {

  CAN::processEvents();
  parseSerialInput();
  
}

void msgHandler(CAN::CANAddress CANaddress, uint32_t timestamp, uint32_t duration)
{
  motorola.setMessage(0, Motorola::oldTrainMessage(address[0], true, 0));
  motorola.setMessage(1, Motorola::oldTrainMessage(address[1], true, 0));
  motorola.setMessage(2, Motorola::oldTrainMessage(address[2], true, 0));

  Serial.print("Msg from "); Serial.print(CANaddress, HEX);
  Serial.print(": "); Serial.print(timestamp); Serial.print(" - "); Serial.println(duration);

  if(CANaddress & 0x1 != 0)
    return;

  if(duration != 0)
    return;

  switch(CANaddress & 0xE)
  {
    case 0x0: // SA0, outer entering
      break;
    case 0x2: // SA0, inner entering
      break;
    case 0x4: // SA0, inner leaving
      break;
    case 0x6: // SA0, outer leaving
      break;
    case 0x8: // SA1, outer entering
      break;
    case 0xA: // SA1, inner entering
      break;
    case 0xC: // SA1, inner leaving
      break;
    case 0xE: // SA1, outer leaving
      break;
  }
  
}

void errorHandler(byte flags)
{
  Serial.print("ERROR: "); Serial.println(flags, HEX);
}

