#include "can.h"
#include "motorola.h"

Motorola motorola;

const uint8_t trainAddress[] = {Motorola::IdleAddress, 1, 3, 65};
const uint8_t trainAddressCount = 4;
const uint8_t trainIdleAddressIndex = 0;

constexpr uint8_t switchMsgSlot = 7;

constexpr uint8_t switchArrayAddress[] = {1, 3};
constexpr uint8_t switchArrayState[] = {15, 12, 3};

constexpr uint8_t sectionOccupants[4] = {0, 1, 2, 3};

int incomingSerialByte;
int serialBytes[3];
int parsedSerialBytes[3];

ISR(TIMER1_OVF_vect)
{
  motorola.onTimerOverflow();
}

void setSwitchArray(uint8_t decoderAddress, uint8_t states) //first 4 bits: 0 = straight, 1 = diverging
{
  unsigned long ts;
  motorola.setMessageSpeed(switchMsgSlot, true);
  motorola.setMessageOneShot(switchMsgSlot, true);
  for(int i = 0; i < 4; ++i)
  {
    ts = millis();
    motorola.setMessage(switchMsgSlot, Motorola::switchMessage(decoderAddress, 2 * i + ((states & (0x1 << i))? 1 : 0), true));
    motorola.enableMessage(switchMsgSlot);
    while((millis()-ts) < 150 || motorola.messageEnabled(switchMsgSlot));
    ts = millis();
    motorola.setMessage(switchMsgSlot, Motorola::switchMessage(decoderAddress, 2 * i + ((states & (0x1 << i))? 1 : 0), false));
    motorola.enableMessage(switchMsgSlot);
    while((millis()-ts) < 50 || motorola.messageEnabled(switchMsgSlot));
  }
}

void msgHandler(const CAN::MessageEvent * message)
{
  motorola.setMessage(0, Motorola::oldTrainMessage(trainAddress[0], true, 0));
  motorola.setMessage(1, Motorola::oldTrainMessage(trainAddress[1], true, 0));
  motorola.setMessage(2, Motorola::oldTrainMessage(trainAddress[2], true, 0));
  motorola.setMessage(3, Motorola::oldTrainMessage(trainAddress[3], true, 0));

  CAN::StdIdentifier contactAddr = message->stdIdentifier;
  // uint32_t timestamp =
  // uint32_t duration =
  Serial.print("Msg from 0x");Serial.println(contactAddr, HEX);
  // Serial.print(": "); Serial.print(timestamp); Serial.print(" - "); Serial.println(duration);

  if((contactAddr & 0x1) != 0)
    return;

  // if(duration != 0)
  //   return;

  switch(contactAddr & 0xE)
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

void errorHandler(const CAN::ErrorEvent * error)
{
  Serial.print("ERROR: 0x"); Serial.println(error->flags, HEX);
}

void parseSerialInput()
{
  // Read 1 serial byte
  incomingSerialByte = Serial.read();
  if(incomingSerialByte == -1)
    return;

  serialBytes[0] = serialBytes[1];
  serialBytes[1] = serialBytes[2];
  serialBytes[2] = incomingSerialByte;

  // check that every byte is in [0-9A-F]
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

  if(serialBytes[0] == 'L') // locomotive
  {
    long trainNo = parsedSerialBytes[1];
    long speed = parsedSerialBytes[2];

    // Don't actively set speed for Motorola::IdleAddress
    if(trainNo == trainIdleAddressIndex)
      return;

    Serial.print("Zug "); Serial.print(trainNo); Serial.print(": "); Serial.println(speed);

    if(0 <= trainNo && trainNo < trainAddressCount &&
       0 <= speed && speed < 16)
    {
      motorola.setMessage(trainNo, Motorola::oldTrainMessage(trainAddress[(uint8_t)trainNo], true, (uint8_t)speed));
    }
  }
  else if(serialBytes[0] == 'W') // switch
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

void setup() {
  motorola.start();

  for(uint8_t i = 0; i < trainAddressCount; ++i)
  {
    // Don't create a dedicated message slot for the idle message
    if(i == trainIdleAddressIndex)
      continue;

    motorola.setMessage(i, Motorola::oldTrainMessage(trainAddress[i], true, 0));
    motorola.setMessageSpeed(i, false);
    motorola.setMessageOneShot(i, false);
    motorola.enableMessage(i);
  }

  CAN::StdIdentifier address = 0x300;
  CAN::StdIdentifier mask = 0x700;
  CAN::start(&msgHandler, &errorHandler);
  CAN::setReceiveFilter(address, mask);

  Serial.begin(9600);
  Serial.setTimeout(60000);
}

void loop() {
  parseSerialInput();
}
