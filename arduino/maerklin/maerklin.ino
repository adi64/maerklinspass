#include "can.h"
#include "motorola.h"

const uint8_t trainAddressMap[] = {Motorola::IdleAddress, 1, 3, 65};
const uint8_t trainAddressCount = 4;
const uint8_t trainIdleAddressIndex = 0;
uint8_t trainTargetSpeedMap[] = {0, 8, 8, 6};

constexpr uint8_t switchMsgSlot = 7;

constexpr uint8_t switchArrayAddress[] = {1, 3};
constexpr uint8_t switchArrayStateMap[] = {15, 12, 3}; // straight, in->out, out->in

uint8_t sectionOccupants[4] = {0, 1, 2, 3}; // preloaded with initial state
uint8_t switchArrayOccupants[2][2] = {0}; // 2 switchArrays, max. 2 trains waiting per array

volatile bool switchArrayResetNeeded[2] = {false}; // SA is free but needs to be reset
volatile bool switchArrayBusy[2] = {false}; // train is currently passing

// Serial parsing foo
int incomingSerialByte;
int serialBytes[3] = {0};
int parsedSerialBytes[3] = {-1};

void setSwitchArray(uint8_t decoderAddress, uint8_t states) //first 4 bits: 0 = straight, 1 = diverging
{
  unsigned long ts;
  Motorola::setMessageSpeed(switchMsgSlot, true);
  Motorola::setMessageOneShot(switchMsgSlot, true);
  for(int i = 0; i < 4; ++i)
  {
    ts = millis();
    Motorola::setMessage(switchMsgSlot, Motorola::switchMessage(decoderAddress, 2 * i + ((states & (0x1 << i))? 1 : 0), true));
    Motorola::enableMessage(switchMsgSlot);
    while((millis()-ts) < 150 || Motorola::messageEnabled(switchMsgSlot));
    ts = millis();
    Motorola::setMessage(switchMsgSlot, Motorola::switchMessage(decoderAddress, 2 * i + ((states & (0x1 << i))? 1 : 0), false));
    Motorola::enableMessage(switchMsgSlot);
    while((millis()-ts) < 50 || Motorola::messageEnabled(switchMsgSlot));
  }
}

void handleSwitchArrayEvent(uint8_t section, bool entering)
{
  if(entering) // train is entering switch array
  {
    uint8_t trainNo = sectionOccupants[section];
    if(trainNo == 0 || trainNo > trainAddressCount)
    {
      Serial.print("### ERROR: There shouldn't be a train on section ");
      Serial.println(section);
      return;
    }

    // stop train
    Motorola::setMessage(trainNo, Motorola::oldTrainMessage(trainAddressMap[trainNo], true, 0));

    // Put train in switch array waiting list
    uint8_t switchArrayNo = (section & 0b10) ? 1 : 0; // section 0,1 -> SA0; section 2,3 -> SA1
    if(switchArrayOccupants[switchArrayNo][0] == 0)
    {
      switchArrayOccupants[switchArrayNo][0] = trainNo;
    }
    else if(switchArrayOccupants[switchArrayNo][1] == 0)
    {
      switchArrayOccupants[switchArrayNo][1] = trainNo;
    }
    else
    {
      Serial.print("### ERROR: Switch Array ");
      Serial.print(switchArrayNo);
      Serial.println(" is already occupied on both tracks");
      return;
    }
  }
  else // train is leaving switch array
  {
    uint8_t switchArrayNo = (section & 0b10) ? 0 : 1; // SA0 -> section 2,3; SA1 -> section 0,1
    uint8_t trainNo = switchArrayOccupants[switchArrayNo][0]; // first train in SA queue

    // sanity check train number
    if(trainNo == 0 || trainNo > trainAddressCount)
    {
      Serial.print("### ERROR: Switch array ");
      Serial.print(switchArrayNo);
      Serial.println(" was not occupied!");
      return;
    }

    // sanity check section occupants
    if(sectionOccupants[section] != 0)
    {
      Serial.print("### ERROR: Section ");
      Serial.print(section);
      Serial.print(" is already occupied by train ");
      Serial.println(sectionOccupants[section]);
      return;
    }

    // remove old section occupancy
    for(uint8_t i=0; i<4; i++)
    {
      if(sectionOccupants[i] == trainNo)
      {
        sectionOccupants[i] = 0;
        break;
      }
    }

    // sanity check switch array business
    if(!switchArrayBusy[switchArrayNo])
    {
      Serial.print("### ERROR: Switch array ");
      Serial.print(switchArrayNo);
      Serial.println(" was not marked busy but a train just left it!");
      return;
    }

    // sanity check switch array reset requests
    if(switchArrayResetNeeded[switchArrayNo])
    {
      Serial.print("### ERROR: Switch array ");
      Serial.print(switchArrayNo);
      Serial.println(" still needs to be reset but a train just left it!");
      return;
    }

    // request a reset of switch array
    switchArrayResetNeeded[switchArrayNo] = true;

    // sanity check switch array queue
    if(switchArrayOccupants[switchArrayNo][0] != trainNo)
    {
      Serial.print("### ERROR: Train ");
      Serial.print(trainNo);
      Serial.print(" was not first in queue of switch array ");
      Serial.println(switchArrayNo);
      return;
    }

    // cycle switch array queue
    switchArrayOccupants[switchArrayNo][0] = switchArrayOccupants[switchArrayNo][1];

    // mark new section as occupied
    sectionOccupants[section] = trainNo;
  }
}

void msgHandler(const CAN::MessageEvent * message)
{
  //Motorola::setMessage(0, Motorola::oldTrainMessage(trainAddressMap[0], true, 0));
  //Motorola::setMessage(1, Motorola::oldTrainMessage(trainAddressMap[1], true, 0));
  //Motorola::setMessage(2, Motorola::oldTrainMessage(trainAddressMap[2], true, 0));
  //Motorola::setMessage(3, Motorola::oldTrainMessage(trainAddressMap[3], true, 0));

  CAN::StdIdentifier contactAddr = message->stdIdentifier;
  // uint32_t timestamp =
  // uint32_t duration =
  Serial.print("Msg from 0x");Serial.println(contactAddr, HEX);
  // Serial.print(": "); Serial.print(timestamp); Serial.print(" - "); Serial.println(duration);

  if((contactAddr & 0x1) != 0)
    return;

  // if(duration != 0)
  //   return;

  uint8_t section;
  bool entering;

  switch(contactAddr & 0xE)
  {
    case 0x0: // SA0, outer entering (from 0 to SA0 to 2,3)
      section = 0;
      entering = true;
      break;
    case 0x2: // SA0, inner entering (from 1 to SA0 to 3,2)
      section = 1;
      entering = true;
      break;
    case 0x4: // SA0, inner leaving (from SA0 to 3)
      section = 3;
      entering = false;
      break;
    case 0x6: // SA0, outer leaving (from SA0 to 2)
      section = 2;
      entering = false;
      break;
    case 0x8: // SA1, outer entering (from 2 to SA1 to 0,1)
      section = 2;
      entering = true;
      break;
    case 0xA: // SA1, inner entering (from 3 to SA1 to 1,0)
      section = 3;
      entering = true;
      break;
    case 0xC: // SA1, inner leaving (from SA1 to 1)
      section = 1;
      entering = false;
      break;
    case 0xE: // SA1, outer leaving (from SA1 to 0)
      section = 0;
      entering = false;
      break;
  }

  handleSwitchArrayEvent(section, entering);
}

void operateSwitchArrays()
{

    for(uint8_t switchArrayNo = 0; switchArrayNo < 2; switchArrayNo++)
    {
      if(switchArrayBusy[switchArrayNo])
        continue;

      if(switchArrayResetNeeded[switchArrayNo])
      {
        setSwitchArray(switchArrayAddress[switchArrayNo], switchArrayStateMap[0]);
        switchArrayResetNeeded[switchArrayNo] = false;
        continue;
      }

      uint8_t nextTrainNo = switchArrayOccupants[switchArrayNo][0];
      if(nextTrainNo != 0) // is there a train waiting?
      {
        // find a free section, if any
        uint8_t nextSectionBase = switchArrayNo ? 2 : 0;
        uint8_t freeSection;

        if(sectionOccupants[nextSectionBase] == 0)
        {
          // outer ring is free
          freeSection = nextSectionBase;
        }
        else if(sectionOccupants[nextSectionBase+1] == 0)
        {
          // inner ring is free
          freeSection = nextSectionBase+1;
        }
        else
        {
          // both tracks are occupied
          continue;
        }

        // find out on which section the waiting train is standing on
        uint8_t currentSectionBase = switchArrayNo ? 0 : 2;
        uint8_t currentSection;
        if(sectionOccupants[currentSectionBase] == nextTrainNo)
        {
          currentSection = currentSectionBase;
        }
        else if(sectionOccupants[currentSectionBase+1] == nextTrainNo)
        {
          currentSection = currentSectionBase+1;
        }
        else
        {
            Serial.print("### ERROR: Train ");
            Serial.print(nextTrainNo);
            Serial.print(" is waiting on switch array ");
            Serial.print(switchArrayNo);
            Serial.print(" but is neither on section ");
            Serial.print(currentSectionBase);
            Serial.print(" nor ");
            Serial.println(currentSectionBase+1);
            return;
        }

        // set switch array busy
        switchArrayBusy[switchArrayNo] = true;

        // operate switch array, if neccessary
        if((currentSection & 0x1) != (freeSection & 0x1))
        {
          // train needs to switch tracks
          if(currentSection & 0x1)
          {
            // switch in->out
            setSwitchArray(switchArrayAddress[switchArrayNo], switchArrayStateMap[1]);
          }
          else
          {
            // switch out->in
            setSwitchArray(switchArrayAddress[switchArrayNo], switchArrayStateMap[2]);
          }
          // note that no action has to be taken for in->in or out->out
          // since the switch array is always reset to this state
        }

        // start train
        Motorola::setMessage(nextTrainNo, Motorola::oldTrainMessage(trainAddressMap[(uint8_t)nextTrainNo], true, (uint8_t)8));
      }

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
      Motorola::setMessage(trainNo, Motorola::oldTrainMessage(trainAddressMap[(uint8_t)trainNo], true, (uint8_t)speed));
    }
  }
  else if(serialBytes[0] == 'W') // switch
  {
    long swaAddr = parsedSerialBytes[1];
    long state = parsedSerialBytes[2];
    Serial.print("Weiche "); Serial.print(swaAddr); Serial.print(": "); Serial.println(state);

    if(0 <= state && state < 3)
    {
      setSwitchArray(switchArrayAddress[swaAddr], switchArrayStateMap[state]);
    }
  }
}

void setup() {
  Motorola::start();

  for(uint8_t i = 0; i < trainAddressCount; ++i)
  {
    // Don't create a dedicated message slot for the idle message
    if(i == trainIdleAddressIndex)
      continue;

    Motorola::setMessage(i, Motorola::oldTrainMessage(trainAddressMap[i], true, 0));
    Motorola::setMessageSpeed(i, false);
    Motorola::setMessageOneShot(i, false);
    Motorola::enableMessage(i);
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
  operateSwitchArrays();
}
