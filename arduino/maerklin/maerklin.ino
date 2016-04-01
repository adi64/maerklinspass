#include "can.h"
#include "motorola.h"

/*
Train Controller Demo
^^^^^^^^^^^^^^^^^^^^^

This program controls the movement of 3 trains on 4 track segments (see track layout below)
ensuring that at most one train is on a particular segment.
The current position of a train is tracked by monitoring the segment borders.
Each passing train generates an event on the CAN bus. The event ID for each segment border
is marked on the track layout next to the respective parenthesis in hexadecimal format. (e.g. 3F8)

Track segments can only be crossed in one direction; all trains move counter-clockwise.
There are 2 switch arrays (SA0 and SA1) to connect the track segments.
Both trains and track segments have numbers starting at 0 (each physical train
or track segment has a label with its number attached) and after a controller reset
each train is expected to occupy the track segment of the same number.

track layout:
      ____<T2_____
     /  __<T3___  \
    |  /        \  |
 3F8) |          | )3F6
    | )3FA    3F4) |
    |\|          |/|
    | |SA1    SA0| |
    |/|          |\|
    | )3FC    3F2) |
 3FE) |          | )3F0
    |  \___T1>__/  |
     \_____T0>____/
--------------------



Authors: Adrian Holfter, Lukas Wenzel
*/


const uint8_t trainAddressMap[] = {Motorola::IdleAddress, 1, 3, 65};
const uint8_t trainAddressCount = 4;
const uint8_t trainIdleAddressIndex = 0;
uint8_t trainTargetSpeedMap[] = {0, 8, 6, 8};

constexpr uint8_t switchMsgSlot = 7;

constexpr uint8_t switchArrayAddress[] = {1, 3};
constexpr uint8_t switchArrayStateMap[] = {15, 3, 12}; // straight, in->out, out->in

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

uint32_t decodeLong(const uint8_t * encoded)
{
  return ((uint32_t)encoded[0]) |
         ((uint32_t)encoded[1]) <<  8 |
         ((uint32_t)encoded[2]) << 16 |
         ((uint32_t)encoded[3]) << 24;
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

    // Put train in switch array waiting list
    uint8_t switchArrayNo = (section & 0b10) ? 1 : 0; // section 0,1 -> SA0; section 2,3 -> SA1

    Serial.print("Train ");
    Serial.print(trainNo);
    Serial.print(" is entering SA");
    Serial.println(switchArrayNo);

    Serial.print("Section occupance: ");
    for(uint8_t sec=0; sec<4; sec++)
    {
      Serial.print(sectionOccupants[sec]);
      Serial.print(" ");
    }
    Serial.println("");

    if(switchArrayOccupants[switchArrayNo][0] == trainNo || switchArrayOccupants[switchArrayNo][1] == trainNo)
    {
      Serial.print("### ERROR: Train ");
      Serial.print(trainNo);
      Serial.print(" is already in queue for SA");
      Serial.println(switchArrayNo);
      return;
    }

    // stop train
    Motorola::setMessage(trainNo, Motorola::oldTrainMessage(trainAddressMap[trainNo], true, 0));

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

    Serial.print("SA");
    Serial.print(switchArrayNo);
    Serial.print(" queue: ");
    Serial.print(switchArrayOccupants[switchArrayNo][0]); Serial.print(" ");
    Serial.println(switchArrayOccupants[switchArrayNo][1]);

  }
  else // train is leaving switch array
  {
    uint8_t switchArrayNo = (section & 0b10) ? 0 : 1; // SA0 -> section 2,3; SA1 -> section 0,1
    uint8_t trainNo = switchArrayOccupants[switchArrayNo][0]; // first train in SA queue

    Serial.print("Train ");
    Serial.print(trainNo);
    Serial.print(" is leaving SA");
    Serial.println(switchArrayNo);

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

    switchArrayBusy[switchArrayNo] = false;

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

    Serial.print("schedule restart of SA");
    Serial.println(switchArrayNo);

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
    switchArrayOccupants[switchArrayNo][1] = 0;

    Serial.print("SA");
    Serial.print(switchArrayNo);
    Serial.print(" queue: ");
    Serial.print(switchArrayOccupants[switchArrayNo][0]); Serial.print(" ");
    Serial.println(switchArrayOccupants[switchArrayNo][1]);

    // mark new section as occupied
    sectionOccupants[section] = trainNo;

    Serial.print("Occupy section ");
    Serial.println(section);
  }
}

void msgHandler(const CAN::MessageEvent * message)
{
  //Motorola::setMessage(0, Motorola::oldTrainMessage(trainAddressMap[0], true, 0));
  //Motorola::setMessage(1, Motorola::oldTrainMessage(trainAddressMap[1], true, 0));
  //Motorola::setMessage(2, Motorola::oldTrainMessage(trainAddressMap[2], true, 0));
  //Motorola::setMessage(3, Motorola::oldTrainMessage(trainAddressMap[3], true, 0));

  CAN::StdIdentifier contactAddr = message->stdIdentifier;
  uint32_t timestamp = decodeLong(message->content);
  uint32_t duration = decodeLong(message->content + 4);

  //Serial.print("Msg from 0x");Serial.println(contactAddr, HEX);
  //Serial.print(": "); Serial.print(timestamp); Serial.print(" - "); Serial.println(duration);

  if((contactAddr & 0x1) != 0)
    return;

  if(duration != 0)
    return;

  uint8_t section = UINT8_MAX;
  bool entering = false;

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
      if(switchArrayResetNeeded[switchArrayNo])
      {
        setSwitchArray(switchArrayAddress[switchArrayNo], switchArrayStateMap[0]);
        switchArrayResetNeeded[switchArrayNo] = false;
        continue;
      }

      if(switchArrayBusy[switchArrayNo])
        continue;

      uint8_t nextTrainNo = switchArrayOccupants[switchArrayNo][0];
      if(nextTrainNo != 0) // is there a train waiting?
      {
        // find a free section, if any
        uint8_t nextSectionBase = switchArrayNo ? 0 : 2;
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
        uint8_t currentSectionBase = switchArrayNo ? 2 : 0;
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

        Serial.print("SA");
        Serial.print(switchArrayNo);
        Serial.println(" is busy");

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
        Motorola::setMessage(nextTrainNo, Motorola::oldTrainMessage(trainAddressMap[(uint8_t)nextTrainNo], true, (uint8_t)trainTargetSpeedMap[nextTrainNo]));

        Serial.print("Starting train ");
        Serial.print(nextTrainNo);
        Serial.print(" from section ");
        Serial.print(currentSection);
        Serial.print(" to ");
        Serial.println(freeSection);
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

  if(incomingSerialByte == 'H')
  {
    Serial.println("stopping all trains");
    for(uint8_t trainNo = 0; trainNo < trainAddressCount; trainNo++)
    {
      Motorola::setMessage(trainNo, Motorola::oldTrainMessage(trainAddressMap[(uint8_t)trainNo], true, (uint8_t)0));
    }
  }

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
      trainTargetSpeedMap[trainNo] = speed;
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

  // reset switch arrays
  for(uint8_t switchArrayNo = 0; switchArrayNo < 2; switchArrayNo++)
  {
    setSwitchArray(switchArrayAddress[switchArrayNo], switchArrayStateMap[0]);
  }

  for(uint8_t i = 0; i < trainAddressCount; ++i)
  {
    // Don't create a dedicated message slot for the idle message
    if(i == trainIdleAddressIndex)
      continue;

    Motorola::setMessage(i, Motorola::oldTrainMessage(trainAddressMap[i], true, trainTargetSpeedMap[i]));
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
