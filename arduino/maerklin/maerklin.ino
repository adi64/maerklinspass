#include "can.h"
#include "motorola.h"

#include <Streaming.h>

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

What this code does:

Upon startup, all trains are set to their "default" speed defined in this file.
When a train passes a segment border, an event is generated and transmitted via CAN.
This CAN message / event contains the information which border was crossed (e.g. 3F8).

Note that every "train detector switch" emits events with different IDs for forward and backward activation.
Since trains are only going counter-clockwise in this scenario, we only listen for events important to us.

When a CAN message is received, msgHandler is called and determines the corresponding track segment:
* If the train is entering a switch array, it's the segment the train is coming from.
* If the train is leaving a switch array, it's the segment the train is going to.

Then, handleSwitchArrayEvent is called with this information.
* If the train is entering a switch array, it's stopped immediately.
  The train is then added to the queue of trains waiting to cross this switch array.
  Note that the train is still marked as occupying the segment that it came from.
* If the train is leaving a switch array, it is removed from the array's waiting queue.
  The track segment ahead is then marked as occupied with that train.
  The previous segment is marked as clear.
  The switch array is marked as idle but in need of a reset (to a neutral position).

## TODO operateSwitchArrays
## TODO parseSerialInput

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
      Serial << F("### ERROR: There shouldn't be a train on section ") << section << endl;
      return;
    }

    // Put train in switch array waiting list
    uint8_t switchArrayNo = (section & 0b10) ? 1 : 0; // section 0,1 -> SA0; section 2,3 -> SA1
    Serial << F("Train ") << trainNo << F(" is entering SA") << switchArrayNo << endl;
    if(switchArrayOccupants[switchArrayNo][0] == trainNo || switchArrayOccupants[switchArrayNo][1] == trainNo)
    {
      Serial << F("### ERROR: Train ") << trainNo << F(" is already in queue for SA") << switchArrayNo << endl;
      return;
    }
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
      Serial << F("### ERROR: Switch Array ") << switchArrayNo << F(" is already occupied on both tracks") << endl;
      return;
    }

    // stop train
    Motorola::setMessage(trainNo, Motorola::oldTrainMessage(trainAddressMap[trainNo], true, 0));

    Serial << F("SA") << switchArrayNo << F(" queue: ") << switchArrayOccupants[switchArrayNo][0] << F(" ")
                                                  << switchArrayOccupants[switchArrayNo][1] << endl;

  }
  else // train is leaving switch array
  {
    uint8_t switchArrayNo = (section & 0b10) ? 0 : 1; // SA0 -> section 2,3; SA1 -> section 0,1
    uint8_t trainNo = switchArrayOccupants[switchArrayNo][0]; // first train in SA queue

    Serial << F("Train ") << trainNo << F(" is leaving SA") << switchArrayNo << endl;

    // sanity check train number
    if(trainNo == 0 || trainNo > trainAddressCount)
    {
      Serial << F("### ERROR: Switch array ") << switchArrayNo << F(" was not occupied!") << endl;
      return;
    }

    // sanity check section occupants
    if(sectionOccupants[section] != 0)
    {
      Serial << F("### ERROR: Section ") << section << F(" is already occupied by train ") << sectionOccupants[section] << endl;
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
      Serial << F("### ERROR: Switch array ") << switchArrayNo << F(" was not marked busy but a train just left it!") << endl;
      return;
    }

    switchArrayBusy[switchArrayNo] = false;

    // sanity check switch array reset requests
    if(switchArrayResetNeeded[switchArrayNo])
    {
      Serial << F("### ERROR: Switch array ") << switchArrayNo << F(" still needs to be reset but a train just left it!") << endl;
      return;
    }

    // request a reset of switch array
    switchArrayResetNeeded[switchArrayNo] = true;

    Serial << F("schedule restart of SA") << switchArrayNo << endl;

    // sanity check switch array queue
    if(switchArrayOccupants[switchArrayNo][0] != trainNo)
    {
      Serial << F("### ERROR: Train ") << trainNo << F(" was not first in queue of switch array ") << switchArrayNo << endl;
      return;
    }

    // cycle switch array queue
    switchArrayOccupants[switchArrayNo][0] = switchArrayOccupants[switchArrayNo][1];
    switchArrayOccupants[switchArrayNo][1] = 0;

    Serial << F("SA") << switchArrayNo << F(" queue: ") << switchArrayOccupants[switchArrayNo][0] << F(" ") << switchArrayOccupants[switchArrayNo][1] << endl;

    // mark new section as occupied
    sectionOccupants[section] = trainNo;

    Serial << F("Occupy section ") << section << endl;
  }
}

void msgHandler(const CAN::MessageEvent * message)
{

  CAN::StdIdentifier contactAddr = message->stdIdentifier;
  uint32_t duration = decodeLong(message->content + 4);

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
            Serial << F("### ERROR: Train ") << nextTrainNo << F(" is waiting on switch array ") << switchArrayNo << F(" but is neither on section ") << currentSectionBase << F(" nor ") << currentSectionBase+1 << endl;
            return;
        }

        // set switch array busy
        switchArrayBusy[switchArrayNo] = true;

        Serial << F("SA") << switchArrayNo << F(" is busy") << endl;

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

        Serial << F("Starting train ") << nextTrainNo << F(" from section ") << currentSection << F(" to ") << freeSection << endl;
      }

    }
}

void errorHandler(const CAN::ErrorEvent * error)
{
  Serial << F("ERROR: 0x") << _HEX(error->flags) << endl;
}

void parseSerialInput()
{
  // Read 1 serial byte
  incomingSerialByte = Serial.read();
  if(incomingSerialByte == -1)
    return;

  if(incomingSerialByte == 'H')
  {
    Serial << F("stopping all trains") << endl;
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

    Serial << F("Zug ") << trainNo << F(": ") << speed << endl;

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
    Serial << F("Weiche ") << swaAddr << F(": ") << state << endl;

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
