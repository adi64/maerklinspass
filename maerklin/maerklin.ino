#include "can.h"
#include "motorola.h"

#include <Streaming.h>

/*
Please read the README.md in this directory for general explanations!

What this code does:

Upon startup, all trains are set to their "default" speed defined in trainTargetSpeedMap.
When a train passes a segment border, an event is generated and transmitted via CAN.
This CAN message / event contains the information which border was crossed (e.g. 308).

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

The method operateSwitchArrays is called in a busy loop as long as there are no CAN messages incoming.
* If a switch array is idle and needs to be reset, it is reset to a neutral position (passthrough).
* If a switch array is idle and there is a train waiting in its queue and a connected track segment is free,
  the switch array is marked busy and is set to the appropriate configuration.
  The train waiting first in queue is then started to cross the switch array.
  After the train has crossed the switch array, it will hit a detector switch and therefore trigger a new event.

Authors: Adrian Holfter, Lukas Wenzel
*/


const uint8_t trainAddressMap[] = {Motorola::IdleAddress, 1, 3, 65};
const uint8_t trainAddressCount = 4;
const uint8_t trainIdleAddressIndex = 0;
uint8_t trainTargetSpeedMap[] = {0, 8, 6, 8}; // default speeds of trains - can be updated at runtime

constexpr uint8_t switchMsgSlot = 7;

constexpr uint8_t switchArrayAddress[] = {1, 3};
constexpr uint8_t SWITCH_ARRAY_STRAIGHT = 0xF;
constexpr uint8_t SWITCH_ARRAY_IN2OUT = 0x3;
constexpr uint8_t SWITCH_ARRAY_OUT2IN = 0xC;

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

void stopAllTrains()
{
  Serial << F("stopping all trains") << endl;
  for(uint8_t trainNo = 0; trainNo < trainAddressCount; trainNo++)
  {
    Motorola::setMessage(trainNo, Motorola::oldTrainMessage(trainAddressMap[(uint8_t)trainNo], true, (uint8_t)0));
  }
}

// A train is entering or leaving a switch array - take corresponding action
void handleSwitchArrayEvent(uint8_t section, bool entering)
{
  if(entering) // train is entering switch array
  {
    uint8_t trainNo = sectionOccupants[section];
    if(trainNo == 0 || trainNo > trainAddressCount)
    {
      Serial << F("### ERROR: There shouldn't be a train on section ") << section << endl;
	  stopAllTrains();
      return;
    }

    // Put train in switch array waiting list
    uint8_t switchArrayNo = (section & 0b10) ? 1 : 0; // section 0,1 -> SA0; section 2,3 -> SA1
    Serial << F("Train ") << trainNo << F(" is entering SA") << switchArrayNo << endl;
    if(switchArrayOccupants[switchArrayNo][0] == trainNo || switchArrayOccupants[switchArrayNo][1] == trainNo)
    {
      Serial << F("### WARNING: Train ") << trainNo << F(" is already in queue for SA") << switchArrayNo << endl;
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
	  stopAllTrains();
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
	  stopAllTrains();
      return;
    }

    // sanity check section occupants
    if(sectionOccupants[section] != 0)
    {
	  if(sectionOccupants[section] == trainNo)
	  {
		// probably just a bouncyness problem
		Serial << F("### WARNING: Section ") << section << F(" is already occupied by train ") << sectionOccupants[section] << endl;
	  }
	  else
	  {
		// security violation!
		Serial << F("### ERROR: Section ") << section << F(" is already occupied by train ") << sectionOccupants[section] << endl;
		stopAllTrains();
	  }
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
	  stopAllTrains();
      return;
    }

    switchArrayBusy[switchArrayNo] = false;

    // sanity check switch array reset requests
    if(switchArrayResetNeeded[switchArrayNo])
    {
      Serial << F("### ERROR: Switch array ") << switchArrayNo << F(" still needs to be reset but a train just left it!") << endl;
	  stopAllTrains();
      return;
    }

    // request a reset of switch array
    switchArrayResetNeeded[switchArrayNo] = true;

    Serial << F("schedule restart of SA") << switchArrayNo << endl;

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
	default:
	  return;
  }

  handleSwitchArrayEvent(section, entering);
}

void operateSwitchArrays()
{
    for(uint8_t switchArrayNo = 0; switchArrayNo < 2; switchArrayNo++)
    {
      if(switchArrayResetNeeded[switchArrayNo])
      {
        setSwitchArray(switchArrayAddress[switchArrayNo], SWITCH_ARRAY_STRAIGHT);
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
			stopAllTrains();
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
            setSwitchArray(switchArrayAddress[switchArrayNo], SWITCH_ARRAY_IN2OUT);
          }
          else
          {
            // switch out->in
            setSwitchArray(switchArrayAddress[switchArrayNo], SWITCH_ARRAY_OUT2IN);
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
	stopAllTrains();
  }

  // barrel shift incoming bytes
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

    // Don't set speed for Motorola::IdleAddress
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
    long state = 0;
    switch(parsedSerialBytes[2])
    {
        case 1:
            state = SWITCH_ARRAY_IN2OUT;
            break;
        case 2:
            state = SWITCH_ARRAY_OUT2IN;
            break;
        case 0:
        default:
            state = SWITCH_ARRAY_STRAIGHT;
    }
    Serial << F("Weiche ") << swaAddr << F(": ") << state << endl;
    setSwitchArray(switchArrayAddress[swaAddr], state);
  }
}

void setup() {
  Motorola::start();

  // reset switch arrays
  for(uint8_t switchArrayNo = 0; switchArrayNo < 2; switchArrayNo++)
  {
    setSwitchArray(switchArrayAddress[switchArrayNo], SWITCH_ARRAY_STRAIGHT);
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
