#include "motorola.h"

Motorola motorola;

const uint8_t address[] = {1, 3, 65};
const uint8_t addressCount = 3;

constexpr uint8_t SwitchMsgSlot = 7;

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

void setup() {
  motorola.start(); 

  for(uint8_t i = 0; i < addressCount; ++i)
  {
    motorola.setMessage(i, Motorola::oldTrainMessage(address[i], true, 0));
    motorola.setMessageSpeed(i, false);
    motorola.setMessageOneShot(i, false);
    motorola.enableMessage(i);
  }

  Serial.begin(9600);
  Serial.setTimeout(60000);
}

void loop() {
  String cmd = Serial.readStringUntil(' ');
  if(cmd.indexOf('L') >= 0)
  {
    long trainNo = Serial.parseInt();
    long speed = Serial.parseInt();
    Serial.print("Zug "); Serial.print(trainNo); Serial.print(": "); Serial.println(speed);
    
    if(0 <= trainNo && trainNo < addressCount &&
       0 <= speed && speed < 16)
    {
      motorola.setMessage(trainNo, Motorola::oldTrainMessage(address[(uint8_t)trainNo], true, (uint8_t)speed));
    }
  }
  else if(cmd.indexOf('W') >= 0)
  {
    long swaAddr = Serial.parseInt();
    long state = Serial.parseInt();
    Serial.print("Weiche "); Serial.print(swaAddr); Serial.print(": "); Serial.println(state);
    
    if(0 <= state && state < 3)
    {
      setSwitchArray(swaAddr, (state == 0)? 15 : ((state == 1)? 12 : 3));
    }
  }
}
