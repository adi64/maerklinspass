#include "motorola.h"

Motorola motorola;

ISR(TIMER1_OVF_vect)
{
  motorola.onTimerOverflow();
}

void setup() {
  motorola.start();
  //TODO attachInterrupt(digitalPinToInterrupt(Motorola::PinError), [&motorola]() {motorola.onErrorPin();}, FALLING); 
}

void loop() {
  motorola.message(0) = Motorola::IdleMessage;
  motorola.setMessageSpeed(0, false);
  motorola.enableMessage(0);
  
  motorola.message(1) = Motorola::IdleMessage;
  motorola.setMessageSpeed(1, false);
  motorola.enableMessage(1);

  uint8_t speedStep = 0;
  while(1)
  {
    uint8_t speed = (speedStep > 15)? 30 - speedStep : speedStep;
    if (speedStep == 29)
    {
      speed = 0;
    }
    speedStep = (speedStep + 1) % 30;
    
    motorola.message(0) = Motorola::oldTrainMessage(3, true, speed);
    motorola.message(1) = Motorola::oldTrainMessage(65, true, speed);

    digitalWrite(13, !digitalRead(13));
    delay(1000);
  }
}
