#include "motorola.h"

//TODO-------------
#define MIN_SPEED_V1 0
#define MAX_SPEED_V1 14

#define MIN_SPEED_V2 0
#define MAX_SPEED_V2 14

unsigned char addressToLineBits(signed char address)
{
    // fixes for weird encoding
    if (address == 80) address = 0;
    if (address == -1) address = 81;

    int encodedAddress = 0;

    int dividend = address;
    int quotient = 0;
    int remainder = 0;

    unsigned char trits[] = {
        0b00, // 0
        0b11, // 1
        0b01  // "open"
    };

    int tritValue = 0;

    int i = 0;
    for(i=0; i<4; i++)
    {
        quotient = dividend / 3;
        remainder = dividend % 3;

        dividend = quotient;

        tritValue = trits[remainder] << (2*i);

        encodedAddress |= tritValue;
    }

    return encodedAddress;
}

unsigned char speedToLineBitsV1(int8_t speed)
{
    unsigned int i = 0;
    unsigned char encodedTrit = 0;
    unsigned char trits[] = {
        /* Trit 0          */ (0 << 1 | 0 << 0),
        /* Trit 1          */ (1 << 0 | 1 << 1),
        /* Trit 2 ("Open") */ (0 << 0 | 1 << 1)
    };
    /* Märklin fucked up encoding fixes: */
    if (speed >= 1) speed += 1;
    if (speed > MAX_SPEED_V1 + 1) speed = MAX_SPEED_V1 + 1;
    if (speed == -1) speed = 1;
    for (i = 0; i < 4; i++) {
        encodedTrit <<= 2;
        encodedTrit |= trits[speed & 1];
        speed >>= 1; // /= 2
    }
    return encodedTrit;
}

unsigned char speedToLineBitsV2(int8_t speed)
{
    unsigned char line = speedToLineBitsV1(speed);
    line &= 0b10101010; // Discard odd bits
    if (speed >= 0) {
        if (speed >= 7) {
            line |= 0b00010000;
        } else {
            line |= 0b00010001;
        }
    } else {
        if (speed >= 7) {
            line |= 0b10001000;
        } else {
            line |= 0b10001010;
        }
    }
    return line;
}

unsigned char speedToLineBits(int8_t speed)
{
    if (speed > 15)
    {
        speed = 15;
    }

    int encodedSpeed = 0;
    int dividend = speed;
    int quotient = 0;
    int remainder = 0;

    unsigned char trits[] = {
        0b00, // 0
        0b11, // 1
        0b01  // "open"
    };

    int tritValue = 0;

    int i = 0;
    for(i=0; i<4; i++)
    {
        quotient = dividend / 2;
        remainder = dividend % 2;

        dividend = quotient;

        tritValue = trits[remainder] << (2*i);

        encodedSpeed |= tritValue;
    }

    return encodedSpeed;
}

Motorola::Message motorolaAssembleMessage(
    Motorola::MessageSpeed messageSpeed,
    uint8_t address,
    boolean function,
    int8_t speed)
{
    Motorola::Message message = 0;

    message |= (uint32_t)0 << 31; // set invalid bit to false
    message |= (uint32_t)messageSpeed << 18;

    message |= speedToLineBits(speed) << 10;
    message |= (function * 0b11) << 8;
    message |= addressToLineBits(address) << 0;

    return message;
}
//---------------------
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

    motorola.message(0) = motorolaAssembleMessage(
                            0,
                            3,
                            true,
                            speed);
    motorola.message(1) = motorolaAssembleMessage(
                            0,
                            65,
                            true,
                            speed);
    Serial.println(speed);
    digitalWrite(13, !digitalRead(13));
    delay(1000);
  }
  
  
  
}
