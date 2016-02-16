#include "motorola.h"

ISR(TIMER1_OVF_vect)
{
  Motorola::onTimerOverflow();
}

uint8_t addressToLineBits(int8_t address)
{
    // fixes for weird encoding
    if (address == 80) address = 0;
    if (address < 0 || address > 80) address = 81;

    uint8_t encodedAddress = 0;

    uint8_t dividend = address;
    uint8_t quotient = 0;
    uint8_t remainder = 0;

    uint8_t trits[] = {
        0b00, // 0
        0b11, // 1
        0b01  // "open"
    };

    for(uint8_t i=0; i<4; i++)
    {
        quotient = dividend / 3;
        remainder = dividend % 3;

        dividend = quotient;

        encodedAddress |= trits[remainder] << (2*i);
    }

    return encodedAddress;
}

uint8_t speedToLineBits(uint8_t speed)
{
    if (speed > 15)
    {
        speed = 15;
    }
    uint8_t encodedSpeed = 0;

    for(uint8_t i = 0; i < 4; i++)
    {
      encodedSpeed |= ((speed & (0x1 << i))? 0b11 : 0b00) << (2*i);
    }

    return encodedSpeed;
}

uint8_t switchStateToLineBits(uint8_t switchAddress, bool state)
{
  uint8_t encodedState = 0;

  for(int i = 0; i < 3; ++i)
  {
    encodedState |= ((switchAddress & (0x1 << i))? 0b11 : 0b00) << (2*i);
  }
  encodedState |= (state? 0b11 : 0b00) << 6;

  return encodedState;
}

Motorola::Message Motorola::oldTrainMessage(uint8_t address, bool function, uint8_t speedLevel)
{
  Motorola::Message message = 0;

  message |= ((uint32_t) speedToLineBits(speedLevel)) << 10;
  message |= (function? 0b11 : 0b00) << 8;
  message |= ((uint32_t) addressToLineBits(address)) << 0;

  return message;
}

Motorola::Message Motorola::switchMessage(uint8_t decoderAddress, uint8_t switchAddress, bool state)
{
  Motorola::Message message = 0;

  message |= ((uint32_t) switchStateToLineBits(switchAddress, state)) << 10;
  message |= ((uint32_t) addressToLineBits(decoderAddress)) << 0;

  return message;
}

void Motorola::start()
{
  cli();

  digitalWrite(PinGo, LOW);
  pinMode(PinGo, OUTPUT);

  digitalWrite(PinData, HIGH);
  pinMode(PinData, OUTPUT);

  pinMode(PinError, INPUT);
  attachInterrupt(digitalPinToInterrupt(PinError), &onErrorPin, RISING);

  s_msgEnabled = 0;
  s_msgSpeed = 0;
  s_msgOneShot = 0;
  s_currentMsgNumber = 0;
  loadNextMessage();
  s_state = false;
  s_bitCounter = 0;

  TCCR1A = 0b11000010;
  TCCR1B = 0b00011010; // non inverted fast PWM on OC1A (D9), fT1 = fCPU / 8, TOP = ICR1
  ICR1 = 416;
  OCR1A = 0;
  TIMSK1 |= (1 << TOIE1); // Enable Timer 1 Overflow Interrupt
  TIFR1 = 0;
  TCNT1  = 0;
  s_running = true;

  sei();
  digitalWrite(PinGo, LOW);
}

void Motorola::setMessage(uint8_t n, Message message)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag

  if(n < MessageBufferSize)
  {
    s_msgBuffer[n] = message;
  }

  SREG = SaveSREG; // restore interrupt flag
}
Motorola::Message Motorola::getMessage(uint8_t n)
{
  if(n < MessageBufferSize)
  {
    return s_msgBuffer[n];
  }
  return IdleMessage;
}
void Motorola::enableMessage(uint8_t n)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag

  s_msgEnabled |= (0x1 << n);

  SREG = SaveSREG; // restore interrupt flag
}
void Motorola::disableMessage(uint8_t n)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag

  s_msgEnabled &= ~(0x1 << n);

  SREG = SaveSREG; // restore interrupt flag
}

boolean Motorola::messageEnabled(uint8_t n)
{
  return s_msgEnabled & (0x1 << n);
}

void Motorola::setMessageSpeed(uint8_t n, boolean speed)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag

  if(speed)
  {
    s_msgSpeed |= (0x1 << n);
  }
  else
  {
    s_msgSpeed &= ~(0x1 << n);
  }

  SREG = SaveSREG; // restore interrupt flag
}

void Motorola::setMessageOneShot(uint8_t n, boolean oneShot)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag

  if(oneShot)
  {
    s_msgOneShot |= (0x1 << n);
  }
  else
  {
    s_msgOneShot &= ~(0x1 << n);
  }

  SREG = SaveSREG; // restore interrupt flag
}

uint8_t cnt = 0;

void Motorola::onTimerOverflow()
{
  if(!s_running)
    return;

  ICR1 = s_currentSpeed? 208 : 416;
  if(s_bitCounter >= BitCountMsg - 1)
  {
    if(s_state) //End of Message repetition
    {
      ICR1 *= (BitCountWait + 1);
      loadNextMessage();
    }
    else // End of Message
    {
      ICR1 *= (BitCountGap + 1);
    }
    s_state = !s_state;
    s_bitCounter = 0;
  }
  else
  {
    ++s_bitCounter;
  }
  boolean currentBit = (s_currentMessage >> s_bitCounter) & 0x1;
  OCR1A = (currentBit? 182 : 26) * (s_currentSpeed? 1 : 2);
}

void Motorola::onErrorPin()
{
  if(!s_running)
    return;
  s_running = false;
  digitalWrite(PinGo, HIGH); //switch rail voltage off
}

void Motorola::loadNextMessage()
{
  if(s_msgEnabled)
  {
    MessageBufferMask mask;
    do
    {
      s_currentMsgNumber = (s_currentMsgNumber + 1) % MessageBufferSize;
      mask = 1 << s_currentMsgNumber;
    } while(!(s_msgEnabled & mask));

    if(s_msgOneShot & mask)
    {
      s_msgEnabled &= ~mask;
    }
    s_currentMessage = s_msgBuffer[s_currentMsgNumber];
    s_currentSpeed = s_msgSpeed & mask;
  }
  else
  {
    s_currentMessage = IdleMessage;
    s_currentSpeed = IdleSpeed;
  }
}

Motorola::Message Motorola::s_msgBuffer[Motorola::MessageBufferSize];

Motorola::MessageBufferMask Motorola::s_msgEnabled;
Motorola::MessageBufferMask Motorola::s_msgSpeed;
Motorola::MessageBufferMask Motorola::s_msgOneShot;

bool Motorola::s_running = false;

uint8_t Motorola::s_currentMsgNumber;
bool Motorola::s_state;
uint8_t Motorola::s_bitCounter;
Motorola::MessageSpeed Motorola::s_currentSpeed;
Motorola::Message Motorola::s_currentMessage;
