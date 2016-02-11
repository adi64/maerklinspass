#include "motorola.h"

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

Motorola::Motorola()
: m_msgEnabled(0x00)
, m_msgSpeed(0x00)
, m_running(false)
{
  pinMode(PinData, OUTPUT);
  pinMode(PinGo, OUTPUT);
  pinMode(PinError, INPUT);
  
  digitalWrite(PinGo, HIGH);
}

void Motorola::start()
{
  cli();
  
  m_currentMsgNumber = 0;
  loadNextMessage();
  m_state = false;
  m_bitCounter = 0;
  
  TCCR1A = 0b11000010;
  TCCR1B = 0b00011010; // non inverted fast PWM on OC1A (D9), fT1 = fCPU / 8, TOP = ICR1
  ICR1 = 416;
  OCR1A = 0;
  TIMSK1 |= (1 << TOIE1); // Enable Timer 1 Overflow Interrupt
  TIFR1 = 0;
  TCNT1  = 0;
  m_running = true;

  sei();
  digitalWrite(PinGo, LOW);
}

void Motorola::setMessage(uint8_t n, Message message)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag

  if(n < MessageBufferSize)
  {
    m_msgBuffer[n] = message;
  }

  SREG = SaveSREG; // restore interrupt flag
}
Motorola::Message Motorola::getMessage(uint8_t n)
{
  if(n < MessageBufferSize)
  {
    return m_msgBuffer[n];
  }
  return IdleMessage;
}
void Motorola::enableMessage(uint8_t n)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag

  m_msgEnabled |= (0x1 << n);

  SREG = SaveSREG; // restore interrupt flag
}
void Motorola::disableMessage(uint8_t n)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag

  m_msgEnabled &= ~(0x1 << n);

  SREG = SaveSREG; // restore interrupt flag
}

boolean Motorola::messageEnabled(uint8_t n)
{
  return m_msgEnabled & (0x1 << n);
}

void Motorola::setMessageSpeed(uint8_t n, boolean speed)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag

  if(speed)
  {
    m_msgSpeed |= (0x1 << n);
  }
  else
  {
    m_msgSpeed &= ~(0x1 << n);
  }

  SREG = SaveSREG; // restore interrupt flag
}

void Motorola::setMessageOneShot(uint8_t n, boolean oneShot)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag

  if(oneShot)
  {
    m_msgOneShot |= (0x1 << n);
  }
  else
  {
    m_msgOneShot &= ~(0x1 << n);
  }

  SREG = SaveSREG; // restore interrupt flag
}

uint8_t cnt = 0;

void Motorola::onTimerOverflow()
{
  if(!m_running)
    return;
  
  ICR1 = m_currentSpeed? 208 : 416;
  if(m_bitCounter >= BitCountMsg - 1)
  {
    if(m_state) //End of Message repetition
    {
      ICR1 *= (BitCountWait + 1);
      loadNextMessage();
    }
    else // End of Message
    {
      ICR1 *= (BitCountGap + 1);
    }
    m_state = !m_state;
    m_bitCounter = 0;
  }
  else
  {
    ++m_bitCounter;
  }
  boolean currentBit = (m_currentMessage >> m_bitCounter) & 0x1;
  OCR1A = (currentBit? 182 : 26) * (m_currentSpeed? 1 : 2);
}

void Motorola::onErrorPin()
{
  if(!m_running)
    return;
  m_running = false;
  digitalWrite(PinGo, HIGH); //switch rail voltage off
}

void Motorola::loadNextMessage()
{
  if(m_msgEnabled)
  {
    MessageBufferMask mask;
    do
    {
      m_currentMsgNumber = (m_currentMsgNumber + 1) % MessageBufferSize;
      mask = 1 << m_currentMsgNumber;
    } while(!(m_msgEnabled & mask));

    if(m_msgOneShot & mask)
    {
      m_msgEnabled &= ~mask;
    }
    m_currentMessage = m_msgBuffer[m_currentMsgNumber];
    m_currentSpeed = m_msgSpeed & mask;
  }
  else
  {
    m_currentMessage = IdleMessage;
    m_currentSpeed = IdleSpeed;
  }
}
