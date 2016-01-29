#include "motorola.h"

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
  noInterrupts();
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
  interrupts();
  digitalWrite(PinGo, LOW);
}

Motorola::Message & Motorola::message(uint8_t n)
{
  if(n < MessageBufferSize)
  {
    return m_msgBuffer[n];
  }
  return m_dummyMsg;
}
void Motorola::enableMessage(uint8_t n)
{
  m_msgEnabled |= (0x1 << n);
}
void Motorola::disableMessage(uint8_t n)
{
  m_msgEnabled &= ~(0x1 << n);
}

boolean Motorola::messageEnabled(uint8_t n)
{
  return m_msgEnabled & (0x1 << n);
}

void Motorola::setMessageSpeed(uint8_t n, boolean speed)
{
  if(speed)
  {
    m_msgSpeed |= (0x1 << n);
  }
  else
  {
    m_msgSpeed &= ~(0x1 << n);
  }
}

void Motorola::setMessageOneShot(uint8_t n, boolean oneShot)
{
  if(oneShot)
  {
    m_msgOneShot |= (0x1 << n);
  }
  else
  {
    m_msgOneShot &= ~(0x1 << n);
  }
}

uint8_t cnt = 0;

void Motorola::onTimerOverflow()
{
  if(!m_running)
    return;
  
  //OCR1A = ((m_currentMessage & (0x1 << m_bitCounter))? 182 : 26) * (m_currentSpeed? 1 : 2);
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
