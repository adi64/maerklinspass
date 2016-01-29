class Motorola
{
  enum class State : uint8_t
  {
    MSG0,
    MSG1
  };
  
public:
  using Message = uint32_t;
  using MessageSpeed = boolean; // bit time is (speed)? 104 : 208 us
  using MessageBufferMask = uint8_t;
  static constexpr uint8_t MessageBufferSize = 8;

  static constexpr Message IdleMessage = 0x00055;
  static constexpr Message IdleSpeed = false;

  static constexpr int PinData = 9;
  static constexpr int PinGo = 4;
  static constexpr int PinError = 3;

  static constexpr uint8_t BitCountMsg = 18;
  static constexpr uint8_t BitCountGap = 6;
  static constexpr uint8_t BitCountWait = 22;

  Motorola();

  void start();

  Message & message(uint8_t n);
  void enableMessage(uint8_t n);
  void disableMessage(uint8_t n);
  boolean messageEnabled(uint8_t n);
  void setMessageSpeed(uint8_t n, MessageSpeed speed);
  void setMessageOneShot(uint8_t n, boolean oneShot);

  void onTimerOverflow();
  void onTimerMatchA();
  void onErrorPin();

private:
  void loadNextMessage();
  
private:
  Message m_dummyMsg;
  Message m_msgBuffer[MessageBufferSize];

  MessageBufferMask m_msgEnabled;
  MessageBufferMask m_msgSpeed;
  MessageBufferMask m_msgOneShot;

  boolean m_running;

  uint8_t m_currentMsgNumber;
  boolean m_state;
  uint8_t m_bitCounter;
  MessageSpeed m_currentSpeed;
  Message m_currentMessage;
};

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
  
  TCCR1A = 0b10000010;
  TCCR1B = 0b00011010; // non inverted fast PWM on OC1A (D9), fT1 = fCPU / 8, TOP = ICR1
  ICR1 = 416;
  OCR1A = 0;
  TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A); // Enable Timer 1 Overflow Interrupt and Compare A Interrupt
  TIFR1 = 0;
  TCNT1  = 0;
  m_running = true;
  interrupts();
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
  
  digitalWrite(5, !digitalRead(5));
  digitalWrite(4, m_state);
  
  OCR1A = ((m_currentMessage & (0x1 << m_bitCounter))? 182 : 26) * (m_currentSpeed? 1 : 2);
  ICR1 = m_currentSpeed? 208 : 416;
  if(m_bitCounter >= BitCountMsg - 1)
  {
    if(m_state) //End of Message repetition
    {
      ICR1 *= BitCountWait;
      loadNextMessage();
    }
    else // End of Message
    {
      ICR1 *= BitCountGap;
    }
    m_state = !m_state;
    m_bitCounter = 0;
  }
  else
  {
    ++m_bitCounter;
  }
}

void Motorola::onTimerMatchA()
{
  if(!m_running)
    return;
    
//  if(m_state == State::IDLE)
//  {
//    TCCR1A &= 0b01111111;
//  }
}

void Motorola::onErrorPin()
{
  if(!m_running)
    return;
    
  digitalWrite(PinGo, LOW); //switch rail voltage off
}

void Motorola::loadNextMessage()
{
  if(m_msgEnabled)
  {
    MessageBufferMask mask = 1 << m_currentMsgNumber;
    while(!(m_msgEnabled & mask))
    {
      m_currentMsgNumber = (m_currentMsgNumber + 1) % MessageBufferSize;
      mask = 1 << m_currentMsgNumber;
    }
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

Motorola motorola;

ISR(TIMER1_OVF_vect)
{
  motorola.onTimerOverflow();
}

ISR(TIMER1_COMPA_vect)
{
  motorola.onTimerMatchA();
}

void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  Serial.begin(9600);

  motorola.start();
}

void loop() {
  delay(5000);
  motorola.message(0) = 0x3FFF;
  motorola.setMessageSpeed(0, false);
  motorola.message(6) = 0x3AAAA;
  motorola.setMessageSpeed(6, true);
  motorola.enableMessage(0);
//  while(1)
//  {
//    digitalWrite(2, HIGH);
//    motorola.enableMessage(0);
//    delay(1000);
//    digitalWrite(3, HIGH);
//    motorola.enableMessage(6);
//    delay(1000);
//    digitalWrite(2, LOW);
//    motorola.disableMessage(0);
//    delay(1000);
//    digitalWrite(3, LOW);
//    motorola.disableMessage(6);
//    delay(1000);
//  }
}
