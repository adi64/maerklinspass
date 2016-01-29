//TODO-------------
#define MIN_SPEED_V1 0
#define MAX_SPEED_V1 14

#define MIN_SPEED_V2 0
#define MAX_SPEED_V2 14
//-----------------

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

//  static Message oldTrainMessage(uint8_t address, boolean function, uint8_t speedLevel);
//  static Message newTrainMessageDirection(uint8_t address, boolean function, int8_t speed);
//  static Message newTrainMessageFunction(uint8_t address, boolean function, int8_t speed, uint8_t nFunction, boolean on);
//  static Message switchMessage(uint8_t address,);

  Motorola();

  void start();

  Message & message(uint8_t n);
  void enableMessage(uint8_t n);
  void disableMessage(uint8_t n);
  boolean messageEnabled(uint8_t n);
  void setMessageSpeed(uint8_t n, MessageSpeed speed);
  void setMessageOneShot(uint8_t n, boolean oneShot);

  void onTimerOverflow();
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

//TODO------------------

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
