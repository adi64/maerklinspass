#include "can.h"

void canEventWrapper()
{
  CAN::onCANEvent();
}

const SPISettings CAN::SPIConfig(10000000, MSBFIRST, SPI_MODE0);

void CAN::start(CAN::CANAddress address, CAN::CANAddress mask)
{
  SPI.begin();
  
  cli();
  m_address = address;
  m_mask = mask;
  m_transmitPending = false;
  m_eventsPending = 0;
  m_errorsPending = 0;
  m_msgHandler = nullptr;
  m_rtrHandler = nullptr;
  m_errorHandler = nullptr;
  attachInterrupt(digitalPinToInterrupt(PinNInt), &canEventWrapper, FALLING);

  pinMode(PinNCS, OUTPUT);
  digitalWrite(PinNCS, HIGH);

  byte adrh = (byte)((m_address & 0x7F8) >> 3);
  byte adrl = (byte)((m_address & 0x007) << 5);
  byte mskh = (byte)((m_mask & 0x7F8) >> 3);
  byte mskl = (byte)((m_mask & 0x007) << 5);

  byte resetCommand[] = {0xC0};
  canCommand(resetCommand, sizeof(resetCommand)/sizeof(byte));
  
  delay(100);
  
  byte initCommand[] = {
        0x02, 0x00, //SPI_WRITE beginning at address 0
        adrh, adrl, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84,
        0x00, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84,
        mskh, mskl, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0xF1, 0x04, 0x25, 0x00, 0x00, 0x00, 0x84,
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84,
        0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84,
        0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
  canCommand(initCommand, sizeof(initCommand)/sizeof(byte));

  sei();
}

void CAN::setMsgHandler(CAN::CANMsgHandler * handler)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag
  m_msgHandler = handler;
  SREG = SaveSREG; // restore interrupt flag
}

void CAN::setRTRHandler(CAN::CANRTRHandler * handler)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag
  m_rtrHandler = handler;
  SREG = SaveSREG; // restore interrupt flag
}

void CAN::setErrorHandler(CAN::CANErrorHandler * handler)
{
  uint8_t SaveSREG = SREG;
  cli(); // clear interrupt flag
  m_errorHandler = handler;
  SREG = SaveSREG; // restore interrupt flag
}

void CAN::send(CAN::CANAddress address, uint32_t timestamp, uint32_t duration)
{
  while(m_transmitPending)
    processEvents(0x04);
  m_transmitPending = true;
  
  CANAddress adr = (m_address & m_mask) | (address & ~m_mask);
  byte msgCommand[] = {
    0x40, //fill TXBuffer 0
    (byte)((adr & 0x7F8) >> 3), // adrh
    (byte)((adr & 0x007) << 5), // adrl
    0x00,
    0x00,
    0x08, // dlc
    (byte)((timestamp & 0xFF000000) >> 24), // msg0
    (byte)((timestamp & 0x00FF0000) >> 16), // msg1
    (byte)((timestamp & 0x0000FF00) >>  8), // msg2
    (byte)( timestamp & 0x000000FF),        // msg3
    (byte)((duration  & 0xFF000000) >> 24), // msg4
    (byte)((duration  & 0x00FF0000) >> 16), // msg5
    (byte)((duration  & 0x0000FF00) >>  8), // msg6
    (byte)( duration  & 0x000000FF)         // msg7
  };
  canCommand(msgCommand, sizeof(msgCommand)/sizeof(byte));

  byte sendCommand[] = {0x81};
  canCommand(sendCommand, sizeof(sendCommand)/sizeof(byte));
}

void CAN::sendRTR(CAN::CANAddress address)
{
  while(m_transmitPending)
    processEvents(0x04);
  m_transmitPending = true;
  
  CANAddress adr = (m_address & m_mask) | (address & ~m_mask);
  byte msgCommand[] = {
    0x40, //fill TXBuffer 0
    (byte)((adr & 0x7F8) >> 3), // adrh
    (byte)((adr & 0x007) << 5), // adrl
    0x00,
    0x00,
    0x40 // dlc
  };
  canCommand(msgCommand, sizeof(msgCommand)/sizeof(byte));

  byte sendCommand[] = {0x81};
  
  canCommand(sendCommand, sizeof(sendCommand)/sizeof(byte));
}

void CAN::processEvents(byte eventMask)
{
  while(m_eventsPending & eventMask)
  {
    if(m_eventsPending & eventMask & 0x20) // ERRIF
    {
      if(m_errorHandler)
        m_errorHandler(m_errorsPending);
      
      m_eventsPending &= ~0x20;
    }
    else if (m_eventsPending & eventMask & 0x04) // TX0IF
    {
      m_transmitPending = false;
      
      m_eventsPending &= ~0x04;
    }
    else if (m_eventsPending & eventMask & 0x01) // RX0IF
    {
      byte readCommand[] = { 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
      canCommand(readCommand, sizeof(readCommand)/sizeof(byte));
      CANAddress address = (((CANAddress)readCommand[1]) << 3) | (((CANAddress)readCommand[2]) >> 5);

      if(readCommand[5] & 0x40) // RTR
      {
        if(m_rtrHandler)
          m_rtrHandler(address);
      }
      else
      {
        uint32_t timestamp =  (((uint32_t)readCommand[6]) << 24) |
                              (((uint32_t)readCommand[7]) << 16) |
                              (((uint32_t)readCommand[8]) <<  8) |
                               ((uint32_t)readCommand[9]);
        uint32_t duration =   (((uint32_t)readCommand[10]) << 24) |
                              (((uint32_t)readCommand[11]) << 16) |
                              (((uint32_t)readCommand[12]) <<  8) |
                               ((uint32_t)readCommand[13]);

        if(m_msgHandler)
          m_msgHandler(address, timestamp, duration);
      }
      
      m_eventsPending &= ~0x01;
    }
  }
}

void CAN::onCANEvent()
{
  byte statusCommand[] = {0x03, 0x2B, 0x00, 0x00, 0x00}; // read CANINTE, CANINTF and EFLG
  canCommand(statusCommand, sizeof(statusCommand)/sizeof(byte));

  m_eventsPending |= statusCommand[3] & statusCommand[2];
  m_errorsPending |= statusCommand[4];

  byte clearCommand[] = {0x02, 0x2C, 0x00, 0x00}; // clear CANINTF and EFLG
  canCommand(clearCommand, sizeof(clearCommand)/sizeof(byte));  
}

void CAN::canCommand(byte * command, int length)
{
  SPI.beginTransaction(SPIConfig);
  digitalWrite(PinNCS, LOW);
  for(int i = 0; i < length; ++i) {
    command[i] = SPI.transfer(command[i]);
  }
  digitalWrite(PinNCS, HIGH);
  SPI.endTransaction();
}

CAN::CANAddress CAN::m_address = 0;
CAN::CANAddress CAN::m_mask = 0;
volatile bool CAN::m_transmitPending = false;
volatile byte CAN::m_eventsPending = 0;
volatile byte CAN::m_errorsPending = 0;
CAN::CANMsgHandler * CAN::m_msgHandler = nullptr;
CAN::CANRTRHandler * CAN::m_rtrHandler = nullptr;
CAN::CANErrorHandler * CAN::m_errorHandler = nullptr;

