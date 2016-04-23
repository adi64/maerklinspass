#include "can.h"

const SPISettings CAN::SPIConfig(10000000, MSBFIRST, SPI_MODE0);

void CAN::start(CAN::MessageHandler * msgHandler, CAN::ErrorHandler * errorHandler)
{
  cli();

  s_sendQueueFree = 0;
  s_sendQueueNext = 0;
  s_sendPending = false;

  s_receiveQueueFree = 0;
  s_receiveQueueNext = 0;
  s_handlingMessages = false;

  s_errorQueueFree = 0;
  s_errorQueueNext = 0;
  s_handlingErrors = false;

  s_messageHandler = msgHandler;
  s_errorHandler = errorHandler;

  SPI.begin();
  attachInterrupt(digitalPinToInterrupt(PinNInt), &onInterrupt, FALLING);
  pinMode(PinNCS, OUTPUT);
  digitalWrite(PinNCS, HIGH);

  byte resetCommand[] = {0xC0};
  canCommand(resetCommand, sizeof(resetCommand));

  delay(100);

  byte initCommand[] = {
        0x02, 0x00, //SPI_WRITE beginning at address 0
        0x00, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x00, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0xF1, 0x04, 0x25, 0x00, 0x00, 0x00, 0x80,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  canCommand(initCommand, sizeof(initCommand));

  sei();
}

void CAN::setReceiveFilter(CAN::StdIdentifier identifier, CAN::StdIdentifier mask)
{
  setMode(ModeConfig);

  uint8_t sidfh = (uint8_t)((identifier & 0x7F8) >> 3);
  uint8_t sidfl = (uint8_t)((identifier & 0x007) << 5); // EXIDE (bit 3) cleared
  uint8_t sidmh = (uint8_t)((mask & 0x7F8) >> 3);
  uint8_t sidml = (uint8_t)((mask & 0x007) << 5);

  uint8_t filterCommand[] = {0x02, 0x00, sidfh, sidfl};
  canCommand(filterCommand, sizeof(filterCommand)); //set RXF0

  uint8_t maskCommand[] = {0x02, 0x20, sidmh, sidml};
  canCommand(maskCommand, sizeof(maskCommand)); //set RXM0

  uint8_t bufferCommand[] = {0x02, 0x60, 0x00};
  canCommand(bufferCommand, sizeof(bufferCommand)); //enable RXB0

  setMode(ModeNormal);
}

void CAN::setReceiveFilter(CAN::ExtIdentifier identifier, CAN::ExtIdentifier mask)
{
  setMode(ModeConfig);

  uint8_t sidfh = (uint8_t)((identifier & 0x1FE00000) >> 21);
  uint8_t sidfl = (uint8_t)((identifier & 0x001C0000) >> 13) |
                  (uint8_t)((identifier & 0x00030000) >> 16) | 0x08; // EXIDE (bit 3) set
  uint8_t eidfh = (uint8_t)((identifier & 0x0000FF00) >> 8);
  uint8_t eidfl = (uint8_t) (identifier & 0x000000FF);

  uint8_t sidmh = (uint8_t)((mask & 0x1FE00000) >> 21);
  uint8_t sidml = (uint8_t)((mask & 0x001C0000) >> 13) |
                  (uint8_t)((mask & 0x00030000) >> 16);
  uint8_t eidmh = (uint8_t)((mask & 0x0000FF00) >> 8);
  uint8_t eidml = (uint8_t) (mask & 0x000000FF);

  uint8_t filterCommand[] = {0x02, 0x00, sidfh, sidfl, eidfh, eidfl};
  canCommand(filterCommand, sizeof(filterCommand)); //set RXF0

  uint8_t maskCommand[] = {0x02, 0x20, sidmh, sidml, eidmh, eidml};
  canCommand(maskCommand, sizeof(maskCommand)); //set RXM0

  uint8_t bufferCommand[] = {0x02, 0x60, 0x00};
  canCommand(bufferCommand, sizeof(bufferCommand)); //enable RXB0

  setMode(ModeNormal);
}

void CAN::clearReceiveFilter()
{
  setMode(ModeConfig);

  uint8_t filterCommand[] = {0x02, 0x00, 0x00, 0x08};
  canCommand(filterCommand, sizeof(filterCommand)); //set RXF0 to accept extended messages

  uint8_t bufferCommand[] = {0x02, 0x60, 0x20};
  canCommand(bufferCommand, sizeof(bufferCommand)); //enable RXB0 to accept only standard messages

  setMode(ModeNormal);
}

CAN::MessageEvent * CAN::prepareMessage()
{
  s_prepareMessageSREG = SREG;
  cli();

  if((s_sendQueueFree + 1) % MessageQueueSize == s_sendQueueNext)
  {
    SREG = s_prepareMessageSREG;
    return nullptr;
  }

  return s_sendQueue + s_sendQueueFree;
}

bool CAN::commitMessage(CAN::MessageEvent * message)
{
  if(message != (s_sendQueue + s_sendQueueFree)) // message pointer was not obtained by last call to prepareMessage()
    return false;

  s_sendQueueFree = (s_sendQueueFree + 1) % MessageQueueSize;

  if(!s_sendPending /*&& s_sendQueueNext != s_sendQueueFree*/)
  {
    s_sendPending = true;
    sendMessage(s_sendQueue + s_sendQueueNext);
    s_sendQueueNext = (s_sendQueueNext + 1) % MessageQueueSize;
  }

  SREG = s_prepareMessageSREG;
  return true;
}

void CAN::onInterrupt()
{
  uint8_t statusCommand[] = {0x03, 0x2C, 0x00, 0x00, }; // read CANINTF and EFLG
  canCommand(statusCommand, sizeof(statusCommand));
  uint8_t iflags = statusCommand[2];
  uint8_t eflags = statusCommand[3];
  uint8_t clearCommand[] = {0x02, 0x2C, 0x00, 0x00}; // clear CANINTF and EFLG
  canCommand(clearCommand, sizeof(clearCommand));

  if(iflags & 0x20) // ERRIF
  {
    if((s_errorQueueFree + 1) % ErrorQueueSize != s_errorQueueNext)
    {
      s_errorQueue[s_errorQueueFree].timestamp = millis();
      s_errorQueue[s_errorQueueFree].flags = eflags;
      s_errorQueueFree = (s_errorQueueFree + 1) % ErrorQueueSize;
    }
    //else error queue overflow: error will be lost
  }
  else if(iflags & 0x04) // TX0IF
  {
    if(s_sendQueueNext != s_sendQueueFree) // still messages in send buffer
    {
      s_sendPending = true;
      sendMessage(s_sendQueue + s_sendQueueNext);
      s_sendQueueNext = (s_sendQueueNext + 1) % MessageQueueSize;
    }
    else
    {
      s_sendPending = false;
    }
  }
  else if (iflags & 0x01) // RX0IF
  {
    if((s_receiveQueueFree + 1) % MessageQueueSize != s_receiveQueueNext)
    {
      receiveMessage(s_receiveQueue + s_receiveQueueFree);
      s_receiveQueueFree = (s_receiveQueueFree + 1) % MessageQueueSize;
    }
    //else message queue overflow: message will be lost
  }

  if(!s_handlingErrors)
  {
    s_handlingErrors = true;
    while(s_errorQueueNext != s_errorQueueFree)
    {
      if(s_errorHandler)
      {
        sei();
        s_errorHandler(s_errorQueue + s_errorQueueNext);
        cli();
      }
      s_errorQueueNext = (s_errorQueueNext + 1) % ErrorQueueSize;
    }
    s_handlingErrors = false;
  }

  if(!s_handlingMessages)
  {
    s_handlingMessages = true;
    while(s_receiveQueueNext != s_receiveQueueFree)
    {
      if(s_messageHandler)
      {
        sei();
        s_messageHandler(s_receiveQueue + s_receiveQueueNext);
        cli();
      }
      s_receiveQueueNext = (s_receiveQueueNext + 1) % MessageQueueSize;
    }
    s_handlingMessages = false;
  }
}

void CAN::sendMessage(const CAN::MessageEvent * message)
{
  uint8_t sidh = 0;
  uint8_t sidl = 0;
  uint8_t eidh = 0;
  uint8_t eidl = 0;
  if(message->hasExtIdentifier)
  {
    sidh = (uint8_t)((message->extIdentifier & 0x1FE00000) >> 21);
    sidl = (uint8_t)((message->extIdentifier & 0x001C0000) >> 13) |
           (uint8_t)((message->extIdentifier & 0x00030000) >> 16) | 0x08; // EXIDE (bit 3) set to transmit extended message
    eidh = (uint8_t)((message->extIdentifier & 0x0000FF00) >> 8);
    eidl = (uint8_t) (message->extIdentifier & 0x000000FF);
  }
  else
  {
    sidh = (uint8_t)((message->stdIdentifier & 0x7F8) >> 3);
    sidl = (uint8_t)((message->stdIdentifier & 0x007) << 5); // EXIDE (bit 3) cleared to transmit standard message
  }
  uint8_t dlc = (message->isRTR)? 0x40 : (message->length & 0x0F);

  uint8_t msgCommand[] = {0x40, // Write TXB0 from TXB0SIDH
    sidh,
    sidl,
    eidh,
    eidl,
    dlc,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  if(!message->isRTR)
  {
    for(uint8_t i = 0; i < 8; ++i)
    {
      msgCommand[i + 6] = message->content[i];
    }
  }
  canCommand(msgCommand, sizeof(msgCommand));

  uint8_t rtsCommand[] = {0x81}; // set TXB0CTRL.TXREQ
  canCommand(rtsCommand, sizeof(rtsCommand));
}

void CAN::receiveMessage(CAN::MessageEvent * message)
{
  message->timestamp = millis();

  uint8_t readCommand[] = { 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  canCommand(readCommand, sizeof(readCommand));

  if(readCommand[2] & 0x08) // RXB0SIDL.IDE set -> extended identifier
  {
    message->hasExtIdentifier = true;
    message->extIdentifier = (((ExtIdentifier) readCommand[1]) << 21) |
                             (((ExtIdentifier) readCommand[2] & 0xE0) << 13) |
                             (((ExtIdentifier) readCommand[2] & 0x03) << 16) |
                             (((ExtIdentifier) readCommand[3]) << 8) |
                              ((ExtIdentifier) readCommand[4]);
    message->isRTR = (readCommand[5] & 0x40); // RTR bit in RXB0DLC
  }
  else // standard identifier
  {
    message->hasExtIdentifier = false;
    message->extIdentifier = (((StdIdentifier) readCommand[1]) << 3) |
                             (((StdIdentifier) readCommand[2] & 0xE0) >> 5);
    message->isRTR = (readCommand[2] & 0x10); // RTR bit in RXB0SIDL
  }

  if(!message->isRTR)
  {
    message->length = readCommand[5] & 0x0F;
    for(uint8_t i = 0; i < 8; ++i)
    {
      message->content[i] = readCommand[i + 6];
    }
  }
}

void CAN::setMode(uint8_t mode)
{
  mode = (mode & 0x07) << 5;

  uint8_t command[] = {0x02, 0x0F, mode}; // write requested mode field in CANCTRL
  canCommand(command, sizeof(command));

  do
  {
    command[0] = 0x03;
    command[1] = 0x0E; // read current mode field in CANSTAT
    canCommand(command, sizeof(command));
  }
  while((command[2] & 0xE0) != mode); // wait for mode change
}

void CAN::canCommand(uint8_t * command, uint8_t length)
{
  uint8_t SaveSREG = SREG;
  cli();
  SPI.beginTransaction(SPIConfig);
  digitalWrite(PinNCS, LOW);
  for(int i = 0; i < length; ++i) {
    command[i] = SPI.transfer(command[i]);
  }
  digitalWrite(PinNCS, HIGH);
  SPI.endTransaction();
  SREG = SaveSREG;
}

CAN::MessageEvent CAN::s_receiveQueue[MessageQueueSize];
volatile uint8_t CAN::s_receiveQueueFree;
volatile uint8_t CAN::s_receiveQueueNext;
volatile bool CAN::s_handlingMessages;

CAN::MessageEvent CAN::s_sendQueue[MessageQueueSize];
volatile uint8_t CAN::s_sendQueueFree;
volatile uint8_t CAN::s_sendQueueNext;
volatile bool CAN::s_sendPending;

CAN::ErrorEvent CAN::s_errorQueue[ErrorQueueSize];
volatile uint8_t CAN::s_errorQueueFree;
volatile uint8_t CAN::s_errorQueueNext;
volatile bool CAN::s_handlingErrors;

CAN::MessageHandler * CAN::s_messageHandler;
CAN::ErrorHandler * CAN::s_errorHandler;

uint8_t CAN::s_prepareMessageSREG;

