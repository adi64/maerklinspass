#pragma once

#include <Arduino.h>

#include <SPI.h>


class CAN
{
  static const SPISettings SPIConfig;

  static constexpr int PinNCS = 10;
  static constexpr int PinNInt = 2;
  static constexpr int PinMISO = 12;
  static constexpr int PinMOSI = 11;
  static constexpr int PinSCK = 13;

  static constexpr uint8_t ModeNormal = 0;
  static constexpr uint8_t ModeSleep = 1;
  static constexpr uint8_t ModeLoopback = 2;
  static constexpr uint8_t ModeListen = 3;
  static constexpr uint8_t ModeConfig = 4;

public:
  using StdIdentifier = uint16_t;
  using ExtIdentifier = uint32_t;

  using ErrorEvent = struct
  {
    uint32_t timestamp;

    uint8_t flags;
  };
  using ErrorHandler = void(const ErrorEvent *);

  using MessageEvent = struct
  {
    uint32_t timestamp;

    bool hasExtIdentifier;
    union
    {
      StdIdentifier stdIdentifier;
      ExtIdentifier extIdentifier;
    };
    bool isRTR;

    uint8_t length;
    uint8_t content[8];
  };
  using MessageHandler = void(const MessageEvent *);

  static constexpr int MessageQueueSize = 16;
  static constexpr int ErrorQueueSize = 4;

public:
  static void start(MessageHandler * msgHandler = nullptr, ErrorHandler * errorHandler = nullptr);

  static void setReceiveFilter(StdIdentifier identifier, StdIdentifier mask);
  static void setReceiveFilter(ExtIdentifier identifier, ExtIdentifier mask);
  static void clearReceiveFilter(); // stop message reception entirely

  static MessageEvent * prepareMessage(); //disables interrupts until sendMessage() is called
  static bool commitMessage(MessageEvent * message); // message must be a pointer obtained through prepareMessage()

private:
  CAN() = default;

  static void onInterrupt();

  static void sendMessage(const MessageEvent * message);
  static void receiveMessage(MessageEvent * message);

  static void setMode(uint8_t mode);
  static void canCommand(uint8_t * command, uint8_t length);

private:
  static MessageEvent s_sendQueue[MessageQueueSize];
  static volatile uint8_t s_sendQueueFree;
  static volatile uint8_t s_sendQueueNext;
  static volatile bool s_sendPending;

  static MessageEvent s_receiveQueue[MessageQueueSize];
  static volatile uint8_t s_receiveQueueFree;
  static volatile uint8_t s_receiveQueueNext;
  static volatile bool s_handlingMessages;

  static ErrorEvent s_errorQueue[ErrorQueueSize];
  static volatile uint8_t s_errorQueueFree;
  static volatile uint8_t s_errorQueueNext;
  static volatile bool s_handlingErrors;

  static MessageHandler * s_messageHandler;
  static ErrorHandler * s_errorHandler;

  static uint8_t s_prepareMessageSREG;
};

