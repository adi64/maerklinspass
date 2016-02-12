#pragma once

#include <Arduino.h>

#include <SPI.h>


class CAN
{
  static const SPISettings SPIConfig;

public:
  using CANAddress = uint16_t;
  using CANMsgHandler = void(CANAddress, uint32_t, uint32_t);
  using CANRTRHandler = void(CANAddress);
  using CANErrorHandler = void(byte);

  static constexpr int PinNCS = 10;
  static constexpr int PinNInt = 2;
  static constexpr int PinMISO = 12;
  static constexpr int PinMOSI = 11;
  static constexpr int PinSCK = 13;

  static void start(CANAddress address, CANAddress mask);
  static void setMsgHandler(CANMsgHandler * handler);
  static void setRTRHandler(CANRTRHandler * handler);
  static void setErrorHandler(CANErrorHandler * handler);
  static void send(CANAddress address, uint32_t timestamp, uint32_t duration);
  static void sendRTR(CANAddress address);
  static bool pendingTransmission();

  static void processEvents(byte eventMask = 0xFF);

  static void onCANEvent();

private:
  CAN() = default;
  static void canCommand(byte * command, int length);

private:
  static CAN::CANAddress m_address;
  static CAN::CANAddress m_mask;

  static volatile bool m_transmitPending;
  static volatile byte m_eventsPending;
  static volatile byte m_errorsPending;

  static CANMsgHandler * m_msgHandler;
  static CANRTRHandler * m_rtrHandler;
  static CANErrorHandler * m_errorHandler;
};

