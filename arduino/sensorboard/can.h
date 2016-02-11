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

  static CAN inst();

  void start(CANAddress address, CANAddress mask);
  void setMsgHandler(CANMsgHandler * handler);
  void setRTRHandler(CANRTRHandler * handler);
  void setErrorHandler(CANErrorHandler * handler);
  void send(CANAddress address, uint32_t timestamp, uint32_t duration);
  void sendRTR(CANAddress address);
  bool pendingTransmission();

  void processEvents(byte eventMask = 0xFF);

  void onCANEvent();
  
  CAN();
  
private:
  void canCommand(byte * command, int length);

private:
  CANAddress m_address;
  CANAddress m_mask;

  volatile bool m_transmitPending;
  volatile byte m_eventsPending;
  volatile byte m_errorsPending;

  CANMsgHandler * m_msgHandler;
  CANRTRHandler * m_rtrHandler;
  CANErrorHandler * m_errorHandler;
};

