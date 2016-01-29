#pragma once

#include <Arduino.h>

class Motorola
{
public:
  using Message = uint32_t;
  using MessageSpeed = bool; // bit time is (speed)? 104 : 208 us
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

  static Message oldTrainMessage(uint8_t address, bool function, uint8_t speedLevel);
//  static Message newTrainMessageDirection(uint8_t address, bool function, int8_t speed);
//  static Message newTrainMessageFunction(uint8_t address, bool function, int8_t speed, uint8_t nFunction, bool on);
//  static Message switchMessage(uint8_t address,);

  Motorola();

  void start();

  Message & message(uint8_t n);
  void enableMessage(uint8_t n);
  void disableMessage(uint8_t n);
  bool messageEnabled(uint8_t n);
  void setMessageSpeed(uint8_t n, MessageSpeed speed);
  void setMessageOneShot(uint8_t n, bool oneShot);

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

  bool m_running;

  uint8_t m_currentMsgNumber;
  bool m_state;
  uint8_t m_bitCounter;
  MessageSpeed m_currentSpeed;
  Message m_currentMessage;
};
