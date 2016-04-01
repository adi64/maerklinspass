#pragma once

#include <Arduino.h>

class Motorola
{
public:
  using Message = uint32_t;
  using MessageSpeed = bool; // bit time is (speed)? 104 : 208 us
  using MessageBufferMask = uint8_t;

  static constexpr uint8_t MessageBufferSize = 8;

  static constexpr uint8_t IdleAddress = 81;
  static constexpr Message IdleMessage = 0x00055;
  static constexpr Message IdleSpeed = false;

  static constexpr int PinData = 9;
  static constexpr int PinGo = 4;
  static constexpr int PinError = 3;

  static constexpr uint8_t BitCountMsg = 18;
  static constexpr uint8_t BitCountGap = 6;
  static constexpr uint8_t BitCountWait = 22;

  //TODO newTrainMessageFunction()
  //TODO newTrainMessageDirection()
  static Message oldTrainMessage(uint8_t address, bool function, uint8_t speedLevel);
  static Message switchMessage(uint8_t decoderAddress, uint8_t switchAddress, bool state);

  static void start();

  static void setMessage(uint8_t n, Message message);
  static Message getMessage(uint8_t n);
  static void enableMessage(uint8_t n);
  static void disableMessage(uint8_t n);
  static bool messageEnabled(uint8_t n);
  static void setMessageSpeed(uint8_t n, MessageSpeed speed);
  static void setMessageOneShot(uint8_t n, bool oneShot);

  static void onTimerOverflow();
  static void onErrorPin();

private:
  Motorola() = default;

  static void loadNextMessage();

private:
  static Message s_msgBuffer[MessageBufferSize];

  static MessageBufferMask s_msgEnabled;
  static MessageBufferMask s_msgSpeed;
  static MessageBufferMask s_msgOneShot;

  static bool s_running;

  static uint8_t s_currentMsgNumber;
  static bool s_state;
  static uint8_t s_bitCounter;
  static MessageSpeed s_currentSpeed;
  static Message s_currentMessage;
};
