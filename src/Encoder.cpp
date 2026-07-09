#include "Encoder.h"
#include <Wire.h>

RailEncoder railEncoder;

bool RailEncoder::begin(int sdaPin, int sclPin, float mmPerRevolution, uint8_t direction) {
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(100000);   // Start conservative.
  Wire.setTimeOut(5);      // ESP32 Wire timeout in ms.

  mmPerTick_ = mmPerRevolution / 4096.0f;

  if (!sensor_.begin()) {
    connected_ = false;
    return false;
  }

  sensor_.setDirection(direction);
  sensor_.setPowerMode(AS5600_POWERMODE_NOMINAL);
  sensor_.setWatchDog(AS5600_WATCHDOG_OFF);

  connected_ = true;
  lastPollMs_ = millis();
  return true;
}

void RailEncoder::update() {
  unsigned long now = millis();
  if (now - lastPollMs_ < POLL_INTERVAL_MS) {
    return;
  }
  lastPollMs_ = now;

  rawTicks_ = sensor_.getCumulativePosition(true);
  lastError_ = sensor_.lastError();
  connected_ = (lastError_ == AS5600_OK);

  if (connected_) {
    positionMm_ = rawTicks_ * mmPerTick_;
    int32_t wrapped = ((rawTicks_ % 4096) + 4096) % 4096;
    angleDeg_ = wrapped * AS5600_RAW_TO_DEGREES;
  }
}

void RailEncoder::zero() {
  sensor_.resetCumulativePosition(0);
  rawTicks_ = 0;
  positionMm_ = 0.0f;
}

bool RailEncoder::isConnected() const {
  return connected_;
}

int RailEncoder::lastError() const {
  return lastError_;
}

float RailEncoder::positionMm() const {
  return positionMm_;
}

int32_t RailEncoder::rawTicks() const {
  return rawTicks_;
}

float RailEncoder::rawAngleDeg() const {
  return angleDeg_;
}
