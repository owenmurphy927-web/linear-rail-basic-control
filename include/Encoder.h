#pragma once
#include <Arduino.h>
#include <AS5600.h>

class RailEncoder {
public:
  bool begin(int sdaPin, int sclPin, float mmPerRevolution,
             uint8_t direction = AS5600_CLOCK_WISE);
  void update();          // internally throttled; safe to call every loop()
  void zero();            // redefine current physical position as origin

  bool    isConnected() const;
  int     lastError() const;
  float   positionMm() const;
  int32_t rawTicks() const;
  float   rawAngleDeg() const;

private:
  static const unsigned long POLL_INTERVAL_MS = 10;

  AS5600 sensor_;
  float mmPerTick_ = 0.0f;
  float positionMm_ = 0.0f;
  float angleDeg_ = 0.0f;
  int32_t rawTicks_ = 0;
  int lastError_ = 0;
  bool connected_ = false;
  unsigned long lastPollMs_ = 0;
};

extern RailEncoder railEncoder;
