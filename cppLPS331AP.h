#include <cstdint>
#include "pigpiod_if2.h"

#ifndef LPS331AP_H
#define LPS331AP_H

class LPS331AP {
public:
  LPS331AP();
  bool init();
  void enableDefault();
  bool checkStatus();
  int32_t readPressureRaw();
  float readPressureMillibars();
  float readPressureInchesHg();
  int16_t readTemperatureRaw();
  float readTemperatureCelsius();
  float readTemperatureFahrenheit();
  float pressureToAltitudeMeters(float pressure_mbar, float altimeter_setting_mbar = 1013.25);
  float pressureToAltitudeFeet(float pressure_inHg, float altimeter_setting_inHg = 29.9213);

private:
  int gpio_commander;
  int i2c_handle;
  char address;
  char buf[10];
  char rbuf[10];
  float temperature;
  float pressure;
  float altitude;
};

#endif
