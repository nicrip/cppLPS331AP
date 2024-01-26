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

private:
  int gpio_commander;
  int i2c_handle;
  char address;
  char buf[10];
  char rbuf[10];
  float temperature;
  float pressure;
  float altitude;

  float fluidDensity;

  /** Performs calculations per the sensor data sheet for conversion and
   *  second order compensation.
   */
  void calculate();

  uint8_t crc4(uint16_t n_prom[]);
};

#endif
