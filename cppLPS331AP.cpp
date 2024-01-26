#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>
#include "cppLPS331AP.h"

using namespace std::chrono;

#define SA0_LOW_ADDRESS                 0b1011100
#define SA0_HIGH_ADDRESS                0b1011101
#define LPS331AP_WHO_ID                 0xBB

#define LPS331AP_AUTO_INC               0x80
#define LPS331AP_REG_REF_P_XL           0x08
#define LPS331AP_REG_REF_P_L            0x09
#define LPS331AP_REG_REF_P_H            0x0a
#define LPS331AP_REG_WHO_AM_I           0x0f
#define LPS331AP_REG_RES_CONF           0x10
#define LPS331AP_REG_CTRL_REG1          0x20
#define LPS331AP_REG_CTRL_REG2          0x21
#define LPS331AP_REG_CTRL_REG3          0x22
#define LPS331AP_REG_INT_CFG_REG        0x23
#define LPS331AP_REG_INT_SOURCE_REG     0x24
#define LPS331AP_REG_THS_P_LOW_REG      0x25
#define LPS331AP_REG_THS_P_HIGH_REG     0x26
#define LPS331AP_REG_STATUS_REG         0x27
#define LPS331AP_REG_PRESS_OUT_XL       0x28
#define LPS331AP_REG_PRESS_OUT_L        0x29
#define LPS331AP_REG_PRESS_OUT_H        0x2a
#define LPS331AP_REG_TEMP_OUT_L         0x2b
#define LPS331AP_REG_TEMP_OUT_H         0x2c
#define LPS331AP_REG_AMP_CTRL           0x30

#define LPS331AP_CTRL_REG1_PD           0x80
#define LPS331AP_CTRL_REG1_ODR          0x70
#define LPS331AP_CTRL_REG1_ODR_POS      4
#define LPS331AP_CTRL_REG1_DIFF_EN      0x08
#define LPS331AP_CTRL_REG1_DBDU         0x04
#define LPS331AP_CTRL_REG1_DELTA_EN     0x02
#define LPS331AP_CTRL_REG1_SIM          0x01

LPS331AP::LPS331AP() {
    // Pololu board pulls SA0 high by default
    address = SA0_HIGH_ADDRESS;
}

bool LPS331AP::init() {
    gpio_commander = pigpio_start(NULL, NULL);								// start PiGPIO object on local computer
    i2c_handle = i2c_open(gpio_commander, 1, address, 0);
    if (i2c_handle < 0) {
        std::cout << "i2c bus could not be opened... Exiting." << std::endl;
        exit(0);
    }
}

void LPS331AP::enableDefault() {
    buf[0] = 0x00;
    i2c_write_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_CTRL_REG1, buf, 1);
    buf[0] = 0x79;
    i2c_write_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_RES_CONF, buf, 1);
    buf[0] = 0xE0;
    i2c_write_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_CTRL_REG1, buf, 1);
}

bool LPS331AP::checkStatus() {
    i2c_read_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_STATUS_REG, rbuf, 1);
    if (rbuf == 0) {
        return false;
    } else {
        return true;
    }
}

int32_t LPS331AP::readPressureRaw() {
    buf[0] = (LPS331AP_REG_PRESS_OUT_XL | (1 << 7));
    i2c_write_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_PRESS_OUT_XL, buf, 1);
    i2c_read_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_PRESS_OUT_XL, &rbuf[0], 1);
    i2c_read_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_PRESS_OUT_L, &rbuf[1], 1);
    i2c_read_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_PRESS_OUT_H, &rbuf[2], 1);

    // combine chars
    int32_t raw_pressure = (int32_t)(int8_t)rbuf[2] << 16 | (uint16_t)rbuf[1] << 8 | rbuf[0];
    return raw_pressure;
}

float LPS331AP::readPressureMillibars()
{
    float mbar_pressure = (float)readPressureRaw()/4096;
    return mbar_pressure;
}

float LPS331AP::readPressureInchesHg()
{
    float inHg_pressure = (float)readPressureRaw()/138706.5;
    return inHg_pressure; 
}

int16_t LPS331AP::readTemperatureRaw()
{
    buf[0] = (LPS331AP_REG_TEMP_OUT_L | (1 << 7));
    i2c_write_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_TEMP_OUT_L, buf, 1);
    i2c_read_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_TEMP_OUT_L, &rbuf[0], 1);
    i2c_read_i2c_block_data(gpio_commander, i2c_handle, LPS331AP_REG_TEMP_OUT_H, &rbuf[1], 1);

    // combine chars
    int32_t raw_temperature = (int16_t)rbuf[1] << 8 | rbuf[0];
    return raw_temperature;
}

float LPS331AP::readTemperatureCelsius()
{
    float celsius_temperature = 42.5 + (float)readTemperatureRaw()/480;
    return celsius_temperature;
}

// reads temperature in degrees F
float LPS331AP::readTemperatureFahrenheit()
{
    float fahrenheit_temperature = 108.5 + (float)readTemperatureRaw()/480 * 1.8;
    return fahrenheit_temperature;
}


int main(int argc, char *argv[])
{
  LPS331AP lps331ap;
  lps331ap.init();
  lps331ap.enableDefault();
  while(1) {
    if (lps331ap.checkStatus()) {
        std::cout << lps331ap.readPressureMillibars() << " | " << lps331ap.readPressureInchesHg() << std::endl;
        std::cout << lps331ap.readTemperatureCelsius() << " | " << lps331ap.readTemperatureFahrenheit() << std::endl;
    }
  }
// 	ms5837.setModel(MS5837::MS5837_30BA);
//   ms5837.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
//   ms5837.setOverSampling(5);
// 	while(1) {
// 		ms5837.read();

// 		std::cout << "Pressure: " << ms5837.pressure() << " mbar" << std::endl;
// 		std::cout << "Temperature: " << ms5837.temperature() << " deg C" << std::endl;
// 		std::cout << "Depth: " << ms5837.depth() << " m" << std::endl;
// 		std::cout << "Altitude: " << ms5837.altitude() << " m above mean sea level" << std::endl << std::endl;
// 	}
}