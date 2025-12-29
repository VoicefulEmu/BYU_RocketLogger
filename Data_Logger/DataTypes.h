// DataTypes.h
#pragma once
#include <Arduino.h>

struct LogSample {
  uint32_t t_us;

  // IMU (ICM-20948)
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float imuTempC;

  // Baro (BMP390)
  float baroTempC;
  float pressPa;
  float altM;      // optional; computed from sea-level pressure

  // H3LIS331 high-g accel (m/s^2)
  float hgx, hgy, hgz;
};
