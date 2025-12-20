#include "IMU.h"

#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

static Adafruit_ICM20948 icm;

void imu_setup() {
  // Do Wire.begin() once in main (recommended), but it's OK here too if you want.
  if (!icm.begin_I2C(0x69, &Wire)) {
    Serial.println("ICM20948 not found");
    while (1) delay(10);
  }

  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
}

bool imu_read(LogSample &out) {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  out.ax = accel.acceleration.x;
  out.ay = accel.acceleration.y;
  out.az = accel.acceleration.z;

  out.gx = gyro.gyro.x;
  out.gy = gyro.gyro.y;
  out.gz = gyro.gyro.z;

  out.mx = mag.magnetic.x;
  out.my = mag.magnetic.y;
  out.mz = mag.magnetic.z;

  out.imuTempC = temp.temperature;
  return true;
}
