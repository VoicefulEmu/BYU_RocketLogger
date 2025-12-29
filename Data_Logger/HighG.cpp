#include "HighG.h"

#include <Wire.h>
#include <Adafruit_LIS331.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>

static Adafruit_H3LIS331 lis;   // specifically for H3LIS331

void highg_setup() {
  // Default address is 0x18; use 0x19 if SDO->3Vo. [web:461]
  if (!lis.begin_I2C(0x18, &Wire)) {
    Serial.println("H3LIS331 not found");
    while (1) delay(10);
  }

  // Pick range appropriate for your expected launch/boost G’s
  lis.setRange(H3LIS331_RANGE_200_G);  // common choice for rockets [web:454]
  lis.setDataRate(LIS331_DATARATE_400_HZ); // optional; higher internal ODR [web:456]
}

bool highg_read(LogSample &out) {
  sensors_event_t e;
  lis.getEvent(&e); // returns acceleration in m/s^2 [web:456]

  out.hgx = e.acceleration.x;
  out.hgy = e.acceleration.y;
  out.hgz = e.acceleration.z;
  return true;
}
