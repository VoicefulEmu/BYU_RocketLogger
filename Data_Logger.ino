#include <Wire.h>
#include "IMU.h"
#include "Baro.h"
#include "HighG.h"
#include "SDCardLog.h"
#include "DataTypes.h"
#include "WebUI.h"

static const int SD_CS = 5;

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();

  imu_setup();
  baro_setup();
  highg_setup();
  sdlog_setup(SD_CS);

  webui_setup_sta_then_ap(
    "YourSSID", "YourPassword",
    "RocketLogger", "12345678",
    8000
  );



}

void loop() {
  LogSample s{};
  s.t_us = micros();

  bool ok1 = imu_read(s);
  bool ok2 = baro_read(s);
  bool ok3 = highg_read(s);

  if (ok1 && ok2 && ok3) {
    sdlog_write_csv(s);
  }
  webui_set_latest(s);   // update dashboard data
  webui_loop();            // keep HTTP responsive [web:521]
}

