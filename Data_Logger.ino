#include <Wire.h>
#include "secrets.h"
#include "IMU.h"
#include "Baro.h"
#include "HighG.h"
#include "SDCardLog.h"
#include "DataTypes.h"
#include "WebUI.h"

static const int SD_CS = 5;

// target logging rate
static const uint32_t LOG_PERIOD_US = 33333;   // 33.3 ms = 30 Hz
static uint32_t next_log_time_us = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();

  imu_setup();
  baro_setup();
  highg_setup();
  sdlog_setup(SD_CS);

  webui_setup_sta_then_ap(
    WIFI_SSID, WIFI_PASSWORD,
    "RocketLogger", "12345678",
    8000
  );

  webui_set_logging(false);
  webui_set_state(STATE_IDLE);   // boot in IDLE

  next_log_time_us = micros();   // initialize scheduler
}

void loop() {
  const uint32_t now = micros();

  // Always service web requests
  webui_loop();

  // Only proceed when it's time for the next sample
  if ((int32_t)(now - next_log_time_us) < 0) {
    return;   // not yet time; come back next loop
  }
  next_log_time_us += LOG_PERIOD_US;   // schedule next slot

  // Build one sample
  LogSample s{};
  s.t_us = now;

  bool ok1 = imu_read(s);
  bool ok2 = baro_read(s);
  bool ok3 = highg_read(s);

  LoggerState st = webui_get_state();

  if (ok1 && ok2 && ok3) {
    if (st == STATE_LOGGING) {
      sdlog_write_csv(s);        // fixed-rate logging
    }
    webui_set_latest(s);         // live display still updates at log rate
  }
}
