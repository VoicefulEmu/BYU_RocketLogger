#include "SDCardLog.h"

#include <SPI.h>
#include <SD.h>
#include <Arduino.h>

static File logFile;

void sdlog_setup(int cs_pin) {
  if (!SD.begin(cs_pin)) {
    Serial.println("SD init failed");
    while (1) delay(1000);
  }

  logFile = SD.open("/log.csv", FILE_WRITE);
  if (!logFile) {
    Serial.println("Failed to open /log.csv");
    while (1) delay(1000);
  }

  logFile.println("t_us,ax,ay,az,gx,gy,gz,mx,my,mz,imuTempC,baroTempC,pressPa,altM,hgx,hgy,hgz");
  logFile.flush(); // ok at start [web:315]
}

void sdlog_write_csv(const LogSample &s) {
  logFile.print(s.t_us); logFile.print(',');

  logFile.print(s.ax, 6); logFile.print(',');
  logFile.print(s.ay, 6); logFile.print(',');
  logFile.print(s.az, 6); logFile.print(',');

  logFile.print(s.gx, 6); logFile.print(',');
  logFile.print(s.gy, 6); logFile.print(',');
  logFile.print(s.gz, 6); logFile.print(',');

  logFile.print(s.mx, 3); logFile.print(',');
  logFile.print(s.my, 3); logFile.print(',');
  logFile.print(s.mz, 3); logFile.print(',');

  logFile.print(s.imuTempC, 2); logFile.print(',');
  logFile.print(s.baroTempC, 2); logFile.print(',');
  logFile.print(s.pressPa, 2); logFile.print(',');
  logFile.print(s.altM, 3); logFile.print(',');

  logFile.print(s.hgx, 6); logFile.print(',');
  logFile.print(s.hgy, 6); logFile.print(',');
  logFile.println(s.hgz, 6);

  static uint16_t n = 0;
  if (++n >= 60) { n = 0; logFile.flush(); }  // periodic flush [web:315]
}
