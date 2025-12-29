#include "SDCardLog.h"

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

static File logFile;
static String currentLogName;

// Helper: check if a file exists
static bool file_exists(const char *path) {
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  f.close();
  return true;
}

// Helper: choose next free filename: /log_0001.csv ... /log_9999.csv
static String make_next_log_filename() {
  const uint16_t maxRuns = 9999;

  for (uint16_t i = 1; i <= maxRuns; ++i) {
    char name[20];
    snprintf(name, sizeof(name), "/log_%04u.csv", i);
    if (!file_exists(name)) {
      return String(name);
    }
  }

  // Fallback if all taken
  return String("/log_last.csv");
}

void sdlog_setup(int cs_pin) {
  if (!SD.begin(cs_pin)) {
    Serial.println("SD init failed");
    while (1) delay(1000);
  }

  String fname = make_next_log_filename();
  currentLogName = fname;

  Serial.print("Opening log file: ");
  Serial.println(fname);

  logFile = SD.open(fname.c_str(), FILE_WRITE);
  if (!logFile) {
    Serial.println("Failed to open log file");
    while (1) delay(1000);
  }

  // CSV header
  logFile.println("t_us,ax,ay,az,gx,gy,gz,mx,my,mz,imuTempC,baroTempC,pressPa,altM,hgx,hgy,hgz");
  logFile.flush();  // ok at start [file:851]
}
String sdlog_get_current_name() {   // << add getter
  return currentLogName;
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
  if (++n >= 60) {  // flush every ~60 samples
    n = 0;
    logFile.flush();
  }
}
