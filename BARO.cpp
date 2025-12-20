#include "Baro.h"

#include <Wire.h>
#include <Adafruit_BMP3XX.h>

static Adafruit_BMP3XX bmp;
static const float SEA_LEVEL_HPA = 1013.25f;  // adjust later for better altitude

void baro_setup() {
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 not found");
    while (1) delay(10);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

bool baro_read(LogSample &out) {
  if (!bmp.performReading()) return false;

  out.baroTempC = bmp.temperature;
  out.pressPa   = bmp.pressure;
  out.altM      = bmp.readAltitude(SEA_LEVEL_HPA);
  return true;
}
