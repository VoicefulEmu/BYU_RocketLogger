#pragma once
#include "DataTypes.h"
#include <Arduino.h>

void sdlog_setup(int cs_pin);
void sdlog_write_csv(const LogSample &s);
String sdlog_get_current_name();     // << declare
