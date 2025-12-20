#pragma once
#include "DataTypes.h"

void sdlog_setup(int cs_pin);
void sdlog_write_csv(const LogSample &s);
