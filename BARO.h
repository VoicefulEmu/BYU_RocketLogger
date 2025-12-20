#pragma once
#include "DataTypes.h"

void baro_setup();
bool baro_read(LogSample &out);   // fills baro fields
