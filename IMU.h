#pragma once
#include "DataTypes.h"

void imu_setup();
bool imu_read(LogSample &out);   // fills IMU fields (doesn't touch baro fields)
