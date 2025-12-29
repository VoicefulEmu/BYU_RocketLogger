#pragma once
#include "DataTypes.h"

void mavlink_setup();
void mavlink_send_sample(const LogSample &s);
