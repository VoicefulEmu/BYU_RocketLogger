// WebUI.h
#pragma once
#include "DataTypes.h"

void webui_setup_sta_then_ap(const char* sta_ssid, const char* sta_pass,
                             const char* ap_ssid,  const char* ap_pass,
                             uint32_t timeout_ms = 8000);
void webui_loop();

void webui_set_latest(const LogSample& s);

// Optional: to disable SD download while logging
void webui_set_logging(bool is_logging);
