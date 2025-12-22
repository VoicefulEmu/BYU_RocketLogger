#pragma once
#include "DataTypes.h"

enum LoggerState {
  STATE_IDLE = 0,
  STATE_LOGGING,
  STATE_STOPPED,
  STATE_DONE
};

void webui_setup_sta_then_ap(const char* sta_ssid, const char* sta_pass,
                             const char* ap_ssid,  const char* ap_pass,
                             uint32_t timeout_ms = 8000);

void webui_loop();

void webui_set_latest(const LogSample& s);

void webui_set_logging(bool is_logging);
bool webui_get_logging();

// New: state helpers
void webui_set_state(LoggerState s);
LoggerState webui_get_state();
