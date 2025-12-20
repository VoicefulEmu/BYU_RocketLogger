// WebUI.cpp
#include "WebUI.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>

static WebServer server(80);

static LogSample latest{};
static volatile bool have_latest = false;
static volatile bool is_logging = true;  // default: assume logging

static void handle_root() {
  // minimal HTML page that polls /latest
  const char* html =
    "<!doctype html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Rocket Logger</title></head><body>"
    "<h2>Rocket Logger</h2>"
    "<pre id='out'>loading...</pre>"
    "<p><a href='/log.csv'>Download log.csv</a></p>"
    "<script>"
    "async function tick(){"
    "  const r=await fetch('/latest');"
    "  document.getElementById('out').textContent=await r.text();"
    "}"
    "setInterval(tick,500); tick();"
    "</script></body></html>";

  server.send(200, "text/html", html);
}

static void handle_latest() {
  if (!have_latest) {
    server.send(200, "text/plain", "no data yet");
    return;
  }

  // plain text output (easy to read)
  String line;
  line.reserve(256);
  line += "t_us=" + String(latest.t_us) + "\n";
  line += "a(m/s^2)=" + String(latest.ax, 3) + "," + String(latest.ay, 3) + "," + String(latest.az, 3) + "\n";
  line += "g(rad/s)=" + String(latest.gx, 3) + "," + String(latest.gy, 3) + "," + String(latest.gz, 3) + "\n";
  line += "m(uT)=" + String(latest.mx, 2) + "," + String(latest.my, 2) + "," + String(latest.mz, 2) + "\n";
  line += "imuTempC=" + String(latest.imuTempC, 2) + "\n";
  line += "baroTempC=" + String(latest.baroTempC, 2) + " pressPa=" + String(latest.pressPa, 1) +
          " altM=" + String(latest.altM, 2) + "\n";
  line += "highG(m/s^2)=" + String(latest.hgx, 3) + "," + String(latest.hgy, 3) + "," + String(latest.hgz, 3) + "\n";

  server.send(200, "text/plain", line);
}

static void handle_log_download() {
  if (is_logging) {
    server.send(423, "text/plain", "Logging active; stop logging to download log.csv");
    return;
  }

  File f = SD.open("/log.csv", FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "log.csv not found");
    return;
  }

  // Stream the file to the client
  server.sendHeader("Content-Disposition", "attachment; filename=log.csv");
  server.streamFile(f, "text/csv");  // streams file contents over HTTP [web:542]
  f.close();
}

void webui_setup_sta_then_ap(const char* sta_ssid, const char* sta_pass,
                             const char* ap_ssid,  const char* ap_pass,
                             uint32_t timeout_ms) {
  // 1) Try STA first
  WiFi.mode(WIFI_STA);
  WiFi.begin(sta_ssid, sta_pass);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("STA connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    // 2) Fallback to AP
    WiFi.disconnect(true);         // stop STA attempt (optional)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_pass);

    Serial.print("STA failed; started AP SSID: ");
    Serial.println(ap_ssid);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }

  // 3) Start web server either way
  server.on("/", HTTP_GET, handle_root);
  server.on("/latest", HTTP_GET, handle_latest);
  server.on("/log.csv", HTTP_GET, handle_log_download);
  server.begin();
}



void webui_loop() {
  server.handleClient();  // must be called regularly to process requests [web:542]
}

void webui_set_latest(const LogSample& s) {
  latest = s;
  have_latest = true;
}

void webui_set_logging(bool logging) {
  is_logging = logging;
}
