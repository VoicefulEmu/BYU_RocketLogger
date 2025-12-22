// WebUI.cpp
#include "WebUI.h"
#include "SDCardLog.h"   // << so we can call sdlog_get_current_name()

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>

static WebServer server(80);

static LogSample latest{};
static volatile bool have_latest = false;
static volatile bool is_logging = true;  // default: assume logging
static LoggerState current_state = STATE_IDLE;


static void handle_root() {
  const char* html =
    "<!doctype html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>RocketLogger</title>"
    "<style>"
    "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;"
      "margin:0;padding:1rem;background:#0b1020;color:#e9edf7;}"
    "h1{margin:0 0 .5rem;font-size:1.6rem;}"
    "h2{font-size:1.1rem;margin:.5rem 0;opacity:.9;}"
    ".top-bar{display:flex;flex-wrap:wrap;align-items:center;gap:.5rem;"
      "justify-content:space-between;margin-bottom:1rem;}"
    ".pill{background:#192040;border-radius:999px;padding:.25rem .75rem;"
      "font-size:.8rem;opacity:.85;}"
      ".btn{display:inline-block;background:#2563eb;color:#fff;text-decoration:none;"
      "padding:.4rem .9rem;border-radius:.4rem;font-size:.85rem;border:none;cursor:pointer;}"
      ".btn:hover{background:#1d4ed8;}"
      ".btn.btn-stop{background:#dc2626;}"
      ".btn.btn-stop:hover{background:#b91c1c;}"
    "a.btn{display:inline-block;background:#2563eb;color:#fff;text-decoration:none;"
      "padding:.4rem .9rem;border-radius:.4rem;font-size:.85rem;}"
    "a.btn:hover{background:#1d4ed8;}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));"
      "gap:.75rem;margin-bottom:1rem;}"
    ".card{background:#111827;border:1px solid #1f2937;border-radius:.6rem;"
      "padding:.6rem .75rem;box-shadow:0 8px 18px rgba(0,0,0,.35);}"
    ".label{font-size:.75rem;text-transform:uppercase;letter-spacing:.06em;"
      "opacity:.7;margin-bottom:.25rem;}"
    ".value{font-size:1rem;font-weight:500;}"
    ".units{font-size:.75rem;opacity:.7;margin-left:.25rem;}"
    "table{width:100%;border-collapse:collapse;font-size:.8rem;margin-top:.25rem;}"
    "th,td{border-bottom:1px solid #1f2937;padding:.25rem .3rem;text-align:right;}"
    "th{text-align:left;font-weight:500;opacity:.8;font-size:.75rem;}"
    "tr:last-child td{border-bottom:none;}"
    ".muted{opacity:.7;font-size:.75rem;margin-top:.5rem;}"
    ".status-ok{color:#4ade80;}"
    ".status-bad{color:#f97373;}"
    "</style>"
    "</head><body>"
    "<div class='top-bar'>"
      "<div>"
        "<h1>RocketLogger</h1>"
        "<div class='pill'>Live sensor snapshot (last sample)</div>"
      "</div>"
      "<div style='display:flex;gap:.4rem;flex-wrap:wrap;'>"
        "<button class='btn' onclick='startLogging()'>Start logging</button>"
        "<button class='btn btn-stop' onclick='stopLogging()'>Stop logging</button>"
        "<button class='btn btn-stop' onclick='markDone()'>Mark DONE</button>"
        "<a class='btn' href='/log.csv'>Download log.csv</a>"
      "</div>"
    "</div>"


    "<div class='grid'>"
      "<div class='card'>"
        "<div class='label'>Timestamp</div>"
        "<div class='value'>"
          "<span id='t_us'>–</span><span class='units'>&micro;s</span>"
        "</div>"
        "<div class='value'>"
          "<span id='t_hms'>–</span><span class='units'> (hh:mm:ss)</span>"
        "</div>"
      "</div>"


      "<div class='card'>"
        "<div class='label'>Barometer</div>"
        "<div class='value'>T=<span id='baroTempC'>–</span><span class='units'>&deg;C</span></div>"
        "<div class='value'>P=<span id='pressPa'>–</span><span class='units'>Pa</span></div>"
        "<div class='value'>Alt=<span id='altM'>–</span><span class='units'>m</span></div>"
      "</div>"

      "<div class='card'>"
        "<div class='label'>IMU Temperature</div>"
        "<div class='value'><span id='imuTempC'>–</span><span class='units'>&deg;C</span></div>"
      "</div>"

      "<div class='card'>"
        "<div class='label'>Status</div>"
        "<div class='value'><span id='status' class='status-bad'>Connecting…</span></div>"
      "</div>"

      "<div class='card'>"
        "<div class='label'>Sample rate</div>"
        "<div class='value'><span id='rateHz'>–</span><span class='units'> Hz</span></div>"
      "</div>"

    "</div>"
    

    "<div class='card'>"
      "<h2>IMU Accel / Gyro / Mag</h2>"
      "<table>"
        "<tr><th>Axis</th><th>Accel (m/s²)</th><th>Gyro (rad/s)</th><th>Mag (µT)</th></tr>"
        "<tr><td>X</td><td id='ax'>–</td><td id='gx'>–</td><td id='mx'>–</td></tr>"
        "<tr><td>Y</td><td id='ay'>–</td><td id='gy'>–</td><td id='my'>–</td></tr>"
        "<tr><td>Z</td><td id='az'>–</td><td id='gz'>–</td><td id='mz'>–</td></tr>"
      "</table>"
    "</div>"

    "<div class='card'>"
      "<h2>High‑G Accelerometer</h2>"
      "<table>"
        "<tr><th>Axis</th><th>Accel (m/s²)</th></tr>"
        "<tr><td>X</td><td id='hgx'>–</td></tr>"
        "<tr><td>Y</td><td id='hgy'>–</td></tr>"
        "<tr><td>Z</td><td id='hgz'>–</td></tr>"
      "</table>"
    "</div>"

    "<div class='muted'>"
      "Page refreshes from <code>/latest</code> every 500&nbsp;ms. "
      "Download complete log as CSV with the button above."
    "</div>"

    "<script>"
    "function parseLatest(text){"
      "const out={};"
      "text.split('\\n').forEach(line=>{"
        "if(!line.trim()) return;"
        "const parts=line.split('=');"
        "if(parts.length<2) return;"
        "const key=parts[0].trim();"
        "const val=parts[1].trim();"
        "out[key]=val;"
      "});"
      "return out;"
    "}"

    "function setText(id,val){"
      "const el=document.getElementById(id);"
      "if(el) el.textContent=val;"
    "}"
    
    "async function markDone(){"
      "try{"
        "const r = await fetch('/done',{cache:'no-cache'});"
        "const msg = await r.text();"
        "setText('status','DONE');"
        "document.getElementById('status').className='status-bad';"
        "console.log(msg);"
      "}catch(e){"
        "setText('status','Error marking DONE');"
        "document.getElementById('status').className='status-bad';"
      "}"
    "}"

    "let last_t_us = null;"
    "let last_rate_hz = null;"


    "async function tick(){"
      "try{"
        "const r=await fetch('/latest',{cache:'no-cache'});"
        "if(!r.ok){"
          "setText('status','No data');"
          "document.getElementById('status').className='status-bad';"
          "return;"
        "}"
        "const txt=await r.text();"
        "const d=parseLatest(txt);"

        "if(txt.indexOf('no data yet')!==-1){"
          "setText('status','Waiting for first sample…');"
          "document.getElementById('status').className='status-bad';"
          "return;"
        "}"

        "setText('t_us', d['t_us']||'–');"
          "if(d['t_us']){"
            "const t_us = Number(d['t_us']);"
            "const t_sec = t_us / 1e6;"
            "const hours = Math.floor(t_sec / 3600);"
            "const minutes = Math.floor((t_sec % 3600) / 60);"
            "const seconds = Math.floor(t_sec % 60);"
            "const hh = String(hours).padStart(2, '0');"
            "const mm = String(minutes).padStart(2, '0');"
            "const ss = String(seconds).padStart(2, '0');"
            "setText('t_hms', hh + ':' + mm + ':' + ss);"
            "if (last_t_us !== null) {"
              "const dt_us = t_us - last_t_us;"
              "if (dt_us > 0) {"
                "const rate_hz = 1e6 / dt_us;"
                "last_rate_hz = rate_hz;"
                "setText('rateHz', rate_hz.toFixed(1));"
              "}"
            "}"
            "last_t_us = t_us;"
          "} else {"
            "setText('rateHz', '–');"
        "}"
        
        "if(d['a(m/s^2)']){"
          "const a=d['a(m/s^2)'].split(',');"
          "setText('ax',a[0]||'–');"
          "setText('ay',a[1]||'–');"
          "setText('az',a[2]||'–');"
        "}"
        "if(d['g(rad/s)']){"
          "const g=d['g(rad/s)'].split(',');"
          "setText('gx',g[0]||'–');"
          "setText('gy',g[1]||'–');"
          "setText('gz',g[2]||'–');"
        "}"
        "if(d['m(uT)']){"
          "const m=d['m(uT)'].split(',');"
          "setText('mx',m[0]||'–');"
          "setText('my',m[1]||'–');"
          "setText('mz',m[2]||'–');"
        "}"

        "if(d['imuTempC']) setText('imuTempC', d['imuTempC']);"

        "if (d['baroTempC']) setText('baroTempC', d['baroTempC']);"
        "if (d['pressPa'])   setText('pressPa',   d['pressPa']);"
        "if (d['altM'])      setText('altM',      d['altM']);"


        "if(d['highG(m/s^2)']){"
          "const h=d['highG(m/s^2)'].split(',');"
          "setText('hgx',h[0]||'–');"
          "setText('hgy',h[1]||'–');"
          "setText('hgz',h[2]||'–');"
        "}"

        "if(d['state']){"
          "setText('status', d['state']);"
          "const el = document.getElementById('status');"
          "if(d['state'] === 'LOGGING') el.className = 'status-ok';"
          "else el.className = 'status-bad';"
        "}"

      "}catch(e){"
        "setText('status','Error');"
        "document.getElementById('status').className='status-bad';"
      "}"
    "}"

    "async function startLogging(){"
      "try{"
        "const r = await fetch('/start',{cache:'no-cache'});"
        "const msg = await r.text();"
        "setText('status','Logging ON');"
        "document.getElementById('status').className='status-ok';"
        "console.log(msg);"
      "}catch(e){"
        "setText('status','Error starting logging');"
        "document.getElementById('status').className='status-bad';"
      "}"
    "}"

    "async function stopLogging(){"
      "try{"
        "const r = await fetch('/stop',{cache:'no-cache'});"
        "const msg = await r.text();"
        "setText('status','Logging OFF');"
        "document.getElementById('status').className='status-bad';"
        "console.log(msg);"
      "}catch(e){"
        "setText('status','Error stopping logging');"
        "document.getElementById('status').className='status-bad';"
      "}"
    "}"
   
    "setInterval(tick,500);"
    "tick();"
    "</script>"

    "</body></html>";

  server.send(200, "text/html", html);
}


static void handle_latest() {
  if (!have_latest) {
    server.send(200, "text/plain", "no data yet");
    return;
  }

  String line;
  line.reserve(256);
  line += "t_us=" + String(latest.t_us) + "\n";
  line += "a(m/s^2)=" + String(latest.ax, 3) + "," + String(latest.ay, 3) + "," + String(latest.az, 3) + "\n";
  line += "g(rad/s)=" + String(latest.gx, 3) + "," + String(latest.gy, 3) + "," + String(latest.gz, 3) + "\n";
  line += "m(uT)=" + String(latest.mx, 2) + "," + String(latest.my, 2) + "," + String(latest.mz, 2) + "\n";
  line += "imuTempC=" + String(latest.imuTempC, 2) + "\n";

  // NEW: three simple baro lines
  line += "baroTempC=" + String(latest.baroTempC, 2) + "\n";
  line += "pressPa="   + String(latest.pressPa,   1) + "\n";
  line += "altM="      + String(latest.altM,      2) + "\n";

  line += "highG(m/s^2)=" + String(latest.hgx, 3) + "," + String(latest.hgy, 3) + "," + String(latest.hgz, 3) + "\n";

  // Add state here
  line += "state=";
  switch (current_state) {
    case STATE_IDLE:    line += "IDLE"; break;
    case STATE_LOGGING: line += "LOGGING"; break;
    case STATE_STOPPED: line += "STOPPED"; break;
    case STATE_DONE:    line += "DONE"; break;
  }
  line += "\n";

  server.send(200, "text/plain", line);
}



static void handle_log_download() {
  if (is_logging) {
    server.send(423, "text/plain",
                "Logging active; stop logging to download log file");
    return;
  }

  String name = sdlog_get_current_name();
  File f = SD.open(name.c_str(), FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "Log file not found");
    return;
  }

  server.sendHeader("Content-Disposition",
                    "attachment; filename=" + name.substring(1)); // drop leading '/'
  server.streamFile(f, "text/csv");
  f.close();
}


static void handle_start_logging() {
  // Only allow starting if we are IDLE or STOPPED
  if (current_state == STATE_IDLE || current_state == STATE_STOPPED) {
    is_logging = true;
    current_state = STATE_LOGGING;
    server.send(200, "text/plain",
                "Logging started; new samples will be appended to the current log file.");
  } else {
    server.send(409, "text/plain", "Cannot start from this state.");
  }
}

static void handle_stop_logging() {
  if (current_state == STATE_LOGGING) {
    is_logging = false;
    current_state = STATE_STOPPED;
    server.send(200, "text/plain",
                "Logging stopped; you can now download the log file.");
  } else {
    server.send(409, "text/plain", "Not currently logging.");
  }
}

static void handle_done() {
  is_logging = false;
  current_state = STATE_DONE;
  server.send(200, "text/plain", "Marked as DONE; logging stays off.");
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
server.on("/stop", HTTP_GET, handle_stop_logging);
server.on("/start", HTTP_GET, handle_start_logging);
server.on("/done", HTTP_GET, handle_done);
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

bool webui_get_logging() {
  return is_logging;
}

void webui_set_state(LoggerState s) {
  current_state = s;
}

LoggerState webui_get_state() {
  return current_state;
}

