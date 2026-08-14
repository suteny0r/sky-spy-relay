/*
 * SKY-SPY-RELAY
 *
 * Second-board relay for the OUI-SPY Sky Spy drone detector.
 *
 * Reads the Sky Spy JSON detection stream from the expansion board's Grove
 * UART (GPIO44 RX / GPIO43 TX, 115200 8N1), connects to WiFi as a station,
 * and publishes each line to MQTT on the SKY-SPY-Aware topic scheme:
 *   skyspy/<topic>/raw         every non-blank serial line
 *   skyspy/<topic>/detections  detection JSON lines only
 *
 * Configuration is done ON DEVICE when the expansion board is present:
 * the OLED shows a setup wizard and the USER button drives it. The buzzer
 * beeps for feedback. Single-button controls: TAP advances/cycles, HOLD
 * selects/confirms. Menu:
 *   1 WIFI SETUP   shows QR codes: scan to join the AP, then open
 *                  192.168.4.1 and configure WiFi + MQTT in the browser
 *   2 MQTT SETUP   on-device broker host/port/user/pass/topic/TLS (fallback)
 *   3 SIM TEST     one-shot publish of captured detection data to MQTT
 *   4 SAVE & RUN   persist and reboot into relay mode
 *
 * The relay always serves an AP "sky-relay" / "skyspyrelay" with a captive
 * portal at 192.168.4.1 while the wizard is active, so a phone that joins the
 * AP is taken straight to the config page.
 *
 * In relay (run) mode a TAP advances the OLED status page and holding the
 * USER button for 2s re-enters the setup wizard. Without an expansion board
 * the web portal is used instead.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <DNSServer.h>
#include <time.h>
#include <esp_sntp.h>
#include <PubSubClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "dashboard.h"

#define FW_VERSION "1.0.1"

// ============================================================================
// Hardware pins
// ============================================================================
// Grove UART on the expansion board: D6 (GPIO43 TX) / D7 (GPIO44 RX).
// The Sky Spy board's relay UART TX (GPIO43) wires to our RX (GPIO44).
#define RELAY_UART_BAUD  115200
#define RELAY_RX_PIN     44      // D7  -- Sky Spy relay TX -> this RX
#define RELAY_TX_PIN     43      // D6  -- this TX -> Sky Spy RX (unused)
#define LED_PIN          21      // Onboard LED (inverted logic)
#define USER_BUTTON_PIN  2       // expansion board USER button (GPIO2) - only button used

// ============================================================================
// Config storage (NVS)
// ============================================================================
#define CFG_NS            "relaycfg"
#define CFG_SSID          "wifi_ssid"
#define CFG_PASS          "wifi_pass"
#define CFG_MQTT_HOST     "mqtt_host"
#define CFG_MQTT_PORT     "mqtt_port"
#define CFG_MQTT_USER     "mqtt_user"
#define CFG_MQTT_PASS     "mqtt_pass"
#define CFG_MQTT_TOPIC    "mqtt_topic"
#define CFG_MQTT_TLS      "mqtt_tls"
#define CFG_WIZARD        "wizard"      // set when we want the wizard on next boot
#define CFG_NET_CACHE     "net_cache"   // JSON array of recently seen SSIDs

// MQTT defaults. The gitignored mqtt_secrets.json is the source of truth:
// scripts/gen_secrets.py (extra_scripts in platformio.ini) generates
// src/secrets.h from it before every build. The fallbacks below are
// public-safe (no credentials) so the repo builds without secrets.
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef SECRET_MQTT_HOST
#define SECRET_MQTT_HOST "65604cba457d4f8992aefe5820219ae4.s1.eu.hivemq.cloud"
#define SECRET_MQTT_PORT 8883
#define SECRET_MQTT_TLS  1
#define SECRET_MQTT_USER ""
#define SECRET_MQTT_PASS ""
#define SECRET_MQTT_TOPIC "skyspy"
#endif

#define DEFAULT_MQTT_HOST SECRET_MQTT_HOST
#define DEFAULT_MQTT_PORT SECRET_MQTT_PORT
#define DEFAULT_MQTT_TLS  (SECRET_MQTT_TLS != 0)
#define DEFAULT_MQTT_USER SECRET_MQTT_USER
#define DEFAULT_MQTT_PASS SECRET_MQTT_PASS
#define DEFAULT_MQTT_TOPIC SECRET_MQTT_TOPIC

static String cfgSsid = "";
static String cfgPass = "";
static String cfgMqttHost = DEFAULT_MQTT_HOST;
static int    cfgMqttPort = DEFAULT_MQTT_PORT;
static String cfgMqttUser = DEFAULT_MQTT_USER;
static String cfgMqttPass = DEFAULT_MQTT_PASS;
static String cfgMqttTopic = DEFAULT_MQTT_TOPIC;
static bool   cfgMqttTls = DEFAULT_MQTT_TLS;

static void loadConfig() {
    Preferences p;
    p.begin(CFG_NS, true);
    cfgSsid = p.getString(CFG_SSID, "");
    cfgPass = p.getString(CFG_PASS, "");
    cfgMqttHost = p.getString(CFG_MQTT_HOST, DEFAULT_MQTT_HOST);
    cfgMqttPort = p.getInt(CFG_MQTT_PORT, DEFAULT_MQTT_PORT);
    cfgMqttUser = p.getString(CFG_MQTT_USER, DEFAULT_MQTT_USER);
    cfgMqttPass = p.getString(CFG_MQTT_PASS, DEFAULT_MQTT_PASS);
    cfgMqttTopic = p.getString(CFG_MQTT_TOPIC, DEFAULT_MQTT_TOPIC);
    cfgMqttTls = p.getBool(CFG_MQTT_TLS, DEFAULT_MQTT_TLS);
    p.end();
}

static void saveConfig(const String &ssid, const String &pass,
                       const String &host, int port,
                       const String &user, const String &pw,
                       const String &topic, bool tls) {
    Preferences p;
    p.begin(CFG_NS, false);
    p.putString(CFG_SSID, ssid);
    p.putString(CFG_PASS, pass);
    p.putString(CFG_MQTT_HOST, host);
    p.putInt(CFG_MQTT_PORT, port);
    p.putString(CFG_MQTT_USER, user);
    p.putString(CFG_MQTT_PASS, pw);
    p.putString(CFG_MQTT_TOPIC, topic);
    p.putBool(CFG_MQTT_TLS, tls);
    p.end();
}

static void setWizardFlag(bool on) {
    Preferences p;
    p.begin(CFG_NS, false);
    p.putBool(CFG_WIZARD, on);
    p.end();
}

static bool getWizardFlag() {
    Preferences p;
    p.begin(CFG_NS, true);
    bool v = p.getBool(CFG_WIZARD, false);
    p.end();
    return v;
}

static bool configValid() {
    return cfgSsid.length() > 0 && cfgMqttHost.length() > 0;
}

// ============================================================================
// Cached WiFi network list
// ============================================================================
// The relay scans for networks while the AP is up (STA+AP mode) and caches
// the SSIDs in NVS. The web portal offers them as a pick list so the phone
// does not have to type an SSID.
#define NET_CACHE_MAX 8
#define NET_SCAN_INTERVAL_MS 60000

static String netCache[NET_CACHE_MAX];
static int netCacheCount = 0;
static bool netScanRunning = false;
static unsigned long lastNetScanMs = 0;

static void netCacheSave() {
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < netCacheCount; i++) arr.add(netCache[i]);
    String out;
    serializeJson(doc, out);
    Preferences p;
    p.begin(CFG_NS, false);
    p.putString(CFG_NET_CACHE, out);
    p.end();
}

static void netCacheLoad() {
    Preferences p;
    p.begin(CFG_NS, true);
    String s = p.getString(CFG_NET_CACHE, "");
    p.end();
    netCacheCount = 0;
    if (s.length() == 0) return;
    JsonDocument doc;
    if (deserializeJson(doc, s)) return;
    JsonArray arr = doc.as<JsonArray>();
    for (JsonVariant v : arr) {
        const char *name = v;
        if (name && netCacheCount < NET_CACHE_MAX) netCache[netCacheCount++] = name;
    }
}

// Kick off / collect an async scan (self-gated to NET_SCAN_INTERVAL_MS).
// Call frequently from the wizard loop; no-ops when not needed.
static void netScanPoll() {
    if (!netScanRunning) {
        if (millis() - lastNetScanMs < NET_SCAN_INTERVAL_MS) return;
        lastNetScanMs = millis();
        netScanRunning = true;
        WiFi.scanNetworks(true);
    } else {
        int done = WiFi.scanComplete();
        if (done < 0) return;  // still scanning
        netScanRunning = false;
        if (done > NET_CACHE_MAX) done = NET_CACHE_MAX;
        netCacheCount = 0;
        for (int i = 0; i < done; i++) {
            String ssid = WiFi.SSID(i);
            if (ssid.length() == 0 || ssid.length() > 32) continue;
            bool dup = false;
            for (int j = 0; j < netCacheCount; j++) {
                if (netCache[j] == ssid) { dup = true; break; }
            }
            if (!dup) netCache[netCacheCount++] = ssid;
        }
        WiFi.scanDelete();
        if (netCacheCount > 0) netCacheSave();
        Serial.printf("[WIZ] Cached %d networks\r\n", netCacheCount);
    }
}

// ============================================================================
// Buzzer / speaker
// ============================================================================
static int buzzerPin = BUZZER_PIN_EXTERNAL;
static bool buzzerOn = true;

static void beep(int freq, int ms) {
    if (!buzzerOn) return;
    tone(buzzerPin, freq, ms);
}

static void beepClick()      { beep(800, 30); }
static void beepSelect()     { beep(1500, 90); }
static void beepChar()       { beep(2000, 40); }
static void beepSuccess()    { beep(1300, 120); delay(60); beep(1800, 160); }
static void beepFail()       { beep(300, 300); }

// ============================================================================
// WiFi / MQTT state
// ============================================================================
static WiFiClient   mqttWifiClient;
static WiFiClientSecure mqttSecureClient;
static PubSubClient *mqtt = nullptr;

static bool wifiConnected = false;
static unsigned long lastMqttAttempt = 0;
static unsigned long lastWifiAttempt = 0;

// Internet time sync state (NTP + IP-based timezone).
static bool timeSynced = false;      // clock has been configured for NTP
static bool timeLogged = false;      // one-shot heartbeat print of the clock
static bool netTimeOk = false;       // SNTP actually completed at least once
static bool rtcSeeded = false;       // boot clock came from the expansion RTC
static String timezoneName = "UTC";  // informational only; clock is always UTC

// Runtime counters for the OLED
static unsigned long detCount = 0;        // total detection lines received
static unsigned long lastDetMs = 0;       // millis() of last detection
static unsigned long lineCount = 0;       // total lines received
static String lastDetBrief = "";          // short summary of last detection

// ============================================================================
// UART line buffer
// ============================================================================
#define LINE_BUF_SIZE 512
static char lineBuf[LINE_BUF_SIZE];
static size_t lineLen = 0;

static bool looksLikeDetection(const char *line) {
    // SKY-SPY-Aware detects detections by the leading {"mac" marker.
    return strncmp(line, "{\"mac\"", 6) == 0;
}

// Add the relay's UTC receive timestamp to a forwarded JSON frame. Non-JSON
// lines (plain console text) are returned verbatim so they stay intact. The
// Sky Spy originator has no internet/time source of its own; the relay does
// (internet-synced, battery-backed RTC), so it is the time authority that
// stamps each detection for later logfile reconstruction.
static String relayStamp(JsonDocument &doc, bool parsed, const char *l) {
    if (!parsed) return String(l);
    time_t nowt = time(nullptr);
    if (nowt > 1600000000) {
        doc["ts"] = (unsigned long)nowt;
        struct tm utc;
        gmtime_r(&nowt, &utc);
        char buf[40];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                 utc.tm_hour, utc.tm_min, utc.tm_sec);
        doc["ts_str"] = buf;
    } else {
        doc["ts_ms"] = millis();  // clock not yet valid; uptime only
    }
    String out;
    serializeJson(doc, out);
    return out;
}

static void handleLine(const char *line) {
    // Work on a local copy so the trailing-newline trim cannot touch the
    // caller's buffer (sim lines are const, UART lines are mutable).
    char buf[LINE_BUF_SIZE];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *l = buf;
    size_t n = strlen(l);
    while (n > 0 && (l[n-1] == '\n' || l[n-1] == '\r')) l[--n] = '\0';
    if (n == 0) return;

    lineCount++;
    bool isDet = looksLikeDetection(l);

    // Parse the Sky Spy frame once. The relay is the time authority: it has an
    // internet-synced, battery-backed RTC while the originator does not, so we
    // stamp every forwarded detection here. This only adds a key, so subscribers
    // that read fields by name (sky-spy-aware-android) are unaffected.
    JsonDocument doc;
    bool parsed = (deserializeJson(doc, l) == DeserializationError::Ok);

    if (isDet) {
        detCount++;
        lastDetMs = millis();
        if (parsed) {
            const char *mac = doc["mac"] | "";
            int rssi = doc["rssi"] | 0;
            char brief[64];
            snprintf(brief, sizeof(brief), "MAC %s RSSI %d", mac, rssi);
            lastDetBrief = brief;
        }
    }

    if (mqtt && mqtt->connected()) {
        String payload = relayStamp(doc, parsed, l);
        String rawTopic = cfgMqttTopic + "/raw";
        String detTopic = cfgMqttTopic + "/detections";
        mqtt->publish(rawTopic.c_str(), payload.c_str());
        if (isDet) {
            mqtt->publish(detTopic.c_str(), payload.c_str());
            Serial.printf("[RELAY] sent detection %s\r\n", payload.c_str());
        } else {
            Serial.printf("[RELAY] sent line %s\r\n", payload.c_str());
        }
    }
}

static void pollRelayUart() {
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n' || lineLen >= LINE_BUF_SIZE - 1) {
            lineBuf[lineLen] = '\0';
            handleLine(lineBuf);
            lineLen = 0;
        } else if (c != '\r') {
            lineBuf[lineLen++] = c;
        }
    }
}

// ============================================================================
// MQTT
// ============================================================================
// Publish a one-shot status message on every MQTT (re)connect so consumers
// know the relay is live, who it is, and when it came online. The timestamp
// falls back to uptime if the NTP sync has not landed yet.
static void publishOnlineAnnounce() {
    String statusTopic = cfgMqttTopic + "/status";
    String announce;
    JsonDocument doc;
    doc["event"] = "publishing";
    doc["device"] = "sky-spy-relay";
    doc["mac"] = WiFi.macAddress();
    doc["ip"] = WiFi.localIP().toString();
    doc["fw"] = FW_VERSION;
    time_t nowt = time(nullptr);
    if (timeSynced && nowt > 100000) {
        doc["ts"] = (unsigned long)nowt;
        struct tm utc;
        gmtime_r(&nowt, &utc);
        char buf[40];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                 utc.tm_hour, utc.tm_min, utc.tm_sec);
        doc["ts_str"] = buf;
    } else {
        doc["ts_ms"] = millis();  // clock not synced yet
    }
    serializeJson(doc, announce);
    bool ok = mqtt->publish(statusTopic.c_str(), announce.c_str());
    Serial.printf("[RELAY] online announce -> %s (%s)\r\n",
                  statusTopic.c_str(), ok ? "ok" : "failed");
}

static void mqttReconnect() {
    if (mqtt == nullptr) return;
    if (mqtt->connected()) return;
    if (millis() - lastMqttAttempt < 3000) return;
    lastMqttAttempt = millis();
    if (!wifiConnected) return;

    String clientId = String("sky-relay-") + String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFF), HEX);
    bool ok = false;
    if (cfgMqttUser.length() > 0) {
        ok = mqtt->connect(clientId.c_str(), cfgMqttUser.c_str(), cfgMqttPass.c_str());
    } else {
        ok = mqtt->connect(clientId.c_str());
    }
    if (ok) {
        Serial.println("[RELAY] MQTT connected");
        publishOnlineAnnounce();
    } else {
        Serial.printf("[RELAY] MQTT connect failed rc=%d\r\n", mqtt->state());
    }
}

static void setupMqtt() {
    if (cfgMqttTls) {
        mqttSecureClient.setInsecure();
        mqtt = new PubSubClient(mqttSecureClient);
    } else {
        mqtt = new PubSubClient(mqttWifiClient);
    }
    mqtt->setServer(cfgMqttHost.c_str(), cfgMqttPort);
    mqtt->setBufferSize(512);
    mqtt->setKeepAlive(30);
}

// ============================================================================
// OLED helpers
// ============================================================================
#define DASH_BASELINE 6
#define DASH_ROW_STEP 8

static void dashRow(uint8_t row) {
    dashboard_set_cursor(0, DASH_BASELINE + row * DASH_ROW_STEP);
}

static void dashBoxRow(uint8_t row, uint8_t col, uint8_t w) {
    dashboard_draw_box(col * 6, DASH_BASELINE + row * DASH_ROW_STEP - 6, w * 6, 7);
}

// ============================================================================
// Setup wizard state machine
// ============================================================================
enum WizState {
    WIZ_MENU = 0,
    WIZ_MQTT_HOST,
    WIZ_MQTT_PORT,
    WIZ_MQTT_USER,
    WIZ_MQTT_PASS,
    WIZ_MQTT_TOPIC,
    WIZ_MQTT_TLS,
    WIZ_PORTAL,
    WIZ_SIMULATE,
};

static WizState wizState = WIZ_MENU;
static uint8_t menuSel = 0;
static const char *MENU_ITEMS[] = {
    "WIFI SETUP", "MQTT SETUP", "SIM TEST", "SAVE & RUN"
};
#define MENU_COUNT 4

// Password / text editor
static String  editText;            // value being built
static int     charIdx = 0;         // current char in charset
static const char CHARSET[] =
    "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ _-.@:/#%+*=!?&";
static const int CHARSET_LEN = sizeof(CHARSET) - 1;  // index CHARSET_LEN == backspace
static bool editSecret = false;

// ============================================================================
// Button handling (single USER button, GPIO2)
// ============================================================================
// The USER button on the expansion board is the only input used:
//   short press (tap)   -> advance / next / cycle
//   long press (hold)   -> select / confirm / done / back
// BOOT (GPIO0) is intentionally not used - it is hard to reach on this board.
#define BTN_LONG_MS 1200    // hold duration that counts as a long press
#define BTN_DEBOUNCE_MS 30

// Returns:
//   0 = no event
//   1 = short press (tap, released before BTN_LONG_MS)
//   2 = long press (held >= BTN_LONG_MS, fires once while held)
static uint8_t pollUserButton() {
    static bool down = false;
    static unsigned long downAt = 0;
    static bool longDone = false;
    static bool shortQueued = false;

    bool raw = digitalRead(USER_BUTTON_PIN) == LOW;

    if (raw && !down) {
        down = true;
        downAt = millis();
        longDone = false;
    }

    // Long press fires on the rising edge of the hold duration.
    if (down && !longDone && (millis() - downAt) >= BTN_LONG_MS) {
        longDone = true;
        return 2;
    }

    // Short press fires on release (only if the press was not long).
    if (!raw && down) {
        down = false;
        if (!longDone && (millis() - downAt) >= BTN_DEBOUNCE_MS) {
            shortQueued = true;
        }
    }

    if (shortQueued) {
        shortQueued = false;
        return 1;
    }
    return 0;
}

// ============================================================================
// Wizard: char editor
// ============================================================================
// Renders the label, current text and a char strip. The strip cycles through
// the printable charset plus two special slots: "<" (backspace) and "OK"
// (done). Returns:
//   true  = finished (long press on the OK slot) - editText holds the value
//   false = still editing
static bool wizEditChar(String &target, const char *label, bool secret) {
    uint8_t btn = pollUserButton();

    if (btn == 1) {
        // short press: advance to the next slot
        charIdx = (charIdx + 1) % (CHARSET_LEN + 2);
        beepClick();
    } else if (btn == 2) {
        // long press: activate the highlighted slot
        if (charIdx == CHARSET_LEN) {
            // backspace
            if (target.length() > 0) target.remove(target.length() - 1);
            beepClick();
        } else if (charIdx == CHARSET_LEN + 1) {
            // done
            beepSelect();
            return true;
        } else {
            target += CHARSET[charIdx];
            beepChar();
        }
    }

    dashboard_clear();
    dashRow(0);
    dashboard_printf("%s", label);
    dashRow(1);
    String shown = secret ? String(target.length(), '*') : target;
    if (shown.length() > 21) shown = shown.substring(shown.length() - 21);
    dashboard_printf("%s", shown.c_str());
    dashRow(2);
    dashboard_printf("LEN %d", target.length());

    // char strip: 17 slots centered on charIdx
    int half = 8;
    int center = charIdx;
    int start = center - half;
    for (int i = 0; i <= half * 2; i++) {
        int idx = start + i;
        if (idx < 0 || idx > CHARSET_LEN + 1) continue;
        int x = i * 6 + 2;
        if (idx == charIdx) {
            dashboard_draw_box(x - 1, DASH_BASELINE + 4 * DASH_ROW_STEP - 6, 6, 7);
            dashboard_set_draw_color(0);
        }
        dashboard_set_cursor(x, DASH_BASELINE + 4 * DASH_ROW_STEP);
        if (idx == CHARSET_LEN) {
            dashboard_printf("<");
        } else if (idx == CHARSET_LEN + 1) {
            dashboard_printf("OK");
        } else {
            dashboard_printf("%c", CHARSET[idx]);
        }
        if (idx == charIdx) dashboard_set_draw_color(1);
    }
    dashRow(6);
    dashboard_printf("TAP NEXT  HOLD SELECT");
    dashRow(7);
    dashboard_printf("< = DEL   OK = DONE");
    dashboard_flush();
    return false;
}

// ============================================================================
// Wizard render functions
// ============================================================================
static void wizRenderMenu() {
    dashboard_clear();
    dashRow(0);
    dashboard_printf("SKY-SPY RELAY SETUP");
    dashboard_draw_hline(0, 7, 128);
    for (int i = 0; i < MENU_COUNT; i++) {
        int row = 2 + i;
        if (i == menuSel) {
            dashboard_draw_box(0, DASH_BASELINE + row * DASH_ROW_STEP - 6, 128, 7);
            dashboard_set_draw_color(0);
            dashRow(row);
            dashboard_printf("%s", MENU_ITEMS[i]);
            dashboard_set_draw_color(1);
        } else {
            dashRow(row);
            dashboard_printf("  %s", MENU_ITEMS[i]);
        }
    }
    dashRow(7);
    dashboard_printf("TAP NEXT  HOLD OK");
    dashboard_flush();
}

// ============================================================================
// Wizard: MQTT editors
// ============================================================================
static const char *MQTT_FIELD_LABEL(int f) {
    switch (f) {
        case WIZ_MQTT_HOST:  return "MQTT HOST";
        case WIZ_MQTT_PORT:  return "MQTT PORT";
        case WIZ_MQTT_USER:  return "MQTT USER";
        case WIZ_MQTT_PASS:  return "MQTT PASS";
        case WIZ_MQTT_TOPIC: return "MQTT TOPIC";
        case WIZ_MQTT_TLS:   return "MQTT TLS";
        default: return "";
    }
}

// Renders/edits the given MQTT field. Returns true when the field is done.
static bool wizEditMqttField(WizState field) {
    // TLS is a yes/no toggle
    if (field == WIZ_MQTT_TLS) {
        uint8_t btn = pollUserButton();
        if (btn == 1) {
            cfgMqttTls = !cfgMqttTls;
            beepChar();
        } else if (btn == 2) {
            beepSelect();
            return true;
        }
        dashboard_clear();
        dashRow(0);
        dashboard_printf("MQTT TLS");
        dashRow(2);
        dashboard_printf(cfgMqttTls ? "YES (8883)" : "NO (1883)");
        dashRow(4);
        dashboard_printf("TAP = TOGGLE");
        dashRow(6);
        dashboard_printf("HOLD = DONE");
        dashboard_flush();
        return false;
    }

    String target;
    switch (field) {
        case WIZ_MQTT_HOST:  target = cfgMqttHost;  editSecret = false; break;
        case WIZ_MQTT_PORT:  target = String(cfgMqttPort); editSecret = false; break;
        case WIZ_MQTT_USER:  target = cfgMqttUser;  editSecret = false; break;
        case WIZ_MQTT_PASS:  target = cfgMqttPass;  editSecret = true;  break;
        case WIZ_MQTT_TOPIC: target = cfgMqttTopic; editSecret = false; break;
        default: return true;
    }
    editText = target;

    if (wizEditChar(editText, MQTT_FIELD_LABEL(field), editSecret)) {
        switch (field) {
            case WIZ_MQTT_HOST:  cfgMqttHost = editText; break;
            case WIZ_MQTT_PORT:  cfgMqttPort = editText.toInt(); if (cfgMqttPort <= 0) cfgMqttPort = 1883; break;
            case WIZ_MQTT_USER:  cfgMqttUser = editText; break;
            case WIZ_MQTT_PASS:  cfgMqttPass = editText; break;
            case WIZ_MQTT_TOPIC: cfgMqttTopic = editText; break;
            default: break;
        }
        return true;
    }
    return false;
}

// ============================================================================
// Simulation data (SIM TEST menu item)
// ============================================================================
// A captured Sky Spy session (two drones) replayed as one-shot MQTT publish
// test traffic. Console messages (heartbeats etc.) are not feed lines and are
// omitted - these are the raw detection lines the Grove UART would carry.
static const char *const SIM_LINES[] = {
    "{\"mac\":\"60:60:1f:13:49:66\",\"rssi\":-78,\"drone_lat\":25.782541,\"drone_long\":-80.155800,\"drone_altitude\":12,\"pilot_lat\":25.782921,\"pilot_long\":-80.155921,\"basic_id\":\"1581F6Z932463003M0UG\"}",
    "{\"mac\":\"60:60:1f:13:49:66\",\"rssi\":-80,\"drone_lat\":25.782516,\"drone_long\":-80.155800,\"drone_altitude\":12,\"pilot_lat\":25.782922,\"pilot_long\":-80.155922,\"basic_id\":\"1581F6Z932463003M0UG\"}",
    "{\"mac\":\"60:60:1f:13:49:66\",\"rssi\":-79,\"drone_lat\":25.782469,\"drone_long\":-80.155785,\"drone_altitude\":12,\"pilot_lat\":25.782907,\"pilot_long\":-80.155906,\"basic_id\":\"1581F6Z932463003M0UG\"}",
    "{\"mac\":\"60:60:1f:13:49:66\",\"rssi\":-81,\"drone_lat\":25.782440,\"drone_long\":-80.155777,\"drone_altitude\":12,\"pilot_lat\":25.782908,\"pilot_long\":-80.155905,\"basic_id\":\"1581F6Z932463003M0UG\"}",
    "{\"mac\":\"60:60:1f:13:49:66\",\"rssi\":-76,\"drone_lat\":25.782402,\"drone_long\":-80.155746,\"drone_altitude\":12,\"pilot_lat\":25.782892,\"pilot_long\":-80.155882,\"basic_id\":\"1581F6Z932463003M0UG\"}",
    "{\"mac\":\"60:60:1f:13:49:66\",\"rssi\":-76,\"drone_lat\":25.782375,\"drone_long\":-80.155723,\"drone_altitude\":13,\"pilot_lat\":25.782891,\"pilot_long\":-80.155883,\"basic_id\":\"1581F6Z932463003M0UG\"}",
    "{\"mac\":\"60:60:1f:13:49:66\",\"rssi\":-76,\"drone_lat\":25.782333,\"drone_long\":-80.155693,\"drone_altitude\":14,\"pilot_lat\":25.782891,\"pilot_long\":-80.155884,\"basic_id\":\"1581F6Z932463003M0UG\"}",
    "{\"mac\":\"60:60:1f:13:49:66\",\"rssi\":-80,\"drone_lat\":25.782293,\"drone_long\":-80.155655,\"drone_altitude\":14,\"pilot_lat\":25.782860,\"pilot_long\":-80.155845,\"basic_id\":\"1581F6Z932463003M0UG\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-82,\"drone_lat\":25.781733,\"drone_long\":-80.143249,\"drone_altitude\":10,\"pilot_lat\":25.781881,\"pilot_long\":-80.143028,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-79,\"drone_lat\":25.781734,\"drone_long\":-80.143242,\"drone_altitude\":9,\"pilot_lat\":25.781880,\"pilot_long\":-80.143027,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-81,\"drone_lat\":25.781734,\"drone_long\":-80.143242,\"drone_altitude\":9,\"pilot_lat\":25.781881,\"pilot_long\":-80.143027,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-83,\"drone_lat\":25.781736,\"drone_long\":-80.143234,\"drone_altitude\":8,\"pilot_lat\":25.781881,\"pilot_long\":-80.143027,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-78,\"drone_lat\":25.781738,\"drone_long\":-80.143227,\"drone_altitude\":8,\"pilot_lat\":25.781880,\"pilot_long\":-80.143028,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-78,\"drone_lat\":25.781742,\"drone_long\":-80.143211,\"drone_altitude\":7,\"pilot_lat\":25.781883,\"pilot_long\":-80.143027,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-79,\"drone_lat\":25.781742,\"drone_long\":-80.143211,\"drone_altitude\":7,\"pilot_lat\":25.781883,\"pilot_long\":-80.143028,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-80,\"drone_lat\":25.781755,\"drone_long\":-80.143188,\"drone_altitude\":7,\"pilot_lat\":25.781914,\"pilot_long\":-80.143027,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-79,\"drone_lat\":25.781755,\"drone_long\":-80.143188,\"drone_altitude\":7,\"pilot_lat\":25.781915,\"pilot_long\":-80.143027,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-77,\"drone_lat\":25.781757,\"drone_long\":-80.143188,\"drone_altitude\":7,\"pilot_lat\":25.781913,\"pilot_long\":-80.143035,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-80,\"drone_lat\":25.781765,\"drone_long\":-80.143165,\"drone_altitude\":7,\"pilot_lat\":25.781882,\"pilot_long\":-80.143026,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-80,\"drone_lat\":25.781769,\"drone_long\":-80.143158,\"drone_altitude\":7,\"pilot_lat\":25.781880,\"pilot_long\":-80.143027,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-78,\"drone_lat\":25.781771,\"drone_long\":-80.143150,\"drone_altitude\":7,\"pilot_lat\":25.781880,\"pilot_long\":-80.143027,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-81,\"drone_lat\":25.781771,\"drone_long\":-80.143143,\"drone_altitude\":7,\"pilot_lat\":25.781881,\"pilot_long\":-80.143028,\"basic_id\":\"1581F895C25AA007JUZR\"}",
    "{\"mac\":\"8c:1e:d9:31:90:b3\",\"rssi\":-78,\"drone_lat\":25.781773,\"drone_long\":-80.143135,\"drone_altitude\":1,\"pilot_lat\":25.781859,\"pilot_long\":-80.143074,\"basic_id\":\"1581F895C25AA007JUZR\"}",
};
#define SIM_LINE_COUNT (sizeof(SIM_LINES) / sizeof(SIM_LINES[0]))

enum SimStage {
    SIM_WIFI = 0,
    SIM_MQTT,
    SIM_SEND,
    SIM_DONE,
    SIM_FAIL,
};

// One-shot publish of the captured detection list, reachable from both the
// wizard menu (WIZ_SIMULATE) and the run dashboard (DASH_PAGE_SIM). Stages:
// connect WiFi (STA; the AP keeps serving in wizard mode), connect MQTT,
// send all lines through the normal handleLine() path, show the result.
// HOLD exits back to where the sim was started.
static SimStage simStage = SIM_WIFI;
static unsigned long simStageStart = 0;
static uint16_t simSent = 0;
static bool simActive = false;
static bool simFromWizard = false;

static void simLoop() {
    if (pollUserButton() == 2) {
        simStage = SIM_WIFI;
        simStageStart = 0;
        simActive = false;
        if (simFromWizard) {
            simFromWizard = false;
            wizState = WIZ_MENU;
        }
        beepSelect();
        return;
    }

    switch (simStage) {
        case SIM_WIFI: {
            if (!configValid()) {
                Serial.println("[SIM] No WiFi config - aborting");
                simStage = SIM_FAIL;
                break;
            }
            if (WiFi.status() == WL_CONNECTED) {
                simStage = SIM_MQTT;
                simStageStart = 0;
                break;
            }
            if (simStageStart == 0) {
                simStageStart = millis();
                Serial.printf("[SIM] Connecting to '%s'...\r\n", cfgSsid.c_str());
                WiFi.mode(WIFI_AP_STA);
                WiFi.persistent(false);
                WiFi.setAutoReconnect(true);
                WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
            } else if (millis() - simStageStart > 15000) {
                Serial.println("[SIM] WiFi connect timeout");
                simStage = SIM_FAIL;
                break;
            }
            dashboard_clear();
            dashRow(0);
            dashboard_printf("SIM TEST");
            dashboard_draw_hline(0, 7, 128);
            dashRow(2);
            dashboard_printf("CONNECTING WIFI");
            dashRow(4);
            {
                String s = cfgSsid;
                if (s.length() > 21) s = s.substring(0, 21);
                dashboard_printf("%s", s.c_str());
            }
            dashboard_flush();
            break;
        }

        case SIM_MQTT: {
            if (mqtt == nullptr) setupMqtt();
            if (mqtt->connected()) {
                simStage = SIM_SEND;
                simSent = 0;
                break;
            }
            if (simStageStart == 0) {
                simStageStart = millis();
                Serial.printf("[SIM] Connecting MQTT %s:%d...\r\n",
                              cfgMqttHost.c_str(), cfgMqttPort);
            } else if (millis() - simStageStart > 20000) {
                Serial.printf("[SIM] MQTT connect timeout (rc=%d)\r\n",
                              mqtt->state());
                simStage = SIM_FAIL;
                break;
            }
            // The wizard loop never runs the run-mode WiFi tracker, so
            // mqttReconnect()'s wifiConnected gate stays false. We got here
            // with WiFi connected, so unlock it.
            wifiConnected = true;
            mqttReconnect();
            mqtt->loop();
            dashboard_clear();
            dashRow(0);
            dashboard_printf("SIM TEST");
            dashboard_draw_hline(0, 7, 128);
            dashRow(2);
            dashboard_printf("CONNECTING MQTT");
            dashRow(4);
            dashboard_printf("%s", cfgMqttHost.c_str());
            dashboard_flush();
            break;
        }

        case SIM_SEND: {
            Serial.printf("[SIM] Sending %u lines...\r\n", (unsigned)SIM_LINE_COUNT);
            while (simSent < SIM_LINE_COUNT) {
                handleLine(SIM_LINES[simSent]);
                simSent++;
                if (mqtt) mqtt->loop();
                delay(80);
            }
            Serial.println("[SIM] Done");
            beepSuccess();
            simStage = SIM_DONE;
            break;
        }

        case SIM_DONE:
            dashboard_clear();
            dashRow(0);
            dashboard_printf("SIM TEST");
            dashboard_draw_hline(0, 7, 128);
            dashRow(2);
            dashboard_printf("SENT %u LINES", (unsigned)SIM_LINE_COUNT);
            dashRow(4);
            dashboard_printf("MQTT OK");
            dashRow(6);
            dashboard_printf("HOLD = BACK");
            dashboard_flush();
            break;

        case SIM_FAIL:
            dashboard_clear();
            dashRow(0);
            dashboard_printf("SIM TEST");
            dashboard_draw_hline(0, 7, 128);
            dashRow(2);
            dashboard_printf("FAILED");
            if (!configValid()) {
                dashRow(3);
                dashboard_printf("SET UP WIFI FIRST");
            } else if (WiFi.status() != WL_CONNECTED) {
                dashRow(3);
                dashboard_printf("WIFI TIMEOUT");
            } else {
                dashRow(3);
                dashboard_printf("MQTT TIMEOUT");
            }
            dashRow(6);
            dashboard_printf("HOLD = BACK");
            dashboard_flush();
            break;
    }
}

// ============================================================================
// Wizard driver (call every loop while in wizard mode)
// ============================================================================
static bool wizMode = false;

static void wizLoop() {
    // Single-button handling: pollUserButton() returns 1 (tap) or 2 (hold).

    // Periodic heartbeat so the serial port shows wizard state even when idle.
    static unsigned long lastWizBeat = 0;
    if (millis() - lastWizBeat > 5000) {
        lastWizBeat = millis();
        Serial.printf("[WIZ] state=%d menuSel=%u\r\n", (int)wizState, menuSel);
    }

    switch (wizState) {
        case WIZ_MENU: {
            uint8_t btn = pollUserButton();
            if (btn == 1) {
                menuSel = (menuSel + 1) % MENU_COUNT;
                beepClick();
            } else if (btn == 2) {
                beepSelect();
                switch (menuSel) {
                    case 0: wizState = WIZ_PORTAL; break;   // AP + QR + web page
                    case 1: wizState = WIZ_MQTT_HOST; break;
                    case 2: wizState = WIZ_SIMULATE; break;
                    case 3:
                        if (configValid()) {
                            saveConfig(cfgSsid, cfgPass, cfgMqttHost, cfgMqttPort,
                                       cfgMqttUser, cfgMqttPass, cfgMqttTopic, cfgMqttTls);
                            beepSuccess();
                            setWizardFlag(false);
                            delay(500);
                            ESP.restart();
                        } else {
                            beepFail();
                            Serial.println("[WIZ] Need WiFi SSID and MQTT host before SAVE & RUN");
                        }
                        break;
                }
            }
            wizRenderMenu();
            break;
        }

        case WIZ_MQTT_HOST:
        case WIZ_MQTT_PORT:
        case WIZ_MQTT_USER:
        case WIZ_MQTT_PASS:
        case WIZ_MQTT_TOPIC:
        case WIZ_MQTT_TLS: {
            if (wizEditMqttField(wizState)) {
                int next = (int)wizState + 1;
                if (next > (int)WIZ_MQTT_TLS) {
                    wizState = WIZ_MENU;
                } else {
                    wizState = (WizState)next;
                }
            }
            break;
        }

        case WIZ_PORTAL: {
            static uint8_t portalPage = 0;
            uint8_t btn = pollUserButton();
            if (btn == 1) {
                portalPage = (portalPage + 1) % 3;
                beepClick();
            } else if (btn == 2) {
                beepSelect();
                wizState = WIZ_MENU;
                portalPage = 0;
                break;
            }
            dashboard_clear();
            if (portalPage == 0) {
                // Full-screen WiFi QR: scanning it joins the phone to the AP
                // automatically. No room for a title at 2 px/module (v3 QR),
                // and the side margins are the quiet zone.
                dashboard_draw_qrcode("WIFI:T:WPA;S:sky-relay;P:skyspyrelay;;");
            } else if (portalPage == 1) {
                // URL QR: opens the config page after joining the AP.
                dashRow(0);
                dashboard_printf("CONFIG URL");
                dashboard_draw_qrcode("http://192.168.4.1");
                dashRow(7);
                dashboard_printf("TAP=PAGE");
            } else {
                // Info page: AP credentials for phones that don't scan QRs.
                dashRow(0);
                dashboard_printf("WIFI SETUP");
                dashboard_draw_hline(0, 7, 128);
                dashRow(2);
                dashboard_printf("AP sky-relay");
                dashRow(3);
                dashboard_printf("PASS skyspyrelay");
                dashRow(4);
                dashboard_printf("URL 192.168.4.1");
                dashRow(6);
                dashboard_printf("SETUP VIA PHONE");
                dashRow(7);
                dashboard_printf("TAP=PAGE HOLD=BACK");
            }
            dashboard_flush();
            break;
        }

        case WIZ_SIMULATE:
            // Shared with the run dashboard (DASH_PAGE_SIM). simLoop() draws
            // its own screens and returns to the wizard menu on exit.
            simFromWizard = true;
            simLoop();
            break;
    }
}

// ============================================================================
// Web config portal (fallback when no expansion board)
// ============================================================================
static AsyncWebServer portalServer(80);

static const char PORTAL_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SKY SPY RELAY</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:monospace;background:#000;color:#0f0;min-height:100vh;display:flex;align-items:center;justify-content:center;padding:12px}
.t{border:2px solid #0f0;max-width:420px;width:100%;padding:14px}
h1{font-size:16px;letter-spacing:2px;text-align:center;border-bottom:1px solid #0f0;padding-bottom:8px;margin-bottom:12px}
label{display:block;font-size:10px;opacity:.7;margin-top:8px}
input{width:100%;padding:6px;background:#000;color:#0f0;border:1px solid #0f0;font-family:monospace;font-size:12px;margin-top:2px}
input:focus{outline:none;border-color:#fff;color:#fff}
.c{display:flex;align-items:center;gap:6px;margin-top:10px;font-size:11px}
button{width:100%;padding:10px;background:#0f0;color:#000;border:none;font-family:monospace;font-size:13px;font-weight:bold;cursor:pointer;margin-top:14px}
button:active{background:#fff}
#st{text-align:center;font-size:11px;margin-top:8px;min-height:14px}
</style></head><body>
<div class="t">
<h1>SKY-SPY RELAY SETUP</h1>
<form id="f">
<label>WiFi SSID (pick a scanned network or type)</label><input id="ssid" list="nets" maxlength="32"><datalist id="nets"></datalist>
<label>WiFi Password</label><input id="pass" type="password" maxlength="63">
<label>MQTT Broker Host</label><input id="host" maxlength="128">
<label>MQTT Port</label><input id="port" type="number" value="1883">
<label>MQTT Username</label><input id="user" maxlength="64">
<label>MQTT Password</label><input id="pw" type="password" maxlength="64">
<label>Topic Prefix</label><input id="topic" maxlength="64" value="skyspy">
<div class="c"><input id="tls" type="checkbox" style="width:auto"><span>TLS (port 8883)</span></div>
<button type="submit">SAVE AND REBOOT</button>
<div id="st"></div>
</form>
</div>
<script>
var vals=%JSON%;
document.getElementById('ssid').value=vals.ssid;
document.getElementById('pass').value=vals.pass;
document.getElementById('host').value=vals.host;
document.getElementById('port').value=vals.port;
document.getElementById('user').value=vals.user;
document.getElementById('pw').value=vals.pw;
document.getElementById('topic').value=vals.topic;
document.getElementById('tls').checked=vals.tls;
fetch('/networks').then(function(r){return r.json()}).then(function(arr){
var dl=document.getElementById('nets');
arr.forEach(function(n){var o=document.createElement('option');o.value=n;dl.appendChild(o);});
}).catch(function(){});
document.getElementById('f').addEventListener('submit',function(e){
e.preventDefault();
var b=document.getElementById('st');
var ssid=document.getElementById('ssid').value.trim();
var host=document.getElementById('host').value.trim();
if(!ssid||!host){b.textContent='SSID and broker host required';return;}
var q='ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(document.getElementById('pass').value)
+'&host='+encodeURIComponent(host)+'&port='+encodeURIComponent(document.getElementById('port').value)
+'&user='+encodeURIComponent(document.getElementById('user').value)+'&pw='+encodeURIComponent(document.getElementById('pw').value)
+'&topic='+encodeURIComponent(document.getElementById('topic').value.trim()||'skyspy')
+'&tls='+(document.getElementById('tls').checked?'1':'0');
fetch('/save?'+q).then(function(r){if(r.ok){b.textContent='SAVED - REBOOTING...'}else{b.textContent='ERROR'}}).catch(function(){b.textContent='ERROR'});
});
</script></body></html>
)rawliteral";

static String buildPortalJson() {
    JsonDocument doc;
    doc["ssid"] = cfgSsid;
    doc["pass"] = cfgPass;
    doc["host"] = cfgMqttHost;
    doc["port"] = cfgMqttPort;
    doc["user"] = cfgMqttUser;
    doc["pw"] = cfgMqttPass;
    doc["topic"] = cfgMqttTopic;
    doc["tls"] = cfgMqttTls;
    String out;
    serializeJson(doc, out);
    return out;
}

static void startPortal() {
    portalServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = FPSTR(PORTAL_HTML);
        html.replace("%JSON%", buildPortalJson());
        request->send(200, "text/html", html);
    });

    portalServer.on("/networks", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        for (int i = 0; i < netCacheCount; i++) arr.add(netCache[i]);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out);
    });

    // Any other path (phone captive-portal detection hits arbitrary hosts)
    // also gets the portal page, so the phone pops up "Sign in to network".
    portalServer.onNotFound([](AsyncWebServerRequest *request) {
        String html = FPSTR(PORTAL_HTML);
        html.replace("%JSON%", buildPortalJson());
        request->send(200, "text/html", html);
    });

    portalServer.on("/save", HTTP_GET, [](AsyncWebServerRequest *request) {
        String ssid = request->hasParam("ssid") ? request->getParam("ssid")->value() : "";
        String pass = request->hasParam("pass") ? request->getParam("pass")->value() : "";
        String host = request->hasParam("host") ? request->getParam("host")->value() : "";
        int port = request->hasParam("port") ? request->getParam("port")->value().toInt() : 1883;
        String user = request->hasParam("user") ? request->getParam("user")->value() : "";
        String pw = request->hasParam("pw") ? request->getParam("pw")->value() : "";
        String topic = request->hasParam("topic") ? request->getParam("topic")->value() : "skyspy";
        bool tls = request->hasParam("tls") && request->getParam("tls")->value() == "1";
        if (ssid.length() < 1 || host.length() < 1) {
            request->send(400, "text/plain", "SSID and broker host required");
            return;
        }
        if (tls && port != 8883) port = 8883;
        saveConfig(ssid, pass, host, port, user, pw, topic, tls);
        setWizardFlag(false);
        request->send(200, "text/plain", "OK");
        delay(300);
        ESP.restart();
    });

    portalServer.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        Preferences p;
        p.begin(CFG_NS, false);
        p.clear();
        p.end();
        request->send(200, "text/plain", "Config cleared");
        delay(300);
        ESP.restart();
    });

    portalServer.begin();
}

// ============================================================================
// WiFi
// ============================================================================
static DNSServer dnsServer;

// Start the config AP with a captive-portal DNS that answers every query with
// the portal IP, so a phone that joins the AP is taken to 192.168.4.1.
static void startApPortal() {
    WiFi.softAP("sky-relay", "skyspyrelay");
    dnsServer.start(53, "*", WiFi.softAPIP());
    startPortal();
    Serial.print("[RELAY] Portal AP IP ");
    Serial.println(WiFi.softAPIP());
}

static void startWifiStation() {
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
    Serial.printf("[RELAY] Connecting to '%s'...\r\n", cfgSsid.c_str());
}

// ============================================================================
// Internet time sync: NTP only, clock always kept in UTC. Drone location events
// are timestamped in UTC so consumers can translate to their own local time;
// we deliberately do NOT apply any timezone offset (no geo-IP lookup).
// ============================================================================

static void syncTimeFromInternet() {
    if (timeSynced) return;

    // UTC only: configTime with a 0 offset keeps the libc clock on UTC
    // (gmtime == localtime == UTC), and the RTC also stores UTC.
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    timeSynced = true;
    Serial.println("[TIME] NTP sync started (UTC)");
}

// ============================================================================
// Expansion board RTC (PCF8563 @ 0x51): the CR1220 coin cell keeps the clock
// running while the board is powered off. On boot we seed the libc clock from
// it if the time is still valid, so timestamps are correct before NTP lands;
// after each NTP sync we refresh the RTC so the battery holds the new time.
// ============================================================================
#define RTC_I2C_ADDR 0x51
static bool rtcPresent = false;

static uint8_t rtcBcd(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }

static bool rtcReadTime(struct tm *out) {
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(0x02);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)RTC_I2C_ADDR, (uint8_t)7) != 7) return false;
    uint8_t b[7];
    for (int i = 0; i < 7; i++) b[i] = Wire.read();

    if (b[0] & 0x80) return false;  // VL set: clock integrity not guaranteed

    // PCF8563 clones (e.g. BM8563) leave reserved high bits set in the
    // weekday/century_month registers; mask them so the BCD decode is valid.
    uint8_t sec = rtcBcd(b[0] & 0x7F);
    uint8_t min = rtcBcd(b[1] & 0x7F);
    uint8_t hour;
    if (b[2] & 0x80) {  // 12h mode
        uint8_t h = b[2] & 0x3F;
        hour = ((h >> 4) & 0x01) * 10 + (h & 0x0F);
        if (hour == 0) hour = 12;
        if ((h & 0x20) && hour < 12) hour += 12;  // PM
    } else {
        hour = rtcBcd(b[2] & 0x3F);
    }
    uint8_t day = rtcBcd(b[3] & 0x3F);
    uint8_t month = rtcBcd(b[5] & 0x1F);
    uint8_t year = rtcBcd(b[6]);

    if (sec > 59 || min > 59 || hour > 23 || day < 1 || day > 31 ||
        month < 1 || month > 12) return false;

    out->tm_sec = sec;
    out->tm_min = min;
    out->tm_hour = hour;
    out->tm_mday = day;
    out->tm_mon = month - 1;
    out->tm_year = 2000 + year - 1900;
    out->tm_wday = 0;
    out->tm_isdst = 0;
    return true;
}

// Write a UTC broken-down time, clearing the VL flag (24h mode). The RTC has
// no timezone concept, so we always store UTC and let the TZ env handle local.
static bool rtcWriteTime(const struct tm *t) {
    uint8_t b[7];
    b[0] = ((t->tm_sec / 10) << 4) | (t->tm_sec % 10);        // VL cleared
    b[1] = ((t->tm_min / 10) << 4) | (t->tm_min % 10);
    b[2] = ((t->tm_hour / 10) << 4) | (t->tm_hour % 10);      // 24h mode
    b[3] = ((t->tm_mday / 10) << 4) | (t->tm_mday % 10);
    b[4] = 0;
    int mm = t->tm_mon + 1;
    b[5] = ((mm / 10) << 4) | (mm % 10);
    int yy = (t->tm_year + 1900) % 100;
    b[6] = ((yy / 10) << 4) | (yy % 10);
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(0x02);
    Wire.write(b, 7);
    return Wire.endTransmission() == 0;
}

static void initRtcTime() {
    // Shares the OLED I2C bus; must run after dashboard_init() probes and
    // before Serial1.begin() so the pins are free on headless builds.
    Wire.begin(DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);
    Wire.beginTransmission(RTC_I2C_ADDR);
    rtcPresent = (Wire.endTransmission() == 0);
    if (!rtcPresent) {
        Serial.println("[RTC] PCF8563 not detected");
        if (!dashboard_present()) Wire.end();  // release pins when headless
        return;
    }
    Serial.println("[RTC] PCF8563 detected (CR1220 backed)");

    struct tm rtc;
    if (!rtcReadTime(&rtc)) {
        Serial.println("[RTC] time invalid (VL set or bad values) - waiting for NTP");
        if (!dashboard_present()) Wire.end();
        return;
    }
    int yr = rtc.tm_year + 1900;
    if (yr < 2024 || yr > 2036) {
        Serial.printf("[RTC] time implausible (year %d) - ignoring\r\n", yr);
        if (!dashboard_present()) Wire.end();
        return;
    }
    // TZ is still unset at this point, so mktime treats the broken-down
    // fields as UTC. configTime() later only sets the TZ env var and starts
    // SNTP; it does not reset the epoch, so this seed survives it.
    time_t epoch = mktime(&rtc);
    struct timeval tv = { epoch, 0 };
    settimeofday(&tv, nullptr);
    rtcSeeded = true;
    Serial.printf("[TIME] RTC seed %04d-%02d-%02d %02d:%02d:%02d UTC\r\n",
                  yr, rtc.tm_mon + 1, rtc.tm_mday,
                  rtc.tm_hour, rtc.tm_min, rtc.tm_sec);
    if (!dashboard_present()) Wire.end();
}

// ============================================================================
// Run mode OLED pages
// ============================================================================
#define DASH_REFRESH_MS 1000
static unsigned long lastDashRefresh = 0;

enum DashPage {
    DASH_PAGE_STATUS = 0,
    DASH_PAGE_LAST,
    DASH_PAGE_COUNTS,
    DASH_PAGE_HELP,
    DASH_PAGE_SIM,
    DASH_PAGE_COUNT
};
static uint8_t dashPage = DASH_PAGE_STATUS;

static const char *wifiStatusStr() {
    switch (WiFi.status()) {
        case WL_CONNECTED: return "CONNECTED";
        case WL_IDLE_STATUS: return "IDLE";
        case WL_NO_SSID_AVAIL: return "NO SSID";
        case WL_CONNECT_FAILED: return "FAILED";
        case WL_CONNECTION_LOST: return "LOST";
        case WL_DISCONNECTED: return "DISCONNECTED";
        default: return "UNKNOWN";
    }
}

static void getClockStrings(char *dateBuf, size_t dlen, char *timeBuf, size_t tlen) {
    time_t now = time(nullptr);
    if (now < 1600000000) {
        strncpy(dateBuf, "--/--/----", dlen - 1);
        dateBuf[dlen - 1] = '\0';
        strncpy(timeBuf, "--:--:--", tlen - 1);
        timeBuf[tlen - 1] = '\0';
        return;
    }
    struct tm t;
    gmtime_r(&now, &t);
    strftime(dateBuf, dlen, "%Y-%m-%d", &t);
    strftime(timeBuf, tlen, "%H:%M:%S", &t);
}

// Short status of the internet time sync for the dashboard (always UTC).
static const char *netTimeStatus() {
    if (netTimeOk) return "NTP OK";
    if (timeSynced) return "NTP SYNC";
    if (rtcSeeded) return "RTC ONLY";
    return "NTP OFF";
}

static void renderRunDashboard() {
    if (!dashboard_present()) return;
    unsigned long now = millis();
    if (now - lastDashRefresh < DASH_REFRESH_MS) return;
    lastDashRefresh = now;

    dashboard_clear();
    dashRow(0);
    dashboard_printf("SKY-SPY RELAY");
    dashboard_draw_hline(0, 7, 128);

    switch (dashPage) {
        case DASH_PAGE_STATUS:
            {
                char dbuf[16], tbuf[16];
                getClockStrings(dbuf, sizeof(dbuf), tbuf, sizeof(tbuf));
                dashRow(1);
                dashboard_printf("%s", dbuf);
                dashRow(2);
                dashboard_printf("%s %s", tbuf, timezoneName.c_str());
                dashRow(3);
                dashboard_printf("WIFI %s", wifiStatusStr());
                dashRow(4);
                if (WiFi.status() == WL_CONNECTED) {
                    dashboard_printf("IP %s", WiFi.localIP().toString().c_str());
                } else {
                    dashboard_printf("RSSI -");
                }
                dashRow(5);
                dashboard_printf("MQTT %s", (mqtt && mqtt->connected()) ? "CONNECTED" : "DOWN");
                dashRow(6);
                dashboard_printf("TIME %s", netTimeStatus());
                dashRow(7);
                {
                    String host = cfgMqttHost;
                    if (host.length() > 18) host = host.substring(0, 18);
                    dashboard_printf("%s:%d", host.c_str(), cfgMqttPort);
                }
            }
            break;

        case DASH_PAGE_LAST:
            dashRow(1);
            dashboard_printf("LAST DETECTION");
            if (lastDetBrief.length() > 0) {
                dashRow(3);
                dashboard_printf("%s", lastDetBrief.c_str());
                dashRow(4);
                unsigned long age = (millis() >= lastDetMs) ? (millis() - lastDetMs) / 1000UL : 0;
                dashboard_printf("AGE %lus", age);
            } else {
                dashRow(3);
                dashboard_printf("NO DETECTIONS");
                dashRow(4);
                dashboard_printf("YET");
            }
            dashRow(6);
            dashboard_printf("UART %d", RELAY_UART_BAUD);
            break;

        case DASH_PAGE_COUNTS:
            dashRow(1);
            dashboard_printf("COUNTERS");
            dashRow(3);
            dashboard_printf("DETECTIONS %lu", detCount);
            dashRow(4);
            dashboard_printf("LINES %lu", lineCount);
            dashRow(6);
            dashboard_printf("TOPIC %s", cfgMqttTopic.c_str());
            break;

        case DASH_PAGE_HELP:
            dashRow(1);
            dashboard_printf("HELP");
            dashRow(3);
            dashboard_printf("TAP = NEXT PAGE");
            dashRow(4);
            dashboard_printf("HOLD 2s = SETUP");
            dashRow(6);
            dashboard_printf("P%u/%u", (unsigned)(dashPage + 1), (unsigned)DASH_PAGE_COUNT);
            break;

        case DASH_PAGE_SIM:
            dashRow(1);
            dashboard_printf("SIM TEST");
            dashRow(3);
            dashboard_printf("HOLD 1s = PUBLISH");
            dashRow(4);
            dashboard_printf("%u DETECTION LINES", (unsigned)SIM_LINE_COUNT);
            dashRow(6);
            dashboard_printf("ONCE TO MQTT");
            break;
    }

    dashboard_flush();
}

// ============================================================================
// Run mode button: TAP advances the page, HOLD 2s enters the setup wizard.
// Uses its own hold timing (2s) so page taps stay responsive.
// ============================================================================
static bool runBtnDown = false;
static unsigned long runBtnDownAt = 0;
static bool runBtnLongDone = false;

static void checkRunButton() {
    bool raw = digitalRead(USER_BUTTON_PIN) == LOW;
    if (raw && !runBtnDown) {
        runBtnDown = true;
        runBtnDownAt = millis();
        runBtnLongDone = false;
    } else if (raw && runBtnDown && !runBtnLongDone &&
               dashPage == DASH_PAGE_SIM &&
               (millis() - runBtnDownAt) >= 1000) {
        // HOLD 1s on the SIM TEST page starts the one-shot publish while
        // the relay keeps running. runBtnLongDone blocks the 2s wizard hold.
        runBtnLongDone = true;
        Serial.println("[RELAY] Starting SIM TEST from run dashboard");
        beepSelect();
        simStage = SIM_WIFI;
        simStageStart = 0;
        simSent = 0;
        simActive = true;
        simFromWizard = false;
    } else if (raw && runBtnDown && !runBtnLongDone &&
               (millis() - runBtnDownAt) >= 2000) {
        runBtnLongDone = true;
        Serial.println("[RELAY] USER held 2s - entering setup wizard");
        setWizardFlag(true);
        delay(200);
        ESP.restart();
    } else if (!raw && runBtnDown) {
        runBtnDown = false;
        if (!runBtnLongDone && (millis() - runBtnDownAt) >= 30) {
            // tap: advance page
            dashPage++;
            if (dashPage >= DASH_PAGE_COUNT) dashPage = 0;
            lastDashRefresh = 0;
        }
    }
}

// ============================================================================
// Arduino entry points
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println("\n========================================");
    Serial.printf("SKY-SPY-RELAY v%s\r\n", FW_VERSION);
    Serial.println("========================================");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // LED off (inverted logic)

    dashboard_init();
    initRtcTime();
    buzzerPin = dashboard_buzzer_pin();
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);

    // Relay UART: read the Sky Spy JSON stream from the Grove UART.
    Serial1.begin(RELAY_UART_BAUD, SERIAL_8N1, RELAY_RX_PIN, RELAY_TX_PIN);
    Serial.printf("[RELAY] Listening on Grove UART RX GPIO%d @ %d baud\r\n",
                  RELAY_RX_PIN, RELAY_UART_BAUD);

    loadConfig();
    netCacheLoad();

    bool wantWizard = getWizardFlag() || !configValid();
    setWizardFlag(false);

    if (dashboard_present() && wantWizard) {
        Serial.println("[RELAY] Starting on-device setup wizard");
        wizMode = true;
        wizState = WIZ_MENU;
        wizRenderMenu();
        // The AP + captive portal is the provisioning path now: a phone that
        // scans the OLED QR joins "sky-relay" and lands on 192.168.4.1.
        WiFi.mode(WIFI_AP_STA);
        WiFi.persistent(false);
        startApPortal();
        beepSelect();
    } else if (configValid()) {
        Serial.println("[RELAY] Config found - starting relay mode");
        setupMqtt();
        startWifiStation();
        lastWifiAttempt = millis();
    } else {
        // Headless, no config: web portal only. AP_STA so the STA interface
        // can scan for the cached network list.
        Serial.println("[RELAY] No expansion board/config - starting web portal");
        WiFi.mode(WIFI_AP_STA);
        WiFi.persistent(false);
        startApPortal();
    }
}

void loop() {
    dnsServer.processNextRequest();

    if (wizMode) {
        netScanPoll();
        wizLoop();
        delay(10);
        return;
    }

    // Headless portal mode: keep the cached network list fresh too.
    if (!configValid()) netScanPoll();

    // Periodic heartbeat in relay mode.
    static unsigned long lastRunBeat = 0;
    if (millis() - lastRunBeat > 5000) {
        lastRunBeat = millis();
        Serial.printf("[RELAY] wifi=%d mqtt=%d lines=%lu det=%lu\r\n",
                      (int)WiFi.status(), (mqtt && mqtt->connected()) ? 1 : 0,
                      lineCount, detCount);
    }

    // Log the clock once the NTP sync actually lands (SNTP is async), then
    // refresh the RTC so the coin cell holds the corrected time. If the RTC
    // seeded the clock at boot, time() is already valid; only SNTP_COMPLETED
    // means we truly synchronized with an internet source.
    if (timeSynced && !timeLogged) {
        if (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
            time_t nowt = time(nullptr);
            if (nowt > 100000) {
                timeLogged = true;
                netTimeOk = true;
                struct tm utc;
                gmtime_r(&nowt, &utc);
                Serial.printf("[TIME] sync OK %04d-%02d-%02d %02d:%02d:%02d UTC\r\n",
                              utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                              utc.tm_hour, utc.tm_min, utc.tm_sec);
                if (rtcPresent) {
                    struct tm utc;
                    gmtime_r(&nowt, &utc);
                    if (rtcWriteTime(&utc)) {
                        Serial.println("[TIME] RTC updated (UTC)");
                    } else {
                        Serial.println("[TIME] RTC write failed");
                    }
                }
            }
        }
    }

    checkRunButton();
    // (page advance is handled inside checkRunButton's tap logic)

    if (simActive) {
        // One-shot SIM TEST started from the run dashboard. simLoop() owns
        // the display until it exits; keep the MQTT client serviced.
        simLoop();
        if (mqtt) mqtt->loop();
        delay(10);
        return;
    }

    pollRelayUart();

    if (configValid()) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!wifiConnected) {
                wifiConnected = true;
                Serial.printf("[RELAY] WiFi connected, IP %s\r\n",
                              WiFi.localIP().toString().c_str());
                syncTimeFromInternet();
            }
        } else {
            if (wifiConnected) {
                wifiConnected = false;
                timeSynced = false;
                timeLogged = false;
                netTimeOk = false;
                Serial.println("[RELAY] WiFi lost");
            }
            if (millis() - lastWifiAttempt > 10000) {
                lastWifiAttempt = millis();
                WiFi.reconnect();
            }
        }

        if (mqtt != nullptr) {
            mqttReconnect();
            mqtt->loop();
        }
    }

    renderRunDashboard();
    delay(5);
}
