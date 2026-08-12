# SKY-SPY-RELAY

Second-board relay for the **OUI-SPY** [Sky Spy](https://github.com/colonelpanichacks/oui-spy-unified-blue) drone detector (Mode 5).

The Sky Spy detector streams its full JSON detection output over the expansion board's **Grove UART**. This project runs on a second **Seeed Studio XIAO ESP32-S3** (also in an expansion board), reads that stream, connects to WiFi, and publishes each detection to an **MQTT broker** — no PC required.

It uses the same topic scheme as the [SKY-SPY-Aware](https://github.com/suteny0r/SKY-SPY-Aware) dashboard, so a SKY-SPY-Aware instance in subscribe mode can consume the feed, or any other MQTT tool can.

## Architecture

```
                Grove UART (cable)
Sky Spy (Mode 5)  ---------------->  sky-spy-relay (this project)
   GPIO43 TX  ---cable--->  GPIO44 RX   |  WiFi (STA)
                                        v
                                     MQTT broker
                              skyspy/<topic>/raw
                              skyspy/<topic>/detections
```

- **Sky Spy side (Mode 5, oui-spy-unified-blue):** when the expansion board OLED is detected, Serial1 moves to the Grove UART pins (GPIO43 TX / GPIO44 RX) and emits one full JSON line per detection. Headless builds keep the legacy Heltec LoRa/Meshtastic compact messages on GPIO5/6.
- **Relay side (this project):** `Serial1` on GPIO44 RX reads those lines. Each line is published to `skyspy/<topic>/raw`; lines that are detection JSON (start with `{"mac"`) are also published to `skyspy/<topic>/detections`.

## Wiring

Two expansion boards, one Grove UART port cabled to the other:

| Sky Spy expansion board | Relay expansion board |
|-------------------------|------------------------|
| Grove UART TX (GPIO43 / D6) | Grove UART RX (GPIO44 / D7) |
| Grove UART RX (GPIO44 / D7) | Grove UART TX (GPIO43 / D6) |
| GND | GND |

A standard 4-pin Grove cable can be used between the two Grove UART ports. Both boards need common ground.

## Requirements

- PlatformIO
- 2x Seeed Studio XIAO ESP32-S3 + 2x Seeed Studio Expansion Board Base for XIAO
- An MQTT broker (see [SKY-SPY-Aware](https://github.com/suteny0r/SKY-SPY-Aware) for broker recommendations: HiveMQ Cloud / EMQX on 8883 TLS, or self-hosted Mosquitto on 1883)

## Build & Flash

```bash
pio run                     # Build firmware
pio run -t upload           # Build and flash via USB
pio run -t clean            # Clean build artifacts
pio device monitor          # Serial monitor (115200 baud)
```

Build output: `.pio/build/seeed_xiao_esp32s3/firmware.bin`

## First Boot & Configuration

The relay boots into an on-device setup wizard whenever no config is stored (or when you hold the USER button 2s in relay mode). The expansion board OLED + USER button drive it, and the buzzer gives audio feedback.

**Single-button controls (USER button only):**
- **TAP** (short press) = advance / next / cycle
- **HOLD** (long press) = select / confirm / done / back

The BOOT button (GPIO0) is intentionally unused - it is hard to reach on the expansion board.

The wizard offers:

| Menu | What it does |
|------|--------------|
| **WIFI SETUP** | Starts the AP `sky-relay` / `skyspyrelay` and shows QR codes on the OLED: scan the first to join the AP, then the phone lands on **http://192.168.4.1** (captive portal). Page 2 has a URL QR, page 3 the plain-text credentials |
| **MQTT SETUP** | On-device fallback: broker host, port (1883 plain / 8883 TLS), username, password, topic prefix via the char editor |
| **SIM TEST** | One-shot: connects to WiFi + MQTT (using the saved config) and publishes a captured two-drone detection session (23 lines) to `<topic>/raw` and `<topic>/detections` to verify relay publishing |
| **SAVE & RUN** | Persists everything to NVS and reboots into relay mode |

**Char editor:** TAP cycles through characters (plus `<` delete and `OK` done), HOLD activates the highlighted slot.

### Setup with a phone (WIFI SETUP)

1. Select **WIFI SETUP** in the wizard. The relay is already serving its access point:
   - SSID: `sky-relay`
   - Password: `skyspyrelay`
2. Scan the **WiFi QR** on the OLED (page 1) with the phone camera - the phone joins the AP automatically and pops up the config page at **http://192.168.4.1** (captive portal). Phones that don't auto-popup get the URL from the page-2 QR or page-3 text.
3. The form is prefilled with sensible defaults:
   - WiFi SSID: pick from the **cached network list** (the relay scans for
     networks while the AP is up and offers them as a dropdown) or type a new
     one; enter the password.
   - MQTT: default broker `65604cba457d4f8992aefe5820219ae4.s1.eu.hivemq.cloud`,
     port `8883` with TLS, user/pass/topic matching the SKY-SPY-Aware server
     (`mqtt_secrets.json`). Override only if needed.
4. Click **SAVE AND REBOOT**. The relay reboots into station mode, connects to WiFi, and starts publishing detections.

The relay must be on the same WiFi network as the MQTT broker (or the broker must be reachable from it).

### Web portal (no expansion board / headless)

With no saved config the relay boots its AP and serves the same config page at **http://192.168.4.1** - connect to `sky-relay` / `skyspyrelay` and follow steps 3-4 above.

### Returning to the config portal

- **Hold the USER button (GPIO2) for 2 seconds** at any time. This clears the saved WiFi credentials and reboots into the wizard.
- The portal also exposes `/reset` to wipe all saved config.

## MQTT Topics

With topic prefix `skyspy` (the default):

| Topic | Payload |
|-------|---------|
| `skyspy/<prefix>/raw` | every non-blank line received on the Grove UART |
| `skyspy/<prefix>/detections` | detection JSON lines only |

Detection JSON line (identical to the Sky Spy USB serial output):

```json
{
  "mac": "8c:1e:d9:c8:c9:f7",
  "rssi": -81,
  "drone_lat": 25.784279,
  "drone_long": -80.149010,
  "drone_altitude": 118,
  "pilot_lat": 25.767196,
  "pilot_long": -80.137115,
  "basic_id": "1581F8LQC255L00227P5"
}
```

### Consuming with SKY-SPY-Aware

Run SKY-SPY-Aware in subscribe mode against the same broker:

```bash
python server.py --mqtt-subscribe
```

## Expansion Board OLED + Button

- **USER button** is the only input: TAP advances the OLED status pages (Status, Last Detection, Counters, Help, SIM Test), HOLD 2s re-enters the setup wizard. On the SIM Test page, HOLD 1s publishes the 23 captured detection lines to MQTT once, while the relay keeps running.
- All `dashboard_*()` calls are safe no-ops when no expansion board is attached, so the relay also works bare.

## Pins

| GPIO | Function |
|------|----------|
| 44 / D7 | Grove UART RX — Sky Spy relay TX in |
| 43 / D6 | Grove UART TX — out (currently unused) |
| 21 | Onboard LED (inverted logic) |
| 2 / D1 | Expansion board USER button (TAP = next, HOLD 2s = setup wizard) |
| 5 / D4 | OLED I2C SDA (expansion board) |
| 6 / D5 | OLED I2C SCL (expansion board) |
| 4 / D3 | Expansion board buzzer (unused) |
| 0 | BOOT button (unused — hard to reach on expansion board) |


