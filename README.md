# Mycila JSY App ⚡️📡

An Arduino / ESP32 web application for Shenzhen Jiansiyan (JSY) electricity meters (single-phase and three-phase).
Mycila JSY App reads JSY series meters (JSY1031, JSY-MK-163, JSY-MK-193, JSY-MK-194, JSY-MK-227, JSY-MK-229, JSY-MK-333 from [https://www.jsypowermeter.com](https://www.jsypowermeter.com)) and exposes a web dashboard, UDP broadcast stream, and a set of REST endpoints.
It can also emulate Shelly EM and Shelly 3EM REST APIs so home automation platforms that speak Shelly devices can integrate with JSY meters.

Mycila JSY App is built using the [Mycila JSY library](https://github.com/mathieucarbou/MycilaJSY) for JSY meter communication and the [Mycila ESPConnect library](https://github.com/mathieucarbou/MycilaESPConnect) for WiFi connectivity management.

[![](https://mathieu.carbou.me/MycilaJSYApp/screenshot.png)](https://mathieu.carbou.me/MycilaJSYApp/screenshot.png)

[![Latest Release](https://img.shields.io/github/release/mathieucarbou/MycilaJSYApp.svg)](https://GitHub.com/mathieucarbou/MycilaJSYApp/releases/)
[![GPLv3 license](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.txt)
[![Contributor Covenant](https://img.shields.io/badge/Contributor%20Covenant-2.1-4baaaa.svg)](code_of_conduct.md)
[![Build](https://github.com/mathieucarbou/MycilaJSYApp/actions/workflows/ci.yml/badge.svg)](https://github.com/mathieucarbou/MycilaJSYApp/actions/workflows/ci.yml)
[![GitHub latest commit](https://badgen.net/github/last-commit/mathieucarbou/MycilaJSYApp)](https://GitHub.com/mathieucarbou/MycilaJSYApp/commit/)

Table of contents

- 📥 [Downloads](#downloads)
- ⚙️ [Installation](#installation)
- 🔌 [Wiring](#wiring)
- 🧰 [Building](#how-to-build-your-own-version)
- 👀 [Usage](#usage)
- 📡 [REST API](#rest-api)
- 🪄 [Shelly EM & 3EM emulation](#shelly-em-and-3em-emulation)
- 🧩 [For developers](#for-developers)

## Features

- Reads JSY electricity meters over a serial interface (many JSY models supported).
- Web dashboard (via ESPDash) with real-time graphs and statistics.
- UDP broadcast of JSY data (binary MsgPack style payload) for listeners.
- Per-channel UDP publish switches (enable/disable publish per JSY channel or phase via the dashboard).
- OTA firmware update support (ElegantOTA).
- Emulates Shelly EM and Shelly 3EM REST APIs so 3rd-party integrations can treat the device as a Shelly energy meter.
- Shelly ID remapping for two-channel JSY models: map Shelly ids 0/1 to JSY Channel 1/2 from the dashboard.
- Configurable pins / serial port via build flags.

Important: this application can mimic Shelly EM and Shelly 3EM devices (Shelly Gen2 API) so it is easy to integrate into platforms expecting Shelly devices.

## Supported JSY models

- JSY-MK-163T
- JSY-MK-194T / JSY-MK-193
- JSY-MK-333
- JSY1031 and other similar single-/two-/three-phase JSY models

## Downloads

Releases (firmware binaries) are available in the GitHub releases page:

[https://github.com/mathieucarbou/MycilaJSYApp/releases](https://github.com/mathieucarbou/MycilaJSYApp/releases)

Naming convention:

- `MycilaJSYApp-BOARD.FACTORY.bin` — first-time / factory flash image
- `MycilaJSYApp-BOARD.OTA.bin` — OTA update image

FYI, Supported ESP32 boards with Ethernet support:

- [OLIMEX ESP32-PoE](https://docs.platformio.org/en/stable/boards/espressif32/esp32-poe.html)
- [OLIMEX ESP32-GATEWAY](https://docs.platformio.org/en/stable/boards/espressif32/esp32-gateway.html)
- [Wireless-Tag WT32-ETH01 Ethernet Module](https://docs.platformio.org/en/stable/boards/espressif32/wt32-eth01.html)
- [T-ETH-Lite ESP32 S3](https://github.com/Xinyuan-LilyGO/LilyGO-T-ETH-Series/)
- [Waveshare ESP32-S3 ETH Board](https://www.waveshare.com/wiki/ESP32-S3-ETH)

## Installation

First-time flash (factory image):

```bash
esptool.py erase_flash
esptool.py write_flash 0x0 MycilaJSYApp-BOARD.FACTORY.bin
```

OTA update (upload via web UI):

1. Download `MycilaJSYApp-BOARD.OTA.bin` from Releases
2. Open http://<device-ip>/update and upload the OTA file
3. Device reboots

## Wiring

After flashing the device the serial pins used for the JSY are printed to the Serial console during boot. The default board pin mappings are in the code and summarized below.

| Board                 | Serial2 RX (JSY TX) | Serial2 TX (JSY RX) | WiFi | Ethernet |
| :-------------------- | :-----------------: | :-----------------: | :--: | :------: |
| denky_d4              |         22          |         21          |  ✅  |    ❌    |
| esp32-c3-devkitc-02   |         18          |         19          |  ✅  |    ❌    |
| esp32-c6-devkitc-1    |          4          |          5          |  ✅  |    ❌    |
| esp32-gateway         |         16          |         17          |  ✅  |    ✅    |
| esp32-poe             |         35          |         33          |  ✅  |    ✅    |
| esp32-s2-saola-1      |         18          |         17          |  ✅  |    ❌    |
| esp32-s3-devkitc-1    |         16          |         17          |  ✅  |    ❌    |
| esp32dev              |         16          |         17          |  ✅  |    ❌    |
| esp32s3box            |         19          |         20          |  ✅  |    ❌    |
| lilygo_eth_lite_s3    |         17          |         18          |  ✅  |    ✅    |
| nodemcu-32s           |         16          |         17          |  ✅  |    ❌    |
| tinypico              |          4          |         25          |  ✅  |    ❌    |
| waveshare_esp32s3_eth |         19          |         20          |  ✅  |    ✅    |
| wipy3                 |          4          |         25          |  ✅  |    ❌    |
| wt32-eth01            |          5          |         17          |  ✅  |    ✅    |

Electric wiring note for 2-channel JSY boards:

- Channel 1 (CT1): any current conductor
- Channel 2 (CT2): grid should pass through CT2 (clamp) on many JSY boards

## How to build your own version

Using PlatformIO (recommended):

```bash
PIO_BOARD=esp32dev pio run -e ci
```

You can override the JSY serial mapping and UDP port using build flags, for example:

```bash
PIO_BOARD=esp32dev PLATFORMIO_BUILD_SRC_FLAGS="-DMYCILA_JSY_SERIAL=Serial2 -DMYCILA_JSY_RX=RX2 -DMYCILA_JSY_TX=TX2 -DMYCILA_UDP_PORT=53964" pio run -e ci
```

## Usage

- On first boot the device creates an access point to configure WiFi. Connect and select your network.
- Dashboard: http://<device-ip>/dashboard (root `/` is rewritten to `/dashboard` in normal mode)
- Console logs (web serial): http://<device-ip>/console
- OTA update page: http://<device-ip>/update

The device also broadcasts JSY data via UDP on port 53964 by default (configurable).

Tip: you can toggle UDP publishing per channel/phase directly from the dashboard (each channel/phase card includes an "UDP Publish" switch). These preferences are saved.

## REST API

The following HTTP endpoints are implemented in `src/main.cpp` (matching the Shelly Gen2 API for EM / 3EM where applicable). Use `http://<device-ip>` as base.

- `GET /api/jsy`

  - Returns the raw JSY data in JSON format. (Implemented via `jsy.toJson()`)

```json
{
  "enabled": true,
  "time": 2375713792,
  "speed": 9600,
  "address": 1,
  "model": 355,
  "model_name": "JSY-MK-163",
  "frequency": 50.01,
  "voltage": 236.01,
  "current": 6.16,
  "active_power": 1178,
  "reactive_power": 852.8571,
  "apparent_power": 1454.321,
  "power_factor": 0.81,
  "active_energy": 1433820,
  "active_energy_imported": 1101090,
  "active_energy_returned": 332730,
  "resistance": 31.04444,
  "dimmed_voltage": 191.2338,
  "nominal_power": 1794.225,
  "thdi_0": 72.39874
}
```

- `GET|POST /api/jsy/reset`

  - Resets the JSY energy counters. Returns 200 on success or 409 if the reset cannot be performed.

- `GET|POST /api/jsy/publish`

  - With no query parameter: returns the UDP data publishing state as JSON { "switch": "on"|"off" }.
  - With `?switch=on` or `?switch=off`: enables/disables UDP broadcasting for all channels/phases and persists preference.
  - Note: per-channel publish toggles are available from the dashboard UI and are persisted. The GET state reports "on" if any channel/phase is enabled.

- `GET|POST /api/restart`

  - Triggers a device restart. Returns 200 immediately.

- `GET|POST /api/reset`

  - Clears WiFi configuration (factory reset) and restarts the device. Returns 200.

- `GET /rpc`

  - Returns a JSON object listing the available RPC endpoints for the detected JSY model.

### Shelly EM and 3EM emulation

The project exposes Shelly-compatible endpoints (Shelly Gen2 API style) so third-party platforms can treat the device as a Shelly EM or Shelly 3EM. The emulation endpoints implemented in `src/main.cpp` are:

- `GET /rpc/Shelly.GetStatus`

  - Returns an aggregate Shelly EM-style status object containing `em1:0`, `em1:1`, `em1data:0`, and `em1data:1` sections.

- `GET /rpc/Shelly.GetDeviceInfo`

  - Returns JSON with device name, id, mac, firmware version (`MYCILA_JSY_VERSION`) and `app` set to `EM` or `3EM` depending on the detected JSY model.

```json
{
  "name": "Mycila JSY App",
  "id": "shellyproem50-40:22:D8:EA:C5:71",
  "mac": "40:22:D8:EA:C5:71",
  "ver": "15.3.8",
  "app": "EM"
}
```

- `GET /rpc/EM1.GetStatus?id=0|1|2`

  - Shelly EM / 3EM compatible status for a single channel/phase. Returns voltage, current, active/apparent power, power factor, frequency and calibration.
  - For single-phase meters (JSY-MK-163 / JSY1031) `id` must be 0.
  - For two-channel meters `id` must be 0 or 1; ids 0/1 are mapped to JSY channels according to the dashboard Shelly ID remapping controls.
  - For three-phase (JSY-MK-333) `id` can be 0, 1 or 2 (phase A/B/C).

```json
{
  "id": 0,
  "voltage": 240.2,
  "current": 6.473,
  "act_power": 1327.6,
  "aprt_power": 1557.6,
  "pf": 0.87,
  "freq": 50,
  "calibration": "factory"
}
```

- `GET /rpc/EM1Data.GetStatus?id=0|1|2`

  - Returns total active energy and returned energy for the requested channel/phase.
  - For two-channel meters, ids 0/1 follow the same dashboard-configured Shelly ID remapping.

```json
{ "id": 0, "total_act_energy": 2776175.11, "total_act_ret_energy": 571584.87 }
```

- `GET /rpc/EM.GetStatus?id=0|1|2`

  - Shelly 3EM full status (multi-phase) — implemented only when the detected model is a 3-phase meter (JSY-MK-333). Returns per-phase values plus totals.

```json
{
  "id": 0,
  "a_current": 4.029,
  "a_voltage": 236.1,
  "a_act_power": 951.2,
  "a_aprt_power": 951.9,
  "a_pf": 1,
  "a_freq": 50,
  "b_current": 4.027,
  "b_voltage": 236.201,
  "b_act_power": -951.1,
  "b_aprt_power": 951.8,
  "b_pf": 1,
  "b_freq": 50,
  "c_current": 3.03,
  "c_voltage": 236.402,
  "c_active_power": 715.4,
  "c_aprt_power": 716.2,
  "c_pf": 1,
  "c_freq": 50,
  "n_current": 11.029,
  "total_current": 11.083,
  "total_act_power": 2484.782,
  "total_aprt_power": 2486.7,
  "user_calibrated_phase": [],
  "errors": ["phase_sequence"]
}
```

- `GET /rpc/EMData.GetStatus?id=0|1|2`
  - Shelly 3EM energy totals per-phase and global totals — implemented only for JSY-MK-333.

```json
{
  "id": 0,
  "a_total_act_energy": 0,
  "a_total_act_ret_energy": 0,
  "b_total_act_energy": 0,
  "b_total_act_ret_energy": 0,
  "c_total_act_energy": 0,
  "c_total_act_ret_energy": 0,
  "total_act": 0,
  "total_act_ret": 0
}
```

- `GET /status` and `GET /emeter/0`, `GET /emeter/1`, `GET /emeter/2`
  - Old Shelly EM and 3EM

Notes about the Shelly endpoints:

- The implementation mirrors the JSON fields used by Shelly Gen2 API as implemented in `src/main.cpp`.
- If a Shelly endpoint is called but the underlying JSY model does not support the requested operation (for instance calling `/rpc/EM.GetStatus` on a non-3EM device), the server responds with an HTTP 404 or 500 as coded.

## UDP broadcast format

- By default the device broadcasts UDP packets on port 53964. The packet format includes a small header then a MsgPack-like payload and a CRC32. See `src/main.cpp` for serialization details. The UDP message type for JSY data is 0x02.

## For developers

The project uses these libraries (examples shown in `src/main.cpp` includes):

- ArduinoJson
- AsyncTCP
- ESPAsyncWebServer
- ESPDash
- ElegantOTA
- FastCRC32
- Mycila libraries (MycilaJSY, MycilaESPConnect, MycilaUtilities, etc.)

Other Arduino libraries used (platform-provided): DNSServer, ESP32 Async UDP, ESPmDNS, FS, LittleFS, WiFi, Ticker, Update

## Contributing

Patches, issues and suggestions welcome. If you add a Shelly-compatibility improvement, please add test cases and document any behavior differences.

## License

This project is licensed under the GNU General Public License v3.0. See the [LICENSE](LICENSE) file for details.
