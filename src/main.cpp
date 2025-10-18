// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
 */
#include <Arduino.h>

#include <AsyncUDP.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <Preferences.h>

#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson

#include <AsyncTCP.h>          // https://github.com/ESP32Async/AsyncTCP
#include <ESPAsyncWebServer.h> // https://github.com/ESP32Async/ESPAsyncWebServer

#include <ESPDash.h>    // https://github.com/ayushsharma82/ESP-DASH
#include <ElegantOTA.h> // https://github.com/ayushsharma82/ElegantOTA
#include <FastCRC32.h>  // https://github.com/RobTillaart/CRC
#include <WebSerial.h>  // https://github.com/ESP32Async/ESPAsyncWebServer

#include <MycilaCircularBuffer.h> // https://github.com/mathieucarbou/MycilaUtilities
#include <MycilaESPConnect.h>     // https://github.com/mathieucarbou/MycilaESPConnect
#include <MycilaJSY.h>            // https://github.com/mathieucarbou/MycilaJSY
#include <MycilaSystem.h>         // https://github.com/mathieucarbou/MycilaSystem
#include <MycilaTaskManager.h>    // https://github.com/mathieucarbou/MycilaTaskManager
#include <MycilaTime.h>           // https://github.com/mathieucarbou/MycilaUtilities

#include <algorithm>
#include <string>

#ifndef SOC_UART_HP_NUM
  #define SOC_UART_HP_NUM SOC_UART_NUM
#endif

#if SOC_UART_HP_NUM < 3
  #ifndef Serial2
    #define Serial2 Serial1
  #endif
  #ifndef RX2
    #define RX2 RX1
  #endif
  #ifndef TX2
    #define TX2 TX1
  #endif
#endif

#define MYCILA_ADMIN_PASSWORD        ""
#define MYCILA_ADMIN_USERNAME        "admin"
#define MYCILA_APP_NAME              "MycilaJSY App"
#define MYCILA_GRAPH_POINTS          60
#define MYCILA_UDP_MSG_TYPE_JSY_DATA 0x02 // supports all JSY models
#define MYCILA_UDP_PORT              53964
#define MYCILA_UDP_SEND_RATE_WINDOW  20
#define TAG                          "MycilaJSYApp"

static AsyncUDP udp;
static AsyncWebServer webServer(80);
static AsyncAuthenticationMiddleware authMiddleware;
static Mycila::ESPConnect espConnect(webServer);
static ESPDash dashboard = ESPDash(webServer, "/dashboard", false);
static Preferences preferences;

static Mycila::JSY jsy;
static volatile Mycila::JSY::Data jsyData;
static Mycila::JSY::Data prevData;

static Mycila::TaskManager coreTaskManager("core");
static Mycila::TaskManager jsyTaskManager("jsy");

static dash::StatisticValue networkHostname(dashboard, "Network Hostname");
static dash::StatisticValue networkInterface(dashboard, "Network Interface");
static dash::StatisticValue networkAPIP(dashboard, "Network Access Point IP Address");
static dash::StatisticValue networkAPMAC(dashboard, "Network Access Point MAC Address");
static dash::StatisticValue networkEthIP(dashboard, "Network Ethernet IP Address");
static dash::StatisticValue networkEthMAC(dashboard, "Network Ethernet MAC Address");
static dash::StatisticValue networkWiFiIP(dashboard, "Network WiFi IP Address");
static dash::StatisticValue networkWiFiMAC(dashboard, "Network WiFi MAC Address");
static dash::StatisticValue networkWiFiSSID(dashboard, "Network WiFi SSID");
static dash::StatisticValue networkWiFiRSSI(dashboard, "Network WiFi RSSI");
static dash::StatisticValue networkWiFiSignal(dashboard, "Network WiFi Signal");
static dash::StatisticValue uptime(dashboard, "Uptime");
static dash::StatisticValue version(dashboard, "Version");

static dash::ToggleButtonCard publishDataCard(dashboard, "Publish Data");
static dash::ToggleButtonCard restart(dashboard, "Restart");
static dash::ToggleButtonCard energyReset(dashboard, "Reset Energy");
static dash::ToggleButtonCard reset(dashboard, "Factory Reset");

static dash::GenericCard jsyModelCard(dashboard, "Model");
static dash::GenericCard<float, 2> messageRateCard(dashboard, "Message Rate", "msg/s");
static dash::GenericCard<uint32_t> dataRateCard(dashboard, "Data Rate", "bytes/s");

// JSY-MK-163
static dash::GenericCard<float, 1> jsy163Frequency(dashboard, "Frequency", "Hz");
static dash::GenericCard<float, 1> jsy163Voltage(dashboard, "Voltage", "V");
static dash::GenericCard<float, 2> jsy163current(dashboard, "Current", "A");
static dash::GenericCard<float, 2> jsy163PowerFactor(dashboard, "P Factor");
static dash::GenericCard<float, 0> jsy163ActivePower(dashboard, "Active P", "W");
static dash::GenericCard<float, 0> jsy163ApparentPower(dashboard, "Apparent P", "VA");
static dash::GenericCard<float, 0> jsy163ReactivePower(dashboard, "Reactive P", "VAr");
static dash::GenericCard<uint32_t> jsy163ActiveEnergy(dashboard, "Active E", "Wh");
static dash::GenericCard<uint32_t> jsy163ActiveEnergyImported(dashboard, "Active E Import", "Wh");
static dash::GenericCard<uint32_t> jsy163ActiveEnergyReturned(dashboard, "Active E Return", "Wh");
static dash::BarChart<int8_t, int16_t> jsy163ActivePowerHistory(dashboard, "Active Power (W)");

// JSY-MK-194
static dash::GenericCard<float, 1> jsy194Channel1Frequency(dashboard, "Frequency 1", "Hz");
static dash::GenericCard<float, 1> jsy194Channel1Voltage(dashboard, "Voltage 1", "V");
static dash::GenericCard<float, 2> jsy194Channel1Current(dashboard, "Current 1", "A");
static dash::GenericCard<float, 2> jsy194Channel1PowerFactor(dashboard, "P Factor 1");
static dash::GenericCard<float, 0> jsy194Channel1ActivePower(dashboard, "Active P 1", "W");
static dash::GenericCard<float, 0> jsy194Channel1ApparentPower(dashboard, "Apparent P 1", "VA");
static dash::GenericCard<float, 0> jsy194Channel1ReactivePower(dashboard, "Reactive P 1", "VAr");
static dash::GenericCard<uint32_t> jsy194Channel1ActiveEnergy(dashboard, "Active E 1", "Wh");
static dash::GenericCard<uint32_t> jsy194Channel1ActiveEnergyImported(dashboard, "Active E Import 1", "Wh");
static dash::GenericCard<uint32_t> jsy194Channel1ActiveEnergyReturned(dashboard, "Active E Return 1", "Wh");
static dash::GenericCard<float, 1> jsy194Channel2Frequency(dashboard, "Frequency 2", "Hz");
static dash::GenericCard<float, 1> jsy194Channel2Voltage(dashboard, "Voltage 2", "V");
static dash::GenericCard<float, 2> jsy194Channel2Current(dashboard, "Current 2", "A");
static dash::GenericCard<float, 2> jsy194Channel2PowerFactor(dashboard, "P Factor 2");
static dash::GenericCard<float, 0> jsy194Channel2ActivePower(dashboard, "Active P 2", "W");
static dash::GenericCard<float, 0> jsy194Channel2ApparentPower(dashboard, "Apparent P 2", "VA");
static dash::GenericCard<float, 0> jsy194Channel2ReactivePower(dashboard, "Reactive P 2", "VAr");
static dash::GenericCard<uint32_t> jsy194Channel2ActiveEnergy(dashboard, "Active E 2", "Wh");
static dash::GenericCard<uint32_t> jsy194Channel2ActiveEnergyImported(dashboard, "Active E Import 2", "Wh");
static dash::GenericCard<uint32_t> jsy194Channel2ActiveEnergyReturned(dashboard, "Active E Return 2", "Wh");
static dash::BarChart<int8_t, int16_t> jsy194Channel1ActivePowerHistory(dashboard, "Active P 1 (W)");
static dash::BarChart<int8_t, int16_t> jsy194Channel2ActivePowerHistory(dashboard, "Active P 2 (W)");

// JSY-MK-333
static dash::GenericCard<float, 1> jsy333PhaseAFrequency(dashboard, "Frequency A", "Hz");
static dash::GenericCard<float, 1> jsy333PhaseAVoltage(dashboard, "Voltage A", "V");
static dash::GenericCard<float, 2> jsy333PhaseACurrent(dashboard, "Current A", "A");
static dash::GenericCard<float, 2> jsy333PhaseAPowerFactor(dashboard, "P Factor A");
static dash::GenericCard<float, 0> jsy333PhaseAActivePower(dashboard, "Active Power A", "W");
static dash::GenericCard<float, 0> jsy333PhaseAApparentPower(dashboard, "Apparent P A", "VA");
static dash::GenericCard<float, 0> jsy333PhaseAReactivePower(dashboard, "Reactive P A", "VAr");
static dash::GenericCard<uint32_t> jsy333PhaseAActiveEnergy(dashboard, "Active E A", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseAActiveEnergyImported(dashboard, "Active E Import A", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseAActiveEnergyReturned(dashboard, "Active E Return A", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseAReactiveEnergy(dashboard, "Reactive E A", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseAReactiveEnergyImported(dashboard, "Reactive E Import A", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseAReactiveEnergyReturned(dashboard, "Reactive E Return A", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseAApparentEnergy(dashboard, "Apparent E A", "VAh");
static dash::GenericCard<float, 1> jsy333PhaseBFrequency(dashboard, "Frequency B", "Hz");
static dash::GenericCard<float, 1> jsy333PhaseBVoltage(dashboard, "Voltage B", "V");
static dash::GenericCard<float, 2> jsy333PhaseBCurrent(dashboard, "Current B", "A");
static dash::GenericCard<float, 2> jsy333PhaseBPowerFactor(dashboard, "P Factor B");
static dash::GenericCard<float, 0> jsy333PhaseBActivePower(dashboard, "Active P B", "W");
static dash::GenericCard<float, 0> jsy333PhaseBApparentPower(dashboard, "Apparent P B", "VA");
static dash::GenericCard<float, 0> jsy333PhaseBReactivePower(dashboard, "Reactive P B", "VAr");
static dash::GenericCard<uint32_t> jsy333PhaseBActiveEnergy(dashboard, "Active E B", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseBActiveEnergyImported(dashboard, "Active E Import B", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseBActiveEnergyReturned(dashboard, "Active E Return B", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseBReactiveEnergy(dashboard, "Reactive E B", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseBReactiveEnergyImported(dashboard, "Reactive E Import B", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseBReactiveEnergyReturned(dashboard, "Reactive E Return B", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseBApparentEnergy(dashboard, "Apparent E B", "VAh");
static dash::GenericCard<float, 1> jsy333PhaseCFrequency(dashboard, "Frequency C", "Hz");
static dash::GenericCard<float, 1> jsy333PhaseCVoltage(dashboard, "Voltage C", "V");
static dash::GenericCard<float, 2> jsy333PhaseCCurrent(dashboard, "Current C", "A");
static dash::GenericCard<float, 2> jsy333PhaseCPowerFactor(dashboard, "P Factor C");
static dash::GenericCard<float, 0> jsy333PhaseCActivePower(dashboard, "Active P C", "W");
static dash::GenericCard<float, 0> jsy333PhaseCApparentPower(dashboard, "Apparent P C", "VA");
static dash::GenericCard<float, 0> jsy333PhaseCReactivePower(dashboard, "Reactive P C", "VAr");
static dash::GenericCard<uint32_t> jsy333PhaseCActiveEnergy(dashboard, "Active E C", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseCActiveEnergyImported(dashboard, "Active E Import C", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseCActiveEnergyReturned(dashboard, "Active E Return C", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseCReactiveEnergy(dashboard, "Reactive E C", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseCReactiveEnergyImported(dashboard, "Reactive E Import C", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseCReactiveEnergyReturned(dashboard, "Reactive E Return C", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseCApparentEnergy(dashboard, "Apparent E C", "VAh");
static dash::BarChart<int8_t, int16_t> jsy333PhaseAActivePowerHistory(dashboard, "Active P A (W)");
static dash::BarChart<int8_t, int16_t> jsy333PhaseBActivePowerHistory(dashboard, "Active P B (W)");
static dash::BarChart<int8_t, int16_t> jsy333PhaseCActivePowerHistory(dashboard, "Active P C (W)");

// graphs
static int8_t historyX[MYCILA_GRAPH_POINTS] = {0};
static int16_t power1HistoryY[MYCILA_GRAPH_POINTS] = {0};
static int16_t power2HistoryY[MYCILA_GRAPH_POINTS] = {0};
static int16_t power3HistoryY[MYCILA_GRAPH_POINTS] = {0};

static bool udpSendEnabled = true;
static uint16_t jsyModel = MYCILA_JSY_MK_UNKNOWN;

// circular buffer for msg rate
static Mycila::CircularBuffer<float, MYCILA_UDP_SEND_RATE_WINDOW> messageRateBuffer;
static volatile float messageRate = 0;

// circular buffer for data rate
static Mycila::CircularBuffer<uint32_t, MYCILA_UDP_SEND_RATE_WINDOW> dataRateBuffer;
static volatile uint32_t dataRate = 0;

static Mycila::Task jsyTask("JSY", [](void* params) { jsy.read(); });

static Mycila::Task networkManagerTask("ESPConnect", [](void* params) { espConnect.loop(); });

static Mycila::Task networkUpTask("Network UP", Mycila::Task::Type::ONCE, [](void* params) {
  ESP_LOGI(TAG, "Enable Network Services...");

  // Web server
  ESP_LOGI(TAG, "Enable Web Server...");
  webServer.begin();
  webServer.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404);
  });

  // mDNS
#ifndef ESPCONNECT_NO_MDNS
  ESP_LOGI(TAG, "Enable mDNS...");
  MDNS.addService("http", "tcp", 80);
#endif
});

static Mycila::Task otaTask("OTA", Mycila::Task::Type::ONCE, [](void* params) {
  ESP_LOGI(TAG, "Preparing OTA update...");
  udpSendEnabled = false;
  jsy.end();
});

static Mycila::Task restartTask("Restart", Mycila::Task::Type::ONCE, [](void* params) {
  ESP_LOGW(TAG, "Restarting " MYCILA_APP_NAME "...");
  Mycila::System::restart(500);
});

static Mycila::Task dashboardTask("Dashboard", [](void* params) {
  Mycila::ESPConnect::Mode mode = espConnect.getMode();

  networkAPIP.setValue(espConnect.getIPAddress(Mycila::ESPConnect::Mode::AP).toString().c_str());
  networkEthIP.setValue(espConnect.getIPAddress(Mycila::ESPConnect::Mode::ETH).toString().c_str());
  networkInterface.setValue(mode == Mycila::ESPConnect::Mode::AP ? "AP" : (mode == Mycila::ESPConnect::Mode::STA ? "WiFi" : (mode == Mycila::ESPConnect::Mode::ETH ? "Ethernet" : "")));
  networkWiFiIP.setValue(espConnect.getIPAddress(Mycila::ESPConnect::Mode::STA).toString().c_str());
  networkWiFiRSSI.setValue((std::to_string(espConnect.getWiFiRSSI()) + " dBm"));
  networkWiFiSignal.setValue((std::to_string(espConnect.getWiFiSignalQuality()) + " %"));
  networkWiFiSSID.setValue(espConnect.getWiFiSSID());
  uptime.setValue(Mycila::Time::toDHHMMSS(Mycila::System::getUptime()));

  messageRateCard.setValue(messageRate);
  dataRateCard.setValue(dataRate);
  publishDataCard.setValue(udpSendEnabled);

  switch (jsyModel) {
    case MYCILA_JSY_MK_163: {
      jsy163Frequency.setValue(prevData.single().frequency);
      jsy163Voltage.setValue(prevData.single().voltage);
      jsy163current.setValue(prevData.single().current);
      jsy163PowerFactor.setValue(prevData.single().powerFactor);
      jsy163ActivePower.setValue(prevData.single().activePower);
      jsy163ApparentPower.setValue(prevData.single().apparentPower);
      jsy163ReactivePower.setValue(prevData.single().reactivePower);
      jsy163ActiveEnergy.setValue(prevData.single().activeEnergy);
      jsy163ActiveEnergyImported.setValue(prevData.single().activeEnergyImported);
      jsy163ActiveEnergyReturned.setValue(prevData.single().activeEnergyReturned);

      // shift array
      for (size_t i = 0; i < MYCILA_GRAPH_POINTS - 1; i++) {
        power1HistoryY[i] = power1HistoryY[i + 1];
      }

      // set new value
      power1HistoryY[MYCILA_GRAPH_POINTS - 1] = round(prevData.phaseA().activePower);

      // update charts
      jsy163ActivePowerHistory.setY(power1HistoryY, MYCILA_GRAPH_POINTS);

      break;
    }
    case MYCILA_JSY_MK_193:
    case MYCILA_JSY_MK_194: {
      jsy194Channel1Frequency.setValue(prevData.channel1().frequency);
      jsy194Channel1Voltage.setValue(prevData.channel1().voltage);
      jsy194Channel1Current.setValue(prevData.channel1().current);
      jsy194Channel1PowerFactor.setValue(prevData.channel1().powerFactor);
      jsy194Channel1ActivePower.setValue(prevData.channel1().activePower);
      jsy194Channel1ApparentPower.setValue(prevData.channel1().apparentPower);
      jsy194Channel1ReactivePower.setValue(prevData.channel1().reactivePower);
      jsy194Channel1ActiveEnergy.setValue(prevData.channel1().activeEnergy);
      jsy194Channel1ActiveEnergyImported.setValue(prevData.channel1().activeEnergyImported);
      jsy194Channel1ActiveEnergyReturned.setValue(prevData.channel1().activeEnergyReturned);

      jsy194Channel2Frequency.setValue(prevData.channel2().frequency);
      jsy194Channel2Voltage.setValue(prevData.channel2().voltage);
      jsy194Channel2Current.setValue(prevData.channel2().current);
      jsy194Channel2PowerFactor.setValue(prevData.channel2().powerFactor);
      jsy194Channel2ActivePower.setValue(prevData.channel2().activePower);
      jsy194Channel2ApparentPower.setValue(prevData.channel2().apparentPower);
      jsy194Channel2ReactivePower.setValue(prevData.channel2().reactivePower);
      jsy194Channel2ActiveEnergy.setValue(prevData.channel2().activeEnergy);
      jsy194Channel2ActiveEnergyImported.setValue(prevData.channel2().activeEnergyImported);
      jsy194Channel2ActiveEnergyReturned.setValue(prevData.channel2().activeEnergyReturned);

      // shift array
      for (size_t i = 0; i < MYCILA_GRAPH_POINTS - 1; i++) {
        power1HistoryY[i] = power1HistoryY[i + 1];
        power2HistoryY[i] = power2HistoryY[i + 1];
      }

      // set new value
      power1HistoryY[MYCILA_GRAPH_POINTS - 1] = round(prevData.channel1().activePower);
      power2HistoryY[MYCILA_GRAPH_POINTS - 1] = round(prevData.channel2().activePower);

      // update charts
      jsy194Channel1ActivePowerHistory.setY(power1HistoryY, MYCILA_GRAPH_POINTS);
      jsy194Channel2ActivePowerHistory.setY(power2HistoryY, MYCILA_GRAPH_POINTS);

      break;
    }
    case MYCILA_JSY_MK_333: {
      jsy333PhaseAFrequency.setValue(prevData.phaseA().frequency);
      jsy333PhaseAVoltage.setValue(prevData.phaseA().voltage);
      jsy333PhaseACurrent.setValue(prevData.phaseA().current);
      jsy333PhaseAPowerFactor.setValue(prevData.phaseA().powerFactor);
      jsy333PhaseAActivePower.setValue(prevData.phaseA().activePower);
      jsy333PhaseAApparentPower.setValue(prevData.phaseA().apparentPower);
      jsy333PhaseAReactivePower.setValue(prevData.phaseA().reactivePower);
      jsy333PhaseAActiveEnergy.setValue(prevData.phaseA().activeEnergy);
      jsy333PhaseAActiveEnergyImported.setValue(prevData.phaseA().activeEnergyImported);
      jsy333PhaseAActiveEnergyReturned.setValue(prevData.phaseA().activeEnergyReturned);
      jsy333PhaseAReactiveEnergy.setValue(prevData.phaseA().reactiveEnergy);
      jsy333PhaseAReactiveEnergyImported.setValue(prevData.phaseA().reactiveEnergyImported);
      jsy333PhaseAReactiveEnergyReturned.setValue(prevData.phaseA().reactiveEnergyReturned);
      jsy333PhaseAApparentEnergy.setValue(prevData.phaseA().apparentEnergy);

      jsy333PhaseBFrequency.setValue(prevData.phaseB().frequency);
      jsy333PhaseBVoltage.setValue(prevData.phaseB().voltage);
      jsy333PhaseBCurrent.setValue(prevData.phaseB().current);
      jsy333PhaseBPowerFactor.setValue(prevData.phaseB().powerFactor);
      jsy333PhaseBActivePower.setValue(prevData.phaseB().activePower);
      jsy333PhaseBApparentPower.setValue(prevData.phaseB().apparentPower);
      jsy333PhaseBReactivePower.setValue(prevData.phaseB().reactivePower);
      jsy333PhaseBActiveEnergy.setValue(prevData.phaseB().activeEnergy);
      jsy333PhaseBActiveEnergyImported.setValue(prevData.phaseB().activeEnergyImported);
      jsy333PhaseBActiveEnergyReturned.setValue(prevData.phaseB().activeEnergyReturned);
      jsy333PhaseBReactiveEnergy.setValue(prevData.phaseB().reactiveEnergy);
      jsy333PhaseBReactiveEnergyImported.setValue(prevData.phaseB().reactiveEnergyImported);
      jsy333PhaseBReactiveEnergyReturned.setValue(prevData.phaseB().reactiveEnergyReturned);
      jsy333PhaseBApparentEnergy.setValue(prevData.phaseB().apparentEnergy);

      jsy333PhaseCFrequency.setValue(prevData.phaseC().frequency);
      jsy333PhaseCVoltage.setValue(prevData.phaseC().voltage);
      jsy333PhaseCCurrent.setValue(prevData.phaseC().current);
      jsy333PhaseCPowerFactor.setValue(prevData.phaseC().powerFactor);
      jsy333PhaseCActivePower.setValue(prevData.phaseC().activePower);
      jsy333PhaseCApparentPower.setValue(prevData.phaseC().apparentPower);
      jsy333PhaseCReactivePower.setValue(prevData.phaseC().reactivePower);
      jsy333PhaseCActiveEnergy.setValue(prevData.phaseC().activeEnergy);
      jsy333PhaseCActiveEnergyImported.setValue(prevData.phaseC().activeEnergyImported);
      jsy333PhaseCActiveEnergyReturned.setValue(prevData.phaseC().activeEnergyReturned);
      jsy333PhaseCReactiveEnergy.setValue(prevData.phaseC().reactiveEnergy);
      jsy333PhaseCReactiveEnergyImported.setValue(prevData.phaseC().reactiveEnergyImported);
      jsy333PhaseCReactiveEnergyReturned.setValue(prevData.phaseC().reactiveEnergyReturned);
      jsy333PhaseCApparentEnergy.setValue(prevData.phaseC().apparentEnergy);

      // shift array
      for (size_t i = 0; i < MYCILA_GRAPH_POINTS - 1; i++) {
        power1HistoryY[i] = power1HistoryY[i + 1];
        power2HistoryY[i] = power2HistoryY[i + 1];
        power3HistoryY[i] = power3HistoryY[i + 1];
      }

      // set new value
      power1HistoryY[MYCILA_GRAPH_POINTS - 1] = round(prevData.phaseA().activePower);
      power2HistoryY[MYCILA_GRAPH_POINTS - 1] = round(prevData.phaseB().activePower);
      power3HistoryY[MYCILA_GRAPH_POINTS - 1] = round(prevData.phaseC().activePower);

      // update charts
      jsy333PhaseAActivePowerHistory.setY(power1HistoryY, MYCILA_GRAPH_POINTS);
      jsy333PhaseBActivePowerHistory.setY(power2HistoryY, MYCILA_GRAPH_POINTS);
      jsy333PhaseCActivePowerHistory.setY(power3HistoryY, MYCILA_GRAPH_POINTS);

      break;
    }
    default:
      break;
  }

  dashboard.sendUpdates();
});

static int log_redirect_vprintf(const char* format, va_list args) {
  size_t written = Serial.vprintf(format, args);
  if (WebSerial.getConnectionCount())
    WebSerial.vprintf(format, args);
  return written;
}

static int get_id_param(const AsyncWebServerRequest* request) {
  const AsyncWebParameter* idParam = request->getParam("id");
  if (idParam == nullptr) {
    return -1;
  }
  int id = idParam->value().toInt();
  // JSY-MK-163 and JSY1031 => id must be 0
  // JSY-MK-333 => id can be 0, 1 or 2
  // others, 2 channels => id must be 0 or 1
  if (prevData.model == MYCILA_JSY_MK_163 || prevData.model == MYCILA_JSY_MK_1031) {
    return id == 0 ? 0 : -1;
  }
  if (prevData.model == MYCILA_JSY_MK_333) {
    return id < 0 || id > 2 ? -1 : id;
  }
  return id == 0 || id == 1 ? id : -1;
}

void setup() {
  Serial.begin(115200);

#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
  delay(100);
#else
  while (!Serial)
    yield();
#endif

  // logging
  esp_log_level_set("*", ESP_LOG_DEBUG);
  // esp_log_level_set("*", ESP_LOG_INFO);
  // esp_log_level_set("ARDUINO", ESP_LOG_DEBUG);
  esp_log_set_vprintf(log_redirect_vprintf);

  // hostname
  static std::string hostname = std::string("jsy-") + Mycila::System::getChipIDStr();
  std::transform(hostname.begin(), hostname.end(), hostname.begin(), ::tolower);

  // logging
  ESP_LOGW(TAG, "Booting " MYCILA_APP_NAME "...");

  // system
  disableLoopWDT();
  Mycila::System::init();

  // config
  preferences.begin("jsy", false);
  udpSendEnabled = preferences.getBool("udp_send", udpSendEnabled);

  // tasks
  dashboardTask.setEnabledWhen([]() { return espConnect.isConnected() && !dashboard.isAsyncAccessInProgress(); });
  dashboardTask.setInterval(1000);
  jsyTask.setEnabledWhen([]() { return jsy.isEnabled(); });
  jsyTaskManager.addTask(jsyTask);
  coreTaskManager.addTask(dashboardTask);
  coreTaskManager.addTask(networkManagerTask);
  coreTaskManager.addTask(networkUpTask);
  coreTaskManager.addTask(otaTask);
  coreTaskManager.addTask(restartTask);

  // WebSerial
  WebSerial.begin(&webServer, "/console");

  // ElegantOTA
  ElegantOTA.setAutoReboot(false);
  ElegantOTA.onStart([]() { otaTask.resume(); });
  ElegantOTA.onEnd([](bool success) {
    if (success) {
      ESP_LOGI(TAG, "OTA Update Success! Restarting...");
    } else {
      ESP_LOGE(TAG, "OTA Failed! Restarting...");
    }
    restartTask.resume();
  });
  ElegantOTA.begin(&webServer);

  // Dashboard - Auth
  authMiddleware.setAuthType(AsyncAuthType::AUTH_DIGEST);
  authMiddleware.setRealm(hostname.c_str());
  authMiddleware.setUsername(MYCILA_ADMIN_USERNAME);
  authMiddleware.setPassword(MYCILA_ADMIN_PASSWORD);
  authMiddleware.generateHash();
  webServer.addMiddleware(&authMiddleware);

  // Dashboard - Routes
  webServer.rewrite("/", "/dashboard").setFilter([](AsyncWebServerRequest* request) { return espConnect.getState() != Mycila::ESPConnect::State::PORTAL_STARTED; });

  // API: /api/jsy/reset
  // Resets the energy counters
  webServer.on("/api/jsy/reset", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest* request) {
    // if the reset cannot be done because the JSY is too busy then 409 conflict is returned
    request->send(jsy.resetEnergy() ? 200 : 409);
  });
  // API: /api/jsy/publish
  // With no query parameter: returns the UDP data publishing state
  // With "switch" query parameter set to "on" or "off": enables or disables UDP data publishing
  webServer.on("/api/jsy/publish", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest* request) {
    const AsyncWebParameter* enableParam = request->getParam("switch");
    if (enableParam != nullptr) {
      udpSendEnabled = enableParam->value() == "on";
      preferences.putBool("udp_send", udpSendEnabled);
      publishDataCard.setValue(udpSendEnabled);
      dashboard.refresh(publishDataCard);
    }
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    root["switch"] = udpSendEnabled ? "on" : "off";
    request->send(200);
  });

  // API: /api/jsy
  // Returns the current JSY data in JSON format
  webServer.on("/api/jsy", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    jsy.toJson(response->getRoot());
    response->setLength();
    request->send(response);
  });
  // API: /api/restart
  // Restarts the device
  webServer.on("/api/restart", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest* request) {
    restartTask.resume();
    request->send(200);
  });
  // API: /api/reset
  // Resets the device to factory defaults
  webServer.on("/api/reset", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest* request) {
    espConnect.clearConfiguration();
    restartTask.resume();
    request->send(200);
  });
  // API: /api
  // Returns the list of available API endpoints
  webServer.on("/api", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    root["/api/jsy/reset"] = "Resets the energy counters";
    root["/api/jsy/publish?switch=on"] = "Enables UDP data publishing";
    root["/api/jsy/publish?switch=off"] = "Disables UDP data publishing";
    root["/api/jsy"] = "Returns the current JSY data";
    root["/api/restart"] = "Restarts the device";
    root["/api/reset"] = "Resets the device to factory defaults";
    response->setLength();
    request->send(response);
  });

  // API Routes to fake Shelly EM and 3EM

  // API: /rpc/Shelly.GetDeviceInfo
  // For: Shelly EM & Shelly Pro EM
  // Return the device information
  webServer.on("/rpc/Shelly.GetDeviceInfo", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    root["name"] = "Mycila JSY App";
    root["id"] = (prevData.model == MYCILA_JSY_MK_333 ? "shellypro3em-" : "shellyproem50-") + espConnect.getMACAddress();
    root["mac"] = espConnect.getMACAddress();
    root["ver"] = MYCILA_JSY_VERSION;
    root["app"] = prevData.model == MYCILA_JSY_MK_333 ? "3EM" : "EM";
    response->setLength();
    request->send(response);
  });
  // API: /rpc/EM1.GetStatus?id=0|1|2
  // For: Shelly EM & 3EM
  // Ref: https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EM1/#em1getstatus-example
  // Example: { "id": 0, "voltage": 240.2, "current": 6.473, "act_power": 1327.6, "aprt_power": 1557.6, "pf": 0.87, "freq": 50, "calibration": "factory" }
  webServer.on("/rpc/EM1.GetStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
    int id = get_id_param(request);
    if (id == -1) {
      request->send(500);
      return;
    }
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    root["id"] = id;
    if (prevData.model == MYCILA_JSY_MK_163 || prevData.model == MYCILA_JSY_MK_1031) {
      root["voltage"] = prevData.single().voltage;
      root["current"] = prevData.single().current;
      root["act_power"] = prevData.single().activePower;
      root["aprt_power"] = prevData.single().apparentPower;
      root["pf"] = prevData.single().powerFactor;
      root["freq"] = prevData.single().frequency;
    } else if (prevData.model == MYCILA_JSY_MK_333) {
      root["voltage"] = prevData.phase(id).voltage;
      root["current"] = prevData.phase(id).current;
      root["act_power"] = prevData.phase(id).activePower;
      root["aprt_power"] = prevData.phase(id).apparentPower;
      root["pf"] = prevData.phase(id).powerFactor;
      root["freq"] = prevData.phase(id).frequency;
    } else {
      root["voltage"] = prevData.channel(id).voltage;
      root["current"] = prevData.channel(id).current;
      root["act_power"] = prevData.channel(id).activePower;
      root["aprt_power"] = prevData.channel(id).apparentPower;
      root["pf"] = prevData.channel(id).powerFactor;
      root["freq"] = prevData.channel(id).frequency;
    }
    root["calibration"] = "factory";
    response->setLength();
    request->send(response);
  });
  // API: /rpc/EM1Data.GetStatus?id=0|1|2
  // For: Shelly EM & 3EM
  // Ref: https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EM1Data/#em1datagetstatus-example
  // Example: { "id": 0, "total_act_energy": 2776175.11, "total_act_ret_energy": 571584.87 }
  webServer.on("/rpc/EM1Data.GetStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
    int id = get_id_param(request);
    if (id == -1) {
      request->send(500);
      return;
    }
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    root["id"] = id;
    if (prevData.model == MYCILA_JSY_MK_163 || prevData.model == MYCILA_JSY_MK_1031) {
      root["total_act_energy"] = prevData.single().activeEnergy;
      root["total_act_ret_energy"] = prevData.single().activeEnergyReturned;
    } else if (prevData.model == MYCILA_JSY_MK_333) {
      root["total_act_energy"] = prevData.phase(id).activeEnergy;
      root["total_act_ret_energy"] = prevData.phase(id).activeEnergyReturned;
    } else {
      root["total_act_energy"] = prevData.channel(id).activeEnergy;
      root["total_act_ret_energy"] = prevData.channel(id).activeEnergyReturned;
    }
    response->setLength();
    request->send(response);
  });
  // API: /rpc/EM.GetStatus?id=0|1|2
  // For: Shelly 3EM
  // Ref: https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EM/#emgetstatus-example
  // Example:
  // {
  //   "id": 0,
  //   "a_current": 4.029,
  //   "a_voltage": 236.1,
  //   "a_act_power": 951.2,
  //   "a_aprt_power": 951.9,
  //   "a_pf": 1,
  //   "a_freq": 50,
  //   "b_current": 4.027,
  //   "b_voltage": 236.201,
  //   "b_act_power": -951.1,
  //   "b_aprt_power": 951.8,
  //   "b_pf": 1,
  //   "b_freq": 50,
  //   "c_current": 3.03,
  //   "c_voltage": 236.402,
  //   "c_active_power": 715.4,
  //   "c_aprt_power": 716.2,
  //   "c_pf": 1,
  //   "c_freq": 50,
  //   "n_current": 11.029,
  //   "total_current": 11.083,
  //   "total_act_power": 2484.782,
  //   "total_aprt_power": 2486.7,
  //   "user_calibrated_phase": [],
  //   "errors": [
  //     "phase_sequence"
  //   ]
  // }
  webServer.on("/rpc/EM.GetStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (prevData.model != MYCILA_JSY_MK_333) {
      request->send(404);
      return;
    }
    int id = get_id_param(request);
    if (id == -1) {
      request->send(500);
      return;
    }
    // model is JSY-MK-333 and id == 0, 1 or 2
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    root["id"] = id;
    root["a_current"] = prevData.phaseA().current;
    root["a_voltage"] = prevData.phaseA().voltage;
    root["a_act_power"] = prevData.phaseA().activePower;
    root["a_aprt_power"] = prevData.phaseA().apparentPower;
    root["a_pf"] = prevData.phaseA().powerFactor;
    root["a_freq"] = prevData.phaseA().frequency;
    root["b_current"] = prevData.phaseB().current;
    root["b_voltage"] = prevData.phaseB().voltage;
    root["b_act_power"] = prevData.phaseB().activePower;
    root["b_aprt_power"] = prevData.phaseB().apparentPower;
    root["b_pf"] = prevData.phaseB().powerFactor;
    root["b_freq"] = prevData.phaseB().frequency;
    root["c_current"] = prevData.phaseC().current;
    root["c_voltage"] = prevData.phaseC().voltage;
    root["c_active_power"] = prevData.phaseC().activePower;
    root["c_aprt_power"] = prevData.phaseC().apparentPower;
    root["c_pf"] = prevData.phaseC().powerFactor;
    root["c_freq"] = prevData.phaseC().frequency;
    root["n_current"] = 0;
    root["total_current"] = prevData.aggregate.current;
    root["total_act_power"] = prevData.aggregate.activePower;
    root["total_aprt_power"] = prevData.aggregate.apparentPower;
    root["user_calibrated_phase"] = JsonArray();
    root["errors"] = JsonArray();
    response->setLength();
    request->send(response);
  });
  // API: /rpc/EMData.GetStatus?id=0|1|2
  // For: Shelly 3EM
  // Ref: https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/EMData/#emdatagetstatus-example
  // Example:
  // {
  //   "id": 0,
  //   "a_total_act_energy": 0,
  //   "a_total_act_ret_energy": 0,
  //   "b_total_act_energy": 0,
  //   "b_total_act_ret_energy": 0,
  //   "c_total_act_energy": 0,
  //   "c_total_act_ret_energy": 0,
  //   "total_act": 0,
  //   "total_act_ret": 0
  // }
  webServer.on("/rpc/EMData.GetStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (prevData.model != MYCILA_JSY_MK_333) {
      request->send(404);
      return;
    }
    int id = get_id_param(request);
    if (id == -1) {
      request->send(500);
      return;
    }
    // model is JSY-MK-333 and id == 0, 1 or 2
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    root["id"] = id;
    root["a_total_act_energy"] = prevData.phaseA().activeEnergy;
    root["a_total_act_ret_energy"] = prevData.phaseA().activeEnergyReturned;
    root["b_total_act_energy"] = prevData.phaseB().activeEnergy;
    root["b_total_act_ret_energy"] = prevData.phaseB().activeEnergyReturned;
    root["c_total_act_energy"] = prevData.phaseC().activeEnergy;
    root["c_total_act_ret_energy"] = prevData.phaseC().activeEnergyReturned;
    root["total_act"] = prevData.aggregate.activeEnergy;
    root["total_act_ret"] = prevData.aggregate.activeEnergyReturned;
    response->setLength();
    request->send(response);
  });
  // API: /rpc
  // For: Shelly EM & 3EM
  // Returns the list of available API endpoints
  webServer.on("/rpc", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    root["/rpc/Shelly.GetDeviceInfo"] = "Returns the device information";
    if (prevData.model == MYCILA_JSY_MK_163 || prevData.model == MYCILA_JSY_MK_1031) {
      root["/rpc/EM1.GetStatus?id=0"] = "Returns the current EM1 status";
      root["/rpc/EM1Data.GetStatus?id=0"] = "Returns the current EM1 data status";
    } else if (prevData.model == MYCILA_JSY_MK_333) {
      root["/rpc/EM1.GetStatus?id=0"] = "Returns the current EM1 status for phase A";
      root["/rpc/EM1.GetStatus?id=1"] = "Returns the current EM1 status for phase B";
      root["/rpc/EM1.GetStatus?id=2"] = "Returns the current EM1 status for phase C";
      root["/rpc/EM1Data.GetStatus?id=0"] = "Returns the current EM1 data status for phase A";
      root["/rpc/EM1Data.GetStatus?id=1"] = "Returns the current EM1 data status for phase B";
      root["/rpc/EM1Data.GetStatus?id=2"] = "Returns the current EM1 data status for phase C";
      root["/rpc/EM.GetStatus?id=0"] = "Returns the current EM status for all phases";
      root["/rpc/EM.GetStatus?id=1"] = "Returns the current EM status for all phases";
      root["/rpc/EM.GetStatus?id=2"] = "Returns the current EM status for all phases";
      root["/rpc/EMData.GetStatus?id=0"] = "Returns the current EM data status for all phases";
      root["/rpc/EMData.GetStatus?id=1"] = "Returns the current EM data status for all phases";
      root["/rpc/EMData.GetStatus?id=2"] = "Returns the current EM data status for all phases";
    } else {
      root["/rpc/EM1.GetStatus?id=0"] = "Returns the current EM1 status for channel 1";
      root["/rpc/EM1.GetStatus?id=1"] = "Returns the current EM1 status for channel 2";
      root["/rpc/EM1Data.GetStatus?id=0"] = "Returns the current EM1 data status for channel 1";
      root["/rpc/EM1Data.GetStatus?id=1"] = "Returns the current EM1 data status for channel 2";
    }
    response->setLength();
    request->send(response);
  });

  // Dashboard - Callbacks
  restart.onChange([](bool state) { restartTask.resume(); });
  reset.onChange([](bool state) {
    espConnect.clearConfiguration();
    restartTask.resume();
  });
  energyReset.onChange([](bool state) {
    jsy.resetEnergy();
    energyReset.setValue(0);
    dashboard.refresh(energyReset);
  });
  publishDataCard.onChange([](bool state) {
    udpSendEnabled = state;
    preferences.putBool("udp_send", udpSendEnabled);
    publishDataCard.setValue(udpSendEnabled);
    dashboard.refresh(publishDataCard);
  });

  dashboard.onBeforeUpdate([](bool changes_only) {
    if (!changes_only) {
      ESP_LOGI(TAG, "Dashboard refresh requested");
      jsyModelCard.setValue(jsyModel == MYCILA_JSY_MK_UNKNOWN ? "Unknown" : jsy.getModelName());
      networkAPMAC.setValue(espConnect.getMACAddress(Mycila::ESPConnect::Mode::AP));
      networkEthMAC.setValue(espConnect.getMACAddress(Mycila::ESPConnect::Mode::ETH).empty() ? std::string("N/A") : espConnect.getMACAddress(Mycila::ESPConnect::Mode::ETH));
      networkHostname.setValue(hostname);
      networkWiFiMAC.setValue(espConnect.getMACAddress(Mycila::ESPConnect::Mode::STA));
    }
  });

  // Dashboard - Static Widgets Values
  version.setValue(MYCILA_JSY_VERSION);
  for (int i = 0; i < MYCILA_GRAPH_POINTS; i++)
    historyX[i] = i - MYCILA_GRAPH_POINTS;
  jsy163ActivePowerHistory.setX(historyX, MYCILA_GRAPH_POINTS);
  jsy194Channel1ActivePowerHistory.setX(historyX, MYCILA_GRAPH_POINTS);
  jsy194Channel2ActivePowerHistory.setX(historyX, MYCILA_GRAPH_POINTS);
  jsy333PhaseAActivePowerHistory.setX(historyX, MYCILA_GRAPH_POINTS);
  jsy333PhaseBActivePowerHistory.setX(historyX, MYCILA_GRAPH_POINTS);
  jsy333PhaseCActivePowerHistory.setX(historyX, MYCILA_GRAPH_POINTS);
  dashboard.refreshLayout();
  dashboardTask.forceRun();

  // Network Manager
  espConnect.setAutoRestart(true);
  espConnect.setBlocking(CRC16_CCITT_FALSE_REV_IN);
  espConnect.listen([](Mycila::ESPConnect::State previous, Mycila::ESPConnect::State state) {
    ESP_LOGD(TAG, "NetworkState: %s => %s", espConnect.getStateName(previous), espConnect.getStateName(state));
    switch (state) {
      case Mycila::ESPConnect::State::NETWORK_DISABLED:
        ESP_LOGW(TAG, "Disabled Network!");
        break;
      case Mycila::ESPConnect::State::AP_STARTING:
        ESP_LOGI(TAG, "Starting Access Point %s...", espConnect.getAccessPointSSID().c_str());
        break;
      case Mycila::ESPConnect::State::AP_STARTED:
        ESP_LOGI(TAG, "Access Point %s started with IP address %s", espConnect.getWiFiSSID().c_str(), espConnect.getIPAddress().toString().c_str());
        networkUpTask.resume();
        break;
      case Mycila::ESPConnect::State::NETWORK_CONNECTING:
        ESP_LOGI(TAG, "Connecting to network...");
        break;
      case Mycila::ESPConnect::State::NETWORK_CONNECTED:
        ESP_LOGI(TAG, "Connected with IP address %s", espConnect.getIPAddress().toString().c_str());
        networkUpTask.resume();
        break;
      case Mycila::ESPConnect::State::NETWORK_TIMEOUT:
        ESP_LOGW(TAG, "Unable to connect!");
        break;
      case Mycila::ESPConnect::State::NETWORK_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected!");
        break;
      case Mycila::ESPConnect::State::NETWORK_RECONNECTING:
        ESP_LOGI(TAG, "Trying to reconnect...");
        break;
      case Mycila::ESPConnect::State::PORTAL_STARTING:
        ESP_LOGI(TAG, "Starting Captive Portal %s for %" PRIu32 " seconds...", espConnect.getAccessPointSSID().c_str(), espConnect.getCaptivePortalTimeout());
        break;
      case Mycila::ESPConnect::State::PORTAL_STARTED:
        ESP_LOGI(TAG, "Captive Portal started at %s with IP address %s", espConnect.getWiFiSSID().c_str(), espConnect.getIPAddress().toString().c_str());
        break;
      case Mycila::ESPConnect::State::PORTAL_COMPLETE: {
        if (espConnect.getConfig().apMode) {
          ESP_LOGI(TAG, "Captive Portal: Access Point configured");
        } else {
          ESP_LOGI(TAG, "Captive Portal: WiFi configured");
        }
        break;
      }
      case Mycila::ESPConnect::State::PORTAL_TIMEOUT:
        ESP_LOGW(TAG, "Captive Portal: timed out.");
        break;
      default:
        break;
    }
  });
  espConnect.begin(hostname.c_str(), hostname.c_str(), MYCILA_ADMIN_PASSWORD);

  // jsy
  jsy.setCallback([](const Mycila::JSY::EventType eventType, const Mycila::JSY::Data& data) {
    if (prevData == data)
      return;

    prevData = data;

    if (!udpSendEnabled) {
      messageRate = 0;
      dataRate = 0;
      return;
    }

    const Mycila::ESPConnect::Mode mode = espConnect.getMode();
    if (mode == Mycila::ESPConnect::Mode::NONE) {
      messageRate = 0;
      dataRate = 0;
      return;
    }

    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    jsy.toJson(root);

    // buffer[0] == MYCILA_UDP_MSG_TYPE_JSY_DATA (1)
    // buffer[1] == size_t (4)
    // buffer[5] == MsgPack (?)
    // buffer[5 + size] == CRC32 (4)
    size_t size = measureMsgPack(doc);
    size_t packetSize = size + 9;
    // uint8_t buffer[packetSize];
    uint8_t* buffer = new uint8_t[packetSize];
    buffer[0] = MYCILA_UDP_MSG_TYPE_JSY_DATA;
    memcpy(buffer + 1, &size, 4);
    serializeMsgPack(root, buffer + 5, size);

    // crc32
    FastCRC32 crc32;
    crc32.add(buffer, size + 5);
    uint32_t crc = crc32.calc();
    memcpy(buffer + size + 5, &crc, 4);

    if (packetSize > CONFIG_TCP_MSS) {
      ESP_LOGE(TAG, "Packet size too big: %" PRIu32 " / %d bytes", packetSize, CONFIG_TCP_MSS);
      messageRate = 0;
      dataRate = 0;
      return;
    }

    // send
    switch (mode) {
      case Mycila::ESPConnect::Mode::AP:
        udp.broadcastTo(buffer, packetSize, MYCILA_UDP_PORT, tcpip_adapter_if_t::TCPIP_ADAPTER_IF_AP);
        break;
      case Mycila::ESPConnect::Mode::STA:
        udp.broadcastTo(buffer, packetSize, MYCILA_UDP_PORT, tcpip_adapter_if_t::TCPIP_ADAPTER_IF_STA);
        break;
      case Mycila::ESPConnect::Mode::ETH:
        udp.broadcastTo(buffer, packetSize, MYCILA_UDP_PORT, tcpip_adapter_if_t::TCPIP_ADAPTER_IF_ETH);
        break;
      default:
        break;
    }

    // update rate
    messageRateBuffer.add(static_cast<float>(esp_timer_get_time() / 1000000.0f));
    float diff = messageRateBuffer.diff();
    float count = messageRateBuffer.count();
    messageRate = diff == 0 ? 0 : count / diff;
    dataRateBuffer.add(packetSize);
    dataRate = diff == 0 ? 0 : dataRateBuffer.sum() / diff;

    delete[] buffer;
  });

  jsy.begin(MYCILA_JSY_SERIAL, MYCILA_JSY_RX, MYCILA_JSY_TX);

  if (jsy.isEnabled() && jsy.getBaudRate() != jsy.getMaxAvailableBaudRate())
    jsy.setBaudRate(jsy.getMaxAvailableBaudRate());

  jsyModel = jsy.getModel();

  if (jsyModel == MYCILA_JSY_MK_194 || jsyModel == MYCILA_JSY_MK_333 || jsyModel == MYCILA_JSY_MK_UNKNOWN) {
    dashboard.remove(jsy163Frequency);
    dashboard.remove(jsy163Voltage);
    dashboard.remove(jsy163current);
    dashboard.remove(jsy163PowerFactor);
    dashboard.remove(jsy163ActivePower);
    dashboard.remove(jsy163ApparentPower);
    dashboard.remove(jsy163ReactivePower);
    dashboard.remove(jsy163ActiveEnergy);
    dashboard.remove(jsy163ActiveEnergyImported);
    dashboard.remove(jsy163ActiveEnergyReturned);
    dashboard.remove(jsy163ActivePowerHistory);
  }

  if (jsyModel == MYCILA_JSY_MK_163 || jsyModel == MYCILA_JSY_MK_333 || jsyModel == MYCILA_JSY_MK_UNKNOWN) {
    dashboard.remove(jsy194Channel1Frequency);
    dashboard.remove(jsy194Channel1Voltage);
    dashboard.remove(jsy194Channel1Current);
    dashboard.remove(jsy194Channel1PowerFactor);
    dashboard.remove(jsy194Channel1ActivePower);
    dashboard.remove(jsy194Channel1ApparentPower);
    dashboard.remove(jsy194Channel1ReactivePower);
    dashboard.remove(jsy194Channel1ActiveEnergy);
    dashboard.remove(jsy194Channel1ActiveEnergyImported);
    dashboard.remove(jsy194Channel1ActiveEnergyReturned);
    dashboard.remove(jsy194Channel2Frequency);
    dashboard.remove(jsy194Channel2Voltage);
    dashboard.remove(jsy194Channel2Current);
    dashboard.remove(jsy194Channel2PowerFactor);
    dashboard.remove(jsy194Channel2ActivePower);
    dashboard.remove(jsy194Channel2ApparentPower);
    dashboard.remove(jsy194Channel2ReactivePower);
    dashboard.remove(jsy194Channel2ActiveEnergy);
    dashboard.remove(jsy194Channel2ActiveEnergyImported);
    dashboard.remove(jsy194Channel2ActiveEnergyReturned);
    dashboard.remove(jsy194Channel1ActivePowerHistory);
    dashboard.remove(jsy194Channel2ActivePowerHistory);
  }

  if (jsyModel == MYCILA_JSY_MK_163 || jsyModel == MYCILA_JSY_MK_194 || jsyModel == MYCILA_JSY_MK_UNKNOWN) {
    dashboard.remove(jsy333PhaseAFrequency);
    dashboard.remove(jsy333PhaseAVoltage);
    dashboard.remove(jsy333PhaseACurrent);
    dashboard.remove(jsy333PhaseAPowerFactor);
    dashboard.remove(jsy333PhaseAActivePower);
    dashboard.remove(jsy333PhaseAApparentPower);
    dashboard.remove(jsy333PhaseAReactivePower);
    dashboard.remove(jsy333PhaseAActiveEnergy);
    dashboard.remove(jsy333PhaseAActiveEnergyImported);
    dashboard.remove(jsy333PhaseAActiveEnergyReturned);
    dashboard.remove(jsy333PhaseAReactiveEnergy);
    dashboard.remove(jsy333PhaseAReactiveEnergyImported);
    dashboard.remove(jsy333PhaseAReactiveEnergyReturned);
    dashboard.remove(jsy333PhaseAApparentEnergy);
    dashboard.remove(jsy333PhaseBFrequency);
    dashboard.remove(jsy333PhaseBVoltage);
    dashboard.remove(jsy333PhaseBCurrent);
    dashboard.remove(jsy333PhaseBPowerFactor);
    dashboard.remove(jsy333PhaseBActivePower);
    dashboard.remove(jsy333PhaseBApparentPower);
    dashboard.remove(jsy333PhaseBReactivePower);
    dashboard.remove(jsy333PhaseBActiveEnergy);
    dashboard.remove(jsy333PhaseBActiveEnergyImported);
    dashboard.remove(jsy333PhaseBActiveEnergyReturned);
    dashboard.remove(jsy333PhaseBReactiveEnergy);
    dashboard.remove(jsy333PhaseBReactiveEnergyImported);
    dashboard.remove(jsy333PhaseBReactiveEnergyReturned);
    dashboard.remove(jsy333PhaseBApparentEnergy);
    dashboard.remove(jsy333PhaseCFrequency);
    dashboard.remove(jsy333PhaseCVoltage);
    dashboard.remove(jsy333PhaseCCurrent);
    dashboard.remove(jsy333PhaseCPowerFactor);
    dashboard.remove(jsy333PhaseCActivePower);
    dashboard.remove(jsy333PhaseCApparentPower);
    dashboard.remove(jsy333PhaseCReactivePower);
    dashboard.remove(jsy333PhaseCActiveEnergy);
    dashboard.remove(jsy333PhaseCActiveEnergyImported);
    dashboard.remove(jsy333PhaseCActiveEnergyReturned);
    dashboard.remove(jsy333PhaseCReactiveEnergy);
    dashboard.remove(jsy333PhaseCReactiveEnergyImported);
    dashboard.remove(jsy333PhaseCReactiveEnergyReturned);
    dashboard.remove(jsy333PhaseCApparentEnergy);
    dashboard.remove(jsy333PhaseAActivePowerHistory);
    dashboard.remove(jsy333PhaseBActivePowerHistory);
    dashboard.remove(jsy333PhaseCActivePowerHistory);
  }

  coreTaskManager.asyncStart(512 * 8, 5, 1, 100, false);
  jsyTaskManager.asyncStart(512 * 8, 5, 1, 100, false);

  ESP_LOGI(TAG, "Started " MYCILA_APP_NAME "!");
}

void loop() {
  vTaskDelete(NULL);
}
