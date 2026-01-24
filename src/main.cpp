// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) Mathieu Carbou
 */
#include <Arduino.h>

#include <AsyncUDP.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <Preferences.h>

#include <ArduinoJson.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <ESPDash.h>
#include <ElegantOTA.h>
#include <FastCRC32.h>
#include <WebSerial.h>

#include <MycilaAppInfo.h>
#include <MycilaCircularBuffer.h>
#include <MycilaESPConnect.h>
#include <MycilaJSY.h>
#include <MycilaSystem.h>
#include <MycilaTaskManager.h>
#include <MycilaTime.h>

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

#define MYCILA_ADMIN_PASSWORD ""
#define MYCILA_ADMIN_USERNAME "admin"
#define MYCILA_APP_NAME       "MycilaJSY App"
#define MYCILA_GRAPH_POINTS   60
// #define MYCILA_UDP_MSG_TYPE_JSY_DATA 0x02 // supports all JSY models
#define MYCILA_UDP_MSG_TYPE_JSY_DATA 0x03 // new format with message ID
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
static Mycila::JSY::Data savedJSYData;
static Mycila::JSY::Metrics savedJSYDataChannels[2] = {0};

static Mycila::TaskManager coreTaskManager("core");
static Mycila::TaskManager jsyTaskManager("jsy");

static dash::StatisticValue versionJSYApp(dashboard, "MycilaJSYApp Version");
static dash::StatisticValue versionJSY(dashboard, "MycilaJSY Version");
static dash::StatisticValue networkHostname(dashboard, "Network Hostname");
static dash::StatisticValue networkInterface(dashboard, "Network Interface");
static dash::StatisticValue networkAPIP(dashboard, "Network Access Point IP Address");
static dash::StatisticValue networkAPMAC(dashboard, "Network Access Point MAC Address");
#ifdef ESPCONNECT_ETH_SUPPORT
static dash::StatisticValue networkEthIP(dashboard, "Network Ethernet IP Address");
static dash::StatisticValue networkEthIPv6Local(dashboard, "Network Ethernet IPv6 Link-Local Address");
static dash::StatisticValue networkEthIPv6Global(dashboard, "Network Ethernet IPv6 Global Address");
static dash::StatisticValue networkEthMAC(dashboard, "Network Ethernet MAC Address");
#endif
static dash::StatisticValue networkWiFiIP(dashboard, "Network WiFi IP Address");
static dash::StatisticValue networkWiFiIPv6Local(dashboard, "Network WiFi IPv6 Link-Local Address");
static dash::StatisticValue networkWiFiIPv6Global(dashboard, "Network WiFi IPv6 Global Address");
static dash::StatisticValue networkWiFiMAC(dashboard, "Network WiFi MAC Address");
static dash::StatisticValue networkWiFiBSSID(dashboard, "Network WiFi BSSID");
static dash::StatisticValue networkWiFiSSID(dashboard, "Network WiFi SSID");
static dash::StatisticValue networkWiFiRSSI(dashboard, "Network WiFi RSSI");
static dash::StatisticValue networkWiFiSignal(dashboard, "Network WiFi Signal");
static dash::StatisticValue<float, 2> messageRateCard(dashboard, "UDP Message Rate (msg/s)");
static dash::StatisticValue<uint32_t> dataRateCard(dashboard, "UDP Data Rate (bytes/s)");
static dash::StatisticValue uptime(dashboard, "Uptime");

static dash::SeparatorCard sep1(dashboard, "Controls");
static dash::GenericCard jsyModelCard(dashboard, "Model");
static dash::ToggleButtonCard restart(dashboard, "Restart");
static dash::ToggleButtonCard energyReset(dashboard, "Reset Energy");
static dash::ToggleButtonCard reset(dashboard, "Factory Reset");

// JSY-MK-163
static dash::SeparatorCard jsy163Data(dashboard, "Data");
static dash::ToggleButtonCard jsy163Publish(dashboard, "UDP Publish");
static dash::GenericCard<float, 1> jsy163Frequency(dashboard, "Frequency", "Hz");
static dash::GenericCard<float, 1> jsy163Voltage(dashboard, "Voltage", "V");
static dash::GenericCard<float, 2> jsy163current(dashboard, "Current", "A");
static dash::GenericCard<float, 2> jsy163PowerFactor(dashboard, "Power Factor");
static dash::GenericCard<float, 0> jsy163ActivePower(dashboard, "Active Power", "W");
static dash::GenericCard<float, 0> jsy163ApparentPower(dashboard, "Apparent Power", "VA");
static dash::GenericCard<float, 0> jsy163ReactivePower(dashboard, "Reactive Power", "VAr");
static dash::GenericCard<uint32_t> jsy163ActiveEnergy(dashboard, "Active Energy", "Wh");
static dash::GenericCard<uint32_t> jsy163ActiveEnergyImported(dashboard, "Active E. Imported", "Wh");
static dash::GenericCard<uint32_t> jsy163ActiveEnergyReturned(dashboard, "Active E. Returned", "Wh");
static dash::BarChart<int8_t, int16_t> jsy163ActivePowerHistory(dashboard, "Active Power (W)");

// JSY-MK-193, JSY-MK-194
static dash::SeparatorCard jsy194ChannelConfig(dashboard, "Channel Configuration");
static dash::ToggleButtonCard jsy194ChannelSwap(dashboard, "Swap JSY Channels");
static dash::ToggleButtonCard jsy194ShellySwap(dashboard, "Swap Shelly Meters");
static dash::SeparatorCard jsy194Data1(dashboard, "Channel 1");
static dash::ToggleButtonCard jsy194Publish1(dashboard, "UDP Publish");
static dash::GenericCard<float, 1> jsy194Channel1Frequency(dashboard, "Frequency", "Hz");
static dash::GenericCard<float, 1> jsy194Channel1Voltage(dashboard, "Voltage", "V");
static dash::GenericCard<float, 2> jsy194Channel1Current(dashboard, "Current", "A");
static dash::GenericCard<float, 2> jsy194Channel1PowerFactor(dashboard, "Power Factor");
static dash::GenericCard<float, 0> jsy194Channel1ActivePower(dashboard, "Active Power", "W");
static dash::GenericCard<float, 0> jsy194Channel1ApparentPower(dashboard, "Apparent Power", "VA");
static dash::GenericCard<float, 0> jsy194Channel1ReactivePower(dashboard, "Reactive Power", "VAr");
static dash::GenericCard<uint32_t> jsy194Channel1ActiveEnergy(dashboard, "Active Energy", "Wh");
static dash::GenericCard<uint32_t> jsy194Channel1ActiveEnergyImported(dashboard, "Active E. Imported", "Wh");
static dash::GenericCard<uint32_t> jsy194Channel1ActiveEnergyReturned(dashboard, "Active E. Returned", "Wh");
static dash::SeparatorCard jsy194Data2(dashboard, "Channel 2");
static dash::ToggleButtonCard jsy194Publish2(dashboard, "UDP Publish");
static dash::GenericCard<float, 1> jsy194Channel2Frequency(dashboard, "Frequency", "Hz");
static dash::GenericCard<float, 1> jsy194Channel2Voltage(dashboard, "Voltage", "V");
static dash::GenericCard<float, 2> jsy194Channel2Current(dashboard, "Current", "A");
static dash::GenericCard<float, 2> jsy194Channel2PowerFactor(dashboard, "Power Factor");
static dash::GenericCard<float, 0> jsy194Channel2ActivePower(dashboard, "Active Power", "W");
static dash::GenericCard<float, 0> jsy194Channel2ApparentPower(dashboard, "Apparent Power", "VA");
static dash::GenericCard<float, 0> jsy194Channel2ReactivePower(dashboard, "Reactive Power", "VAr");
static dash::GenericCard<uint32_t> jsy194Channel2ActiveEnergy(dashboard, "Active Energy", "Wh");
static dash::GenericCard<uint32_t> jsy194Channel2ActiveEnergyImported(dashboard, "Active E. Imported", "Wh");
static dash::GenericCard<uint32_t> jsy194Channel2ActiveEnergyReturned(dashboard, "Active E. Returned", "Wh");
static dash::SeparatorCard jsy194Charts(dashboard, "History Charts");
static dash::BarChart<int8_t, int16_t> jsy194Channel1ActivePowerHistory(dashboard, "Active Power Channel 1 (W)");
static dash::BarChart<int8_t, int16_t> jsy194Channel2ActivePowerHistory(dashboard, "Active Power Channel 2 (W)");

// JSY-MK-333
static dash::SeparatorCard jsy333DataA(dashboard, "Phase A");
static dash::ToggleButtonCard jsy333PublishA(dashboard, "UDP Publish");
static dash::GenericCard<float, 1> jsy333PhaseAFrequency(dashboard, "Frequency", "Hz");
static dash::GenericCard<float, 1> jsy333PhaseAVoltage(dashboard, "Voltage", "V");
static dash::GenericCard<float, 2> jsy333PhaseACurrent(dashboard, "Current", "A");
static dash::GenericCard<float, 2> jsy333PhaseAPowerFactor(dashboard, "Power Factor");
static dash::GenericCard<float, 0> jsy333PhaseAActivePower(dashboard, "Active Power", "W");
static dash::GenericCard<float, 0> jsy333PhaseAApparentPower(dashboard, "Apparent Power", "VA");
static dash::GenericCard<float, 0> jsy333PhaseAReactivePower(dashboard, "Reactive Power", "VAr");
static dash::GenericCard<uint32_t> jsy333PhaseAActiveEnergy(dashboard, "Active Energy", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseAActiveEnergyImported(dashboard, "Active Energy Import", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseAActiveEnergyReturned(dashboard, "Active Energy Return", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseAReactiveEnergy(dashboard, "Reactive Energy", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseAReactiveEnergyImported(dashboard, "Reactive Energy Import", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseAReactiveEnergyReturned(dashboard, "Reactive Energy Return", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseAApparentEnergy(dashboard, "Apparent Energy", "VAh");
static dash::SeparatorCard jsy333DataB(dashboard, "Phase B");
static dash::ToggleButtonCard jsy333PublishB(dashboard, "UDP Publish");
static dash::GenericCard<float, 1> jsy333PhaseBFrequency(dashboard, "Frequency", "Hz");
static dash::GenericCard<float, 1> jsy333PhaseBVoltage(dashboard, "Voltage", "V");
static dash::GenericCard<float, 2> jsy333PhaseBCurrent(dashboard, "Current", "A");
static dash::GenericCard<float, 2> jsy333PhaseBPowerFactor(dashboard, "Power Factor");
static dash::GenericCard<float, 0> jsy333PhaseBActivePower(dashboard, "Active Power", "W");
static dash::GenericCard<float, 0> jsy333PhaseBApparentPower(dashboard, "Apparent Power", "VA");
static dash::GenericCard<float, 0> jsy333PhaseBReactivePower(dashboard, "Reactive Power", "VAr");
static dash::GenericCard<uint32_t> jsy333PhaseBActiveEnergy(dashboard, "Active Energy", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseBActiveEnergyImported(dashboard, "Active E. Imported", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseBActiveEnergyReturned(dashboard, "Active E. Returned", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseBReactiveEnergy(dashboard, "Reactive Energy", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseBReactiveEnergyImported(dashboard, "Reactive E. Imported", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseBReactiveEnergyReturned(dashboard, "Reactive E. Returned", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseBApparentEnergy(dashboard, "Apparent Energy", "VAh");
static dash::SeparatorCard jsy333DataC(dashboard, "Phase C");
static dash::ToggleButtonCard jsy333PublishC(dashboard, "UDP Publish");
static dash::GenericCard<float, 1> jsy333PhaseCFrequency(dashboard, "Frequency", "Hz");
static dash::GenericCard<float, 1> jsy333PhaseCVoltage(dashboard, "Voltage", "V");
static dash::GenericCard<float, 2> jsy333PhaseCCurrent(dashboard, "Current", "A");
static dash::GenericCard<float, 2> jsy333PhaseCPowerFactor(dashboard, "Power Factor");
static dash::GenericCard<float, 0> jsy333PhaseCActivePower(dashboard, "Active Power", "W");
static dash::GenericCard<float, 0> jsy333PhaseCApparentPower(dashboard, "Apparent Power", "VA");
static dash::GenericCard<float, 0> jsy333PhaseCReactivePower(dashboard, "Reactive Power", "VAr");
static dash::GenericCard<uint32_t> jsy333PhaseCActiveEnergy(dashboard, "Active Energy", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseCActiveEnergyImported(dashboard, "Active E. Imported", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseCActiveEnergyReturned(dashboard, "Active E. Returned", "Wh");
static dash::GenericCard<uint32_t> jsy333PhaseCReactiveEnergy(dashboard, "Reactive Energy", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseCReactiveEnergyImported(dashboard, "Reactive E. Imported", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseCReactiveEnergyReturned(dashboard, "Reactive E. Returned", "VArh");
static dash::GenericCard<uint32_t> jsy333PhaseCApparentEnergy(dashboard, "Apparent Energy", "VAh");
static dash::SeparatorCard jsy333Charts(dashboard, "History Charts");
static dash::BarChart<int8_t, int16_t> jsy333PhaseAActivePowerHistory(dashboard, "Active Power Phase A (W)");
static dash::BarChart<int8_t, int16_t> jsy333PhaseBActivePowerHistory(dashboard, "Active Power Phase B (W)");
static dash::BarChart<int8_t, int16_t> jsy333PhaseCActivePowerHistory(dashboard, "Active Power Phase C (W)");

// graphs
static int8_t historyX[MYCILA_GRAPH_POINTS] = {0};
static int16_t power1HistoryY[MYCILA_GRAPH_POINTS] = {0};
static int16_t power2HistoryY[MYCILA_GRAPH_POINTS] = {0};
static int16_t power3HistoryY[MYCILA_GRAPH_POINTS] = {0};

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
  jsy.end();
});

static Mycila::Task restartTask("Restart", Mycila::Task::Type::ONCE, [](void* params) {
  ESP_LOGW(TAG, "Restarting " MYCILA_APP_NAME "...");
  Mycila::System::restart(500);
});

static Mycila::Task dashboardTask("Dashboard", [](void* params) {
  Mycila::ESPConnect::Mode mode = espConnect.getMode();
  if (mode == Mycila::ESPConnect::Mode::AP) {
    networkInterface.setValue("Access Point");
    networkAPIP.setValue(espConnect.getIPAddress(Mycila::ESPConnect::Mode::AP).toString().c_str());
    networkAPMAC.setValue(espConnect.getMACAddress(Mycila::ESPConnect::Mode::AP));
  } else {
    // Mode
    switch (mode) {
      case Mycila::ESPConnect::Mode::ETH: {
        networkInterface.setValue("Ethernet");
        break;
      }
      case Mycila::ESPConnect::Mode::STA: {
        networkInterface.setValue("WiFi");
        break;
      }
      default:
        networkInterface.setValue("Unknown");
        break;
    }
    // WiFi
    {
      networkWiFiIP.setValue(espConnect.getIPAddress(Mycila::ESPConnect::Mode::STA).toString().c_str());
      auto ipv6Local = espConnect.getLinkLocalIPv6Address(Mycila::ESPConnect::Mode::STA);
      networkWiFiIPv6Local.setValue(ipv6Local == IN6ADDR_ANY ? "N/A" : ipv6Local.toString().c_str());
      auto ipv6Global = espConnect.getGlobalIPv6Address(Mycila::ESPConnect::Mode::STA);
      networkWiFiIPv6Global.setValue(ipv6Global == IN6ADDR_ANY ? "N/A" : ipv6Global.toString().c_str());
      auto mac = espConnect.getMACAddress(Mycila::ESPConnect::Mode::STA);
      networkWiFiMAC.setValue(mac.empty() ? std::string("N/A") : mac);
      networkWiFiSSID.setValue(espConnect.getWiFiSSID());
      networkWiFiBSSID.setValue(espConnect.getWiFiBSSID());
      networkWiFiRSSI.setValue((std::to_string(espConnect.getWiFiRSSI()) + " dBm"));
      networkWiFiSignal.setValue((std::to_string(espConnect.getWiFiSignalQuality()) + " %"));
    }
#ifdef ESPCONNECT_ETH_SUPPORT
    // Ethernet
    {
      networkEthIP.setValue(espConnect.getIPAddress(Mycila::ESPConnect::Mode::ETH).toString().c_str());
      auto ipv6Local = espConnect.getLinkLocalIPv6Address(Mycila::ESPConnect::Mode::ETH);
      networkEthIPv6Local.setValue(ipv6Local == IN6ADDR_ANY ? "N/A" : ipv6Local.toString().c_str());
      auto ipv6Global = espConnect.getGlobalIPv6Address(Mycila::ESPConnect::Mode::ETH);
      networkEthIPv6Global.setValue(ipv6Global == IN6ADDR_ANY ? "N/A" : ipv6Global.toString().c_str());
      auto mac = espConnect.getMACAddress(Mycila::ESPConnect::Mode::ETH);
      networkEthMAC.setValue(mac.empty() ? std::string("N/A") : mac);
    }
#endif
  }

  uptime.setValue(Mycila::Time::toDHHMMSS(Mycila::System::getUptime()));
  messageRateCard.setValue(messageRate);
  dataRateCard.setValue(dataRate);

  switch (jsyModel) {
    case MYCILA_JSY_MK_1031:
    case MYCILA_JSY_MK_163: {
      jsy163Frequency.setValue(savedJSYData.single().frequency);
      jsy163Voltage.setValue(savedJSYData.single().voltage);
      jsy163current.setValue(savedJSYData.single().current);
      jsy163PowerFactor.setValue(savedJSYData.single().powerFactor);
      jsy163ActivePower.setValue(savedJSYData.single().activePower);
      jsy163ApparentPower.setValue(savedJSYData.single().apparentPower);
      jsy163ReactivePower.setValue(savedJSYData.single().reactivePower);
      jsy163ActiveEnergy.setValue(savedJSYData.single().activeEnergy);
      jsy163ActiveEnergyImported.setValue(savedJSYData.single().activeEnergyImported);
      jsy163ActiveEnergyReturned.setValue(savedJSYData.single().activeEnergyReturned);

      // shift array
      for (size_t i = 0; i < MYCILA_GRAPH_POINTS - 1; i++) {
        power1HistoryY[i] = power1HistoryY[i + 1];
      }

      // set new value
      power1HistoryY[MYCILA_GRAPH_POINTS - 1] = round(savedJSYData.phaseA().activePower);

      // update charts
      jsy163ActivePowerHistory.setY(power1HistoryY, MYCILA_GRAPH_POINTS);

      break;
    }
    case MYCILA_JSY_MK_193:
    case MYCILA_JSY_MK_194: {
      jsy194Channel1Frequency.setValue(savedJSYDataChannels[0].frequency);
      jsy194Channel1Voltage.setValue(savedJSYDataChannels[0].voltage);
      jsy194Channel1Current.setValue(savedJSYDataChannels[0].current);
      jsy194Channel1PowerFactor.setValue(savedJSYDataChannels[0].powerFactor);
      jsy194Channel1ActivePower.setValue(savedJSYDataChannels[0].activePower);
      jsy194Channel1ApparentPower.setValue(savedJSYDataChannels[0].apparentPower);
      jsy194Channel1ReactivePower.setValue(savedJSYDataChannels[0].reactivePower);
      jsy194Channel1ActiveEnergy.setValue(savedJSYDataChannels[0].activeEnergy);
      jsy194Channel1ActiveEnergyImported.setValue(savedJSYDataChannels[0].activeEnergyImported);
      jsy194Channel1ActiveEnergyReturned.setValue(savedJSYDataChannels[0].activeEnergyReturned);

      jsy194Channel2Frequency.setValue(savedJSYDataChannels[1].frequency);
      jsy194Channel2Voltage.setValue(savedJSYDataChannels[1].voltage);
      jsy194Channel2Current.setValue(savedJSYDataChannels[1].current);
      jsy194Channel2PowerFactor.setValue(savedJSYDataChannels[1].powerFactor);
      jsy194Channel2ActivePower.setValue(savedJSYDataChannels[1].activePower);
      jsy194Channel2ApparentPower.setValue(savedJSYDataChannels[1].apparentPower);
      jsy194Channel2ReactivePower.setValue(savedJSYDataChannels[1].reactivePower);
      jsy194Channel2ActiveEnergy.setValue(savedJSYDataChannels[1].activeEnergy);
      jsy194Channel2ActiveEnergyImported.setValue(savedJSYDataChannels[1].activeEnergyImported);
      jsy194Channel2ActiveEnergyReturned.setValue(savedJSYDataChannels[1].activeEnergyReturned);

      // shift array
      for (size_t i = 0; i < MYCILA_GRAPH_POINTS - 1; i++) {
        power1HistoryY[i] = power1HistoryY[i + 1];
        power2HistoryY[i] = power2HistoryY[i + 1];
      }

      // set new value
      power1HistoryY[MYCILA_GRAPH_POINTS - 1] = round(savedJSYDataChannels[0].activePower);
      power2HistoryY[MYCILA_GRAPH_POINTS - 1] = round(savedJSYDataChannels[1].activePower);

      // update charts
      jsy194Channel1ActivePowerHistory.setY(power1HistoryY, MYCILA_GRAPH_POINTS);
      jsy194Channel2ActivePowerHistory.setY(power2HistoryY, MYCILA_GRAPH_POINTS);

      break;
    }
    case MYCILA_JSY_MK_333: {
      jsy333PhaseAFrequency.setValue(savedJSYData.phaseA().frequency);
      jsy333PhaseAVoltage.setValue(savedJSYData.phaseA().voltage);
      jsy333PhaseACurrent.setValue(savedJSYData.phaseA().current);
      jsy333PhaseAPowerFactor.setValue(savedJSYData.phaseA().powerFactor);
      jsy333PhaseAActivePower.setValue(savedJSYData.phaseA().activePower);
      jsy333PhaseAApparentPower.setValue(savedJSYData.phaseA().apparentPower);
      jsy333PhaseAReactivePower.setValue(savedJSYData.phaseA().reactivePower);
      jsy333PhaseAActiveEnergy.setValue(savedJSYData.phaseA().activeEnergy);
      jsy333PhaseAActiveEnergyImported.setValue(savedJSYData.phaseA().activeEnergyImported);
      jsy333PhaseAActiveEnergyReturned.setValue(savedJSYData.phaseA().activeEnergyReturned);
      jsy333PhaseAReactiveEnergy.setValue(savedJSYData.phaseA().reactiveEnergy);
      jsy333PhaseAReactiveEnergyImported.setValue(savedJSYData.phaseA().reactiveEnergyImported);
      jsy333PhaseAReactiveEnergyReturned.setValue(savedJSYData.phaseA().reactiveEnergyReturned);
      jsy333PhaseAApparentEnergy.setValue(savedJSYData.phaseA().apparentEnergy);

      jsy333PhaseBFrequency.setValue(savedJSYData.phaseB().frequency);
      jsy333PhaseBVoltage.setValue(savedJSYData.phaseB().voltage);
      jsy333PhaseBCurrent.setValue(savedJSYData.phaseB().current);
      jsy333PhaseBPowerFactor.setValue(savedJSYData.phaseB().powerFactor);
      jsy333PhaseBActivePower.setValue(savedJSYData.phaseB().activePower);
      jsy333PhaseBApparentPower.setValue(savedJSYData.phaseB().apparentPower);
      jsy333PhaseBReactivePower.setValue(savedJSYData.phaseB().reactivePower);
      jsy333PhaseBActiveEnergy.setValue(savedJSYData.phaseB().activeEnergy);
      jsy333PhaseBActiveEnergyImported.setValue(savedJSYData.phaseB().activeEnergyImported);
      jsy333PhaseBActiveEnergyReturned.setValue(savedJSYData.phaseB().activeEnergyReturned);
      jsy333PhaseBReactiveEnergy.setValue(savedJSYData.phaseB().reactiveEnergy);
      jsy333PhaseBReactiveEnergyImported.setValue(savedJSYData.phaseB().reactiveEnergyImported);
      jsy333PhaseBReactiveEnergyReturned.setValue(savedJSYData.phaseB().reactiveEnergyReturned);
      jsy333PhaseBApparentEnergy.setValue(savedJSYData.phaseB().apparentEnergy);

      jsy333PhaseCFrequency.setValue(savedJSYData.phaseC().frequency);
      jsy333PhaseCVoltage.setValue(savedJSYData.phaseC().voltage);
      jsy333PhaseCCurrent.setValue(savedJSYData.phaseC().current);
      jsy333PhaseCPowerFactor.setValue(savedJSYData.phaseC().powerFactor);
      jsy333PhaseCActivePower.setValue(savedJSYData.phaseC().activePower);
      jsy333PhaseCApparentPower.setValue(savedJSYData.phaseC().apparentPower);
      jsy333PhaseCReactivePower.setValue(savedJSYData.phaseC().reactivePower);
      jsy333PhaseCActiveEnergy.setValue(savedJSYData.phaseC().activeEnergy);
      jsy333PhaseCActiveEnergyImported.setValue(savedJSYData.phaseC().activeEnergyImported);
      jsy333PhaseCActiveEnergyReturned.setValue(savedJSYData.phaseC().activeEnergyReturned);
      jsy333PhaseCReactiveEnergy.setValue(savedJSYData.phaseC().reactiveEnergy);
      jsy333PhaseCReactiveEnergyImported.setValue(savedJSYData.phaseC().reactiveEnergyImported);
      jsy333PhaseCReactiveEnergyReturned.setValue(savedJSYData.phaseC().reactiveEnergyReturned);
      jsy333PhaseCApparentEnergy.setValue(savedJSYData.phaseC().apparentEnergy);

      // shift array
      for (size_t i = 0; i < MYCILA_GRAPH_POINTS - 1; i++) {
        power1HistoryY[i] = power1HistoryY[i + 1];
        power2HistoryY[i] = power2HistoryY[i + 1];
        power3HistoryY[i] = power3HistoryY[i + 1];
      }

      // set new value
      power1HistoryY[MYCILA_GRAPH_POINTS - 1] = round(savedJSYData.phaseA().activePower);
      power2HistoryY[MYCILA_GRAPH_POINTS - 1] = round(savedJSYData.phaseB().activePower);
      power3HistoryY[MYCILA_GRAPH_POINTS - 1] = round(savedJSYData.phaseC().activePower);

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
  if (savedJSYData.model == MYCILA_JSY_MK_163 || savedJSYData.model == MYCILA_JSY_MK_1031) {
    return id == 0 ? 0 : -1;
  }
  // JSY-MK-333
  if (savedJSYData.model == MYCILA_JSY_MK_333) {
    return id < 0 || id > 2 ? -1 : id;
  }
  // JSY-MK-194
  if (id == 0 || id == 1) {
    return id;
  }
  // invalid id
  return -1;
}

static int get_jsy_channel_for_shelly_id(int shellyID) {
  return jsy194ShellySwap.value() ? (shellyID == 0 ? 1 : 0) : shellyID;
}

// Shelly.GetDeviceInfo

static void ShellyGetDeviceInfo(const JsonObject& root) {
  root["name"] = "Mycila JSY App";
  root["id"] = (savedJSYData.model == MYCILA_JSY_MK_333 ? "shellypro3em-" : "shellyproem50-") + espConnect.getMACAddress();
  root["mac"] = espConnect.getMACAddress();
  root["ver"] = MYCILA_JSY_VERSION;
  root["app"] = savedJSYData.model == MYCILA_JSY_MK_333 ? "3EM" : "EM";
}

// EM1.GetStatus

static void EM1GetStatus(int id, const JsonObject& root) {
  root["id"] = id;

  if (id == 0 && (savedJSYData.model == MYCILA_JSY_MK_163 || savedJSYData.model == MYCILA_JSY_MK_1031)) {
    root["voltage"] = savedJSYData.single().voltage;
    root["current"] = savedJSYData.single().current;
    root["act_power"] = savedJSYData.single().activePower;
    root["aprt_power"] = savedJSYData.single().apparentPower;
    root["pf"] = savedJSYData.single().powerFactor;
    root["freq"] = savedJSYData.single().frequency;
    // old shelly's
    root["power"] = savedJSYData.single().activePower;

  } else if ((id == 0 || id == 1 || id == 2) && savedJSYData.model == MYCILA_JSY_MK_333) {
    root["voltage"] = savedJSYData.phase(id).voltage;
    root["current"] = savedJSYData.phase(id).current;
    root["act_power"] = savedJSYData.phase(id).activePower;
    root["aprt_power"] = savedJSYData.phase(id).apparentPower;
    root["pf"] = savedJSYData.phase(id).powerFactor;
    root["freq"] = savedJSYData.phase(id).frequency;
    // old shelly's
    root["power"] = savedJSYData.phase(id).activePower;

  } else if ((id == 0 || id == 1) && (savedJSYData.model == MYCILA_JSY_MK_193 || savedJSYData.model == MYCILA_JSY_MK_194)) {
    int channel = get_jsy_channel_for_shelly_id(id);
    root["voltage"] = savedJSYDataChannels[channel].voltage;
    root["current"] = savedJSYDataChannels[channel].current;
    root["act_power"] = savedJSYDataChannels[channel].activePower;
    root["aprt_power"] = savedJSYDataChannels[channel].apparentPower;
    root["pf"] = savedJSYDataChannels[channel].powerFactor;
    root["freq"] = savedJSYDataChannels[channel].frequency;
    // old shelly's
    root["power"] = savedJSYDataChannels[channel].activePower;
  }

  root["calibration"] = "factory";
}

// EM1Data.GetStatus

static void EM1DataGetStatus(int id, const JsonObject& root) {
  root["id"] = id;

  if (id == 0 && (savedJSYData.model == MYCILA_JSY_MK_163 || savedJSYData.model == MYCILA_JSY_MK_1031)) {
    root["total_act_energy"] = savedJSYData.single().activeEnergy;
    root["total_act_ret_energy"] = savedJSYData.single().activeEnergyReturned;
    // old shelly's
    root["total"] = savedJSYData.single().activeEnergy;
    root["total_returned"] = savedJSYData.single().activeEnergyReturned;

  } else if ((id == 0 || id == 1 || id == 2) && savedJSYData.model == MYCILA_JSY_MK_333) {
    root["total_act_energy"] = savedJSYData.phase(id).activeEnergy;
    root["total_act_ret_energy"] = savedJSYData.phase(id).activeEnergyReturned;
    // old shelly's
    root["total"] = savedJSYData.phase(id).activeEnergy;
    root["total_returned"] = savedJSYData.phase(id).activeEnergyReturned;

  } else if ((id == 0 || id == 1) && (savedJSYData.model == MYCILA_JSY_MK_193 || savedJSYData.model == MYCILA_JSY_MK_194)) {
    int channel = get_jsy_channel_for_shelly_id(id);
    root["total_act_energy"] = savedJSYDataChannels[channel].activeEnergy;
    root["total_act_ret_energy"] = savedJSYDataChannels[channel].activeEnergyReturned;
    // old shelly's
    root["total"] = savedJSYDataChannels[channel].activeEnergy;
    root["total_returned"] = savedJSYDataChannels[channel].activeEnergyReturned;
  }
}

// EM.GetStatus

static void EMGetStatus(int id, const JsonObject& root) {
  root["id"] = id;
  root["a_current"] = savedJSYData.phaseA().current;
  root["a_voltage"] = savedJSYData.phaseA().voltage;
  root["a_act_power"] = savedJSYData.phaseA().activePower;
  root["a_aprt_power"] = savedJSYData.phaseA().apparentPower;
  root["a_pf"] = savedJSYData.phaseA().powerFactor;
  root["a_freq"] = savedJSYData.phaseA().frequency;
  root["b_current"] = savedJSYData.phaseB().current;
  root["b_voltage"] = savedJSYData.phaseB().voltage;
  root["b_act_power"] = savedJSYData.phaseB().activePower;
  root["b_aprt_power"] = savedJSYData.phaseB().apparentPower;
  root["b_pf"] = savedJSYData.phaseB().powerFactor;
  root["b_freq"] = savedJSYData.phaseB().frequency;
  root["c_current"] = savedJSYData.phaseC().current;
  root["c_voltage"] = savedJSYData.phaseC().voltage;
  root["c_active_power"] = savedJSYData.phaseC().activePower;
  root["c_aprt_power"] = savedJSYData.phaseC().apparentPower;
  root["c_pf"] = savedJSYData.phaseC().powerFactor;
  root["c_freq"] = savedJSYData.phaseC().frequency;
  root["n_current"] = 0;
  root["total_current"] = savedJSYData.aggregate.current;
  root["total_act_power"] = savedJSYData.aggregate.activePower;
  root["total_aprt_power"] = savedJSYData.aggregate.apparentPower;
  root["user_calibrated_phase"] = JsonArray();
  root["errors"] = JsonArray();
}

// EMData.GetStatus

static void EMDataGetStatus(int id, const JsonObject& root) {
  root["id"] = id;
  root["a_total_act_energy"] = savedJSYData.phaseA().activeEnergy;
  root["a_total_act_ret_energy"] = savedJSYData.phaseA().activeEnergyReturned;
  root["b_total_act_energy"] = savedJSYData.phaseB().activeEnergy;
  root["b_total_act_ret_energy"] = savedJSYData.phaseB().activeEnergyReturned;
  root["c_total_act_energy"] = savedJSYData.phaseC().activeEnergy;
  root["c_total_act_ret_energy"] = savedJSYData.phaseC().activeEnergyReturned;
  root["total_act"] = savedJSYData.aggregate.activeEnergy;
  root["total_act_ret"] = savedJSYData.aggregate.activeEnergyReturned;
}

static size_t sendUDP(const JsonObject& json) {
  // buffer[0] == MYCILA_UDP_MSG_TYPE_JSY_DATA (1)
  // buffer[1] == message ID (4) - uint32_t
  // buffer[5] == jsonSize (4) - size_t
  // buffer[9] == MsgPack (?)
  // buffer[9 + size] == CRC32 (4)

  const size_t jsonSize = measureMsgPack(json);
  const uint32_t messageID = micros();

  // messageSize: total size of the UDP message, which can be split across multiple UDP packets. If this is the case, next packets will have size of 0
  const size_t messageSize = jsonSize + 13;

  uint8_t* buffer = new uint8_t[messageSize];
  buffer[0] = MYCILA_UDP_MSG_TYPE_JSY_DATA;
  memcpy(buffer + 1, &messageID, 4);
  memcpy(buffer + 5, &jsonSize, 4);
  serializeMsgPack(json, buffer + 9, jsonSize);

  // crc32
  FastCRC32 crc32;
  crc32.add(buffer, messageSize - 4);
  uint32_t crc = crc32.calc();
  memcpy(buffer + messageSize - 4, &crc, 4);

  // send
  size_t totalSent = 0;
  while (totalSent < messageSize) {
    size_t sent = 0;
    switch (espConnect.getMode()) {
      case Mycila::ESPConnect::Mode::AP: {
        sent = udp.broadcastTo(buffer + totalSent, messageSize - totalSent, MYCILA_UDP_PORT, tcpip_adapter_if_t::TCPIP_ADAPTER_IF_AP);
        break;
      }
      case Mycila::ESPConnect::Mode::STA: {
        sent = udp.broadcastTo(buffer + totalSent, messageSize - totalSent, MYCILA_UDP_PORT, tcpip_adapter_if_t::TCPIP_ADAPTER_IF_STA);
        break;
      }
      case Mycila::ESPConnect::Mode::ETH: {
        sent = udp.broadcastTo(buffer + totalSent, messageSize - totalSent, MYCILA_UDP_PORT, tcpip_adapter_if_t::TCPIP_ADAPTER_IF_ETH);
        break;
      }
      default:
        sent = 0;
        break;
    }
    if (sent == 0) {
      break;
    }
    totalSent += sent;
  }

  if (totalSent == 0) {
    ESP_LOGW(TAG, "UDP send failed");
  } else if (totalSent < messageSize) {
    ESP_LOGW(TAG, "UDP send incomplete: sent %u of %u bytes", totalSent, messageSize);
  }

  delete[] buffer;

  return totalSent;
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
  jsy163Publish.setValue(preferences.getBool("jsy163_udp", true));
  jsy194Publish1.setValue(preferences.getBool("jsy194_ch1_udp", true));
  jsy194Publish2.setValue(preferences.getBool("jsy194_ch2_udp", true));
  jsy333PublishA.setValue(preferences.getBool("jsy333_a_udp", true));
  jsy333PublishB.setValue(preferences.getBool("jsy333_b_udp", true));
  jsy333PublishC.setValue(preferences.getBool("jsy333_c_udp", true));
  jsy194ChannelSwap.setValue(preferences.getBool("jsy194_ch_swap", false));
  jsy194ShellySwap.setValue(preferences.getBool("jsy194_sh_swap", false));

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
      bool enabled = enableParam->value() == "on";
      preferences.putBool("jsy163_udp", enabled);
      preferences.putBool("jsy194_ch1_udp", enabled);
      preferences.putBool("jsy194_ch2_udp", enabled);
      preferences.putBool("jsy333_a_udp", enabled);
      preferences.putBool("jsy333_b_udp", enabled);
      preferences.putBool("jsy333_c_udp", enabled);
      jsy163Publish.setValue(enabled);
      jsy194Publish1.setValue(enabled);
      jsy194Publish2.setValue(enabled);
      jsy333PublishA.setValue(enabled);
      jsy333PublishB.setValue(enabled);
      jsy333PublishC.setValue(enabled);
    }
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    switch (savedJSYData.model) {
      case MYCILA_JSY_MK_1031:
      case MYCILA_JSY_MK_163:
        root["switch"] = jsy163Publish.value() ? "on" : "off";
        break;
      case MYCILA_JSY_MK_193:
      case MYCILA_JSY_MK_194:
        root["switch"] = jsy194Publish1.value() || jsy194Publish2.value() ? "on" : "off";
        break;
      case MYCILA_JSY_MK_333:
        root["switch"] = jsy333PublishA.value() || jsy333PublishB.value() || jsy333PublishC.value() ? "on" : "off";
        break;
      default:
        break;
    }
    response->setLength();
    request->send(response);
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
  // Ref: https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Shelly#shellygetdeviceinfo
  webServer.on("/rpc/Shelly.GetDeviceInfo", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    ShellyGetDeviceInfo(root);
    response->setLength();
    request->send(response);
  });
  // API: /rpc/Shelly.GetStatus
  // Ref: https://shelly-api-docs.shelly.cloud/gen2/ComponentsAndServices/Shelly#shellygetstatus
  webServer.on("/rpc/Shelly.GetStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    // Shelly Pro EM 50
    EM1GetStatus(0, root["em1:0"].to<JsonObject>());
    EM1GetStatus(1, root["em1:1"].to<JsonObject>());
    EM1DataGetStatus(0, root["em1data:0"].to<JsonObject>());
    EM1DataGetStatus(1, root["em1data:1"].to<JsonObject>());
    // Shelly Pro 3EM
    EM1GetStatus(0, root["em:0"].to<JsonObject>());
    EM1GetStatus(1, root["em:1"].to<JsonObject>());
    EM1GetStatus(2, root["em:2"].to<JsonObject>());
    EM1DataGetStatus(0, root["emdata:0"].to<JsonObject>());
    EM1DataGetStatus(1, root["emdata:1"].to<JsonObject>());
    EM1DataGetStatus(2, root["emdata:2"].to<JsonObject>());
    // send response
    response->setLength();
    request->send(response);
  });
  // API: /rpc/EM1.GetStatus?id=0|1|2
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
    EM1GetStatus(id, root);
    response->setLength();
    request->send(response);
  });
  // API: /rpc/EM1Data.GetStatus?id=0|1|2
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
    EM1DataGetStatus(id, root);
    response->setLength();
    request->send(response);
  });
  // API: /rpc/EM.GetStatus?id=0|1|2
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
    if (savedJSYData.model != MYCILA_JSY_MK_333) {
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
    EMGetStatus(id, root);
    response->setLength();
    request->send(response);
  });
  // API: /rpc/EMData.GetStatus?id=0|1|2
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
    if (savedJSYData.model != MYCILA_JSY_MK_333) {
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
    EMDataGetStatus(id, root);
    response->setLength();
    request->send(response);
  });
  // API: /rpc
  // Returns the list of available API endpoints
  webServer.on("/rpc", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    root["/rpc/Shelly.GetDeviceInfo"] = "Returns the device information";
    root["/rpc/Shelly.GetStatus"] = "Returns the current Shelly status";
    if (savedJSYData.model == MYCILA_JSY_MK_163 || savedJSYData.model == MYCILA_JSY_MK_1031) {
      root["/rpc/EM1.GetStatus?id=0"] = "Returns the current EM1 status";
      root["/rpc/EM1Data.GetStatus?id=0"] = "Returns the current EM1 data status";
    } else if (savedJSYData.model == MYCILA_JSY_MK_333) {
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
  // API: /rpc
  // For old Shelly's
  // Returns the list of available API endpoints
  webServer.on("/emeter/0", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    EM1GetStatus(0, root);
    EM1DataGetStatus(0, root);
    root["is_valid"] = true;
    response->setLength();
    request->send(response);
  });
  webServer.on("/emeter/1", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    EM1GetStatus(1, root);
    EM1DataGetStatus(1, root);
    root["is_valid"] = true;
    response->setLength();
    request->send(response);
  });
  webServer.on("/emeter/2", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    EM1GetStatus(2, root);
    EM1DataGetStatus(2, root);
    root["is_valid"] = true;
    response->setLength();
    request->send(response);
  });
  webServer.on("/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncJsonResponse* response = new AsyncJsonResponse();
    JsonObject root = response->getRoot();
    JsonArray emeters = root["emeters"].to<JsonArray>();
    EM1GetStatus(0, emeters[0].to<JsonObject>());
    EM1DataGetStatus(0, emeters[0].as<JsonObject>());
    EM1GetStatus(1, emeters[1].to<JsonObject>());
    EM1DataGetStatus(1, emeters[1].as<JsonObject>());
    EM1GetStatus(2, emeters[2].to<JsonObject>());
    EM1DataGetStatus(2, emeters[2].as<JsonObject>());
    root["total_power"] = savedJSYData.aggregate.activePower;
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
  jsy194ChannelSwap.onChange([](bool state) {
    preferences.putBool("jsy194_ch_swap", state);
    jsy194ChannelSwap.setValue(state);
    dashboard.refresh(jsy194ChannelSwap);
  });
  jsy194ShellySwap.onChange([](bool state) {
    preferences.putBool("jsy194_sh_swap", state);
    jsy194ShellySwap.setValue(state);
    dashboard.refresh(jsy194ShellySwap);
  });
  jsy163Publish.onChange([](bool state) {
    preferences.putBool("jsy163_udp", state);
    jsy163Publish.setValue(state);
    dashboard.refresh(jsy163Publish);
  });
  jsy194Publish1.onChange([](bool state) {
    preferences.putBool("jsy194_ch1_udp", state);
    jsy194Publish1.setValue(state);
    dashboard.refresh(jsy194Publish1);
  });
  jsy194Publish2.onChange([](bool state) {
    preferences.putBool("jsy194_ch2_udp", state);
    jsy194Publish2.setValue(state);
    dashboard.refresh(jsy194Publish2);
  });
  jsy333PublishA.onChange([](bool state) {
    preferences.putBool("jsy333_a_udp", state);
    jsy333PublishA.setValue(state);
    dashboard.refresh(jsy333PublishA);
  });
  jsy333PublishB.onChange([](bool state) {
    preferences.putBool("jsy333_b_udp", state);
    jsy333PublishB.setValue(state);
    dashboard.refresh(jsy333PublishB);
  });
  jsy333PublishC.onChange([](bool state) {
    preferences.putBool("jsy333_c_udp", state);
    jsy333PublishC.setValue(state);
    dashboard.refresh(jsy333PublishC);
  });

  dashboard.onBeforeUpdate([](bool changes_only) {
    if (!changes_only) {
      ESP_LOGI(TAG, "Dashboard refresh requested");
      jsyModelCard.setValue(jsyModel == MYCILA_JSY_MK_UNKNOWN ? "Unknown" : jsy.getModelName());
      networkHostname.setValue(hostname);
    }
  });

  // Dashboard - Static Widgets Values
  versionJSY.setValue(MYCILA_JSY_VERSION);
  versionJSYApp.setValue(Mycila::AppInfo.version);
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
  espConnect.setBlocking(false);
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
    if (savedJSYData == data)
      return;

    savedJSYData = data;

    switch (savedJSYData.model) {
      case MYCILA_JSY_MK_1031:
      case MYCILA_JSY_MK_163: {
        if (!jsy163Publish.value()) {
          messageRate = 0;
          dataRate = 0;
          return;
        }
        break;
      }
      case MYCILA_JSY_MK_193:
      case MYCILA_JSY_MK_194: {
        const bool swap = jsy194ChannelSwap.value();
        savedJSYDataChannels[0] = swap ? savedJSYData.channel2() : savedJSYData.channel1();
        savedJSYDataChannels[1] = swap ? savedJSYData.channel1() : savedJSYData.channel2();

        if (!jsy194Publish1.value() && !jsy194Publish2.value()) {
          messageRate = 0;
          dataRate = 0;
          return;
        }
        break;
      }
      case MYCILA_JSY_MK_333: {
        if (!jsy333PublishA.value() && !jsy333PublishB.value() && !jsy333PublishC.value()) {
          messageRate = 0;
          dataRate = 0;
          return;
        }
        break;
      }
      default:
        break;
    }

    if (espConnect.getMode() == Mycila::ESPConnect::Mode::NONE) {
      messageRate = 0;
      dataRate = 0;
      return;
    }

    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    jsy.toJson(root);

    // filter json according to enabled udp publish
    switch (savedJSYData.model) {
      case MYCILA_JSY_MK_193:
      case MYCILA_JSY_MK_194: {
        if (jsy194ChannelSwap.value()) {
          savedJSYDataChannels[0].toJson(root["channel1"].to<JsonObject>());
          savedJSYDataChannels[1].toJson(root["channel2"].to<JsonObject>());
        }
        if (!jsy194Publish1.value()) {
          root.remove("channel1");
        }
        if (!jsy194Publish2.value()) {
          root.remove("channel2");
        }
        break;
      }
      case MYCILA_JSY_MK_333: {
        if (!jsy333PublishA.value()) {
          root.remove("phaseA");
        }
        if (!jsy333PublishB.value()) {
          root.remove("phaseB");
        }
        if (!jsy333PublishC.value()) {
          root.remove("phaseC");
        }
        break;
      }
      default:
        break;
    }

    // Serial.print("JSY UDP JSON: ");
    // serializeJson(root, Serial);
    // Serial.println();

    size_t totalSent = sendUDP(root);

    if (totalSent) {
      // update rate
      messageRateBuffer.add(static_cast<float>(esp_timer_get_time() / 1000000.0f));
      float diff = messageRateBuffer.diff();
      float count = messageRateBuffer.count();
      messageRate = diff == 0 ? 0 : count / diff;
      dataRateBuffer.add(totalSent);
      dataRate = diff == 0 ? 0 : dataRateBuffer.sum() / diff;
    }
  });

  jsy.begin(MYCILA_JSY_SERIAL, MYCILA_JSY_RX, MYCILA_JSY_TX);

  if (jsy.isEnabled() && jsy.getBaudRate() != jsy.getMaxAvailableBaudRate())
    jsy.setBaudRate(jsy.getMaxAvailableBaudRate());

  jsyModel = jsy.getModel();

  if (jsyModel == MYCILA_JSY_MK_193 || jsyModel == MYCILA_JSY_MK_194 || jsyModel == MYCILA_JSY_MK_333 || jsyModel == MYCILA_JSY_MK_UNKNOWN) {
    dashboard.remove(jsy163Publish);
    dashboard.remove(jsy163Data);
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

  if (jsyModel == MYCILA_JSY_MK_163 || jsyModel == MYCILA_JSY_MK_1031 || jsyModel == MYCILA_JSY_MK_333 || jsyModel == MYCILA_JSY_MK_UNKNOWN) {
    dashboard.remove(jsy194Publish1);
    dashboard.remove(jsy194Publish2);
    dashboard.remove(jsy194ChannelConfig);
    dashboard.remove(jsy194ChannelSwap);
    dashboard.remove(jsy194ShellySwap);
    dashboard.remove(jsy194Data1);
    dashboard.remove(jsy194Data2);
    dashboard.remove(jsy194Charts);
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

  if (jsyModel == MYCILA_JSY_MK_163 || jsyModel == MYCILA_JSY_MK_1031 || jsyModel == MYCILA_JSY_MK_193 || jsyModel == MYCILA_JSY_MK_194 || jsyModel == MYCILA_JSY_MK_UNKNOWN) {
    dashboard.remove(jsy333PublishA);
    dashboard.remove(jsy333PublishB);
    dashboard.remove(jsy333PublishC);
    dashboard.remove(jsy333DataA);
    dashboard.remove(jsy333DataB);
    dashboard.remove(jsy333DataC);
    dashboard.remove(jsy333Charts);
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

// static uint32_t last = 0;
// // can be put in 1 packet
// static const char* str_1335 = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum."
//                               "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.";
// // will need multiple packets
// static const char* str_2225 = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum."
//                               "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum."
//                               "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum."
//                               "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum."
//                               "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.";

void loop() {
  vTaskDelete(NULL);

  // if (millis() - last > 5000) {
  //   if (random(0, 2) == 0) {
  //     JsonDocument doc;
  //     JsonObject root = doc.to<JsonObject>();
  //     root["txt"] = str_1335;
  //     size_t sent = sendUDP(root);
  //     if (sent) {
  //       ESP_LOGI(TAG, "Sent large UDP packet (%" PRIu32 " bytes)", sent);
  //     }
  //   } else {
  //     JsonDocument doc;
  //     JsonObject root = doc.to<JsonObject>();
  //     root["txt"] = str_2225;
  //     size_t sent = sendUDP(root);
  //     if (sent) {
  //       ESP_LOGI(TAG, "Sent very large UDP packet (%" PRIu32 " bytes)", sent);
  //     }
  //   }
  //   last = millis();
  // }
}
