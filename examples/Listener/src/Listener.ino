// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) Mathieu Carbou
 */
#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>

#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include <FastCRC32.h>   // https://github.com/RobTillaart/CRC

// #define MYCILA_UDP_MSG_TYPE_JSY_DATA 0x01 // old json format
#define MYCILA_UDP_MSG_TYPE_JSY_DATA 0x02 // supports all JSY models
#define MYCILA_UDP_PORT              53964

static AsyncUDP udp;

static uint8_t* reassembledMessage = nullptr;
static size_t reassembledMessageIndex = 0;
static size_t reassembledMessageRemaining = 0;

void setup() {
  Serial.begin(115200);

#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
  delay(100);
#else
  while (!Serial)
    yield();
#endif

  udp.onPacket([](AsyncUDPPacket packet) {
    // buffer[0] == MYCILA_UDP_MSG_TYPE_JSY_DATA (1)
    // buffer[1] == size_t (4)
    // buffer[5] == MsgPack (?)
    // buffer[5 + size] == CRC32 (4)

    size_t len = packet.length();
    uint8_t* buffer = packet.data();

    ESP_LOGD("Listener", "Received UDP packet of size %" PRIu32, len);

    // this is a new message ?
    if (reassembledMessage == nullptr) {
      // check message validity
      if (len < 10 || buffer[0] != MYCILA_UDP_MSG_TYPE_JSY_DATA)
        return;

      // extract message size
      memcpy(&reassembledMessageRemaining, buffer + 1, 4);

      // a message must have a length
      if (!reassembledMessageRemaining) {
        ESP_LOGD("Listener", "Invalid message size: 0");
        return;
      }

      if (reassembledMessageRemaining > 4096) { // arbitrary limit to avoid memory exhaustion
        ESP_LOGD("Listener", "Message size too large: %" PRIu32, reassembledMessageRemaining);
        reassembledMessageRemaining = 0;
        return;
      }

      reassembledMessageRemaining += 9; // add header and CRC32 size

      ESP_LOGD("Listener", "Allocating new message of size %" PRIu32, reassembledMessageRemaining);

      // allocate new message buffer
      reassembledMessageIndex = 0;
      reassembledMessage = new uint8_t[reassembledMessageRemaining];
    }

    // assemble packets
    if (reassembledMessage != nullptr) {
      size_t n = std::min(reassembledMessageRemaining, len);
      ESP_LOGD("Listener", "Appending packet of size %" PRIu32, n);
      memcpy(reassembledMessage + reassembledMessageIndex, buffer, n);
      reassembledMessageIndex += n;
      reassembledMessageRemaining -= n;
    }

    // cannot process yet - wait for more packets
    if (reassembledMessageRemaining)
      return;

    // we have finished reassembling packets
    ESP_LOGD("Listener", "Reassembled full message of size %" PRIu32, reassembledMessageIndex);

    // CRC32 check (last 4 bytes are the CRC32)
    FastCRC32 crc32;
    crc32.add(reassembledMessage, reassembledMessageIndex - 4);
    uint32_t crc = crc32.calc();

    // verify CRC32
    if (memcmp(&crc, reassembledMessage + reassembledMessageIndex - 4, 4) != 0) {
      ESP_LOGD("Listener", "CRC32 mismatch - expected 0x%08" PRIX32 ", got 0x%08" PRIX32, crc, *((uint32_t*)(reassembledMessage + reassembledMessageIndex - 4)));
      delete[] reassembledMessage;
      reassembledMessage = nullptr;
      reassembledMessageIndex = 0;
      reassembledMessageRemaining = 0;
      return;
    }

    // extract message
    ESP_LOGD("Listener", "CRC32 valid - parsing message");
    JsonDocument doc;
    DeserializationError err = deserializeMsgPack(doc, reassembledMessage + 5, reassembledMessageIndex - 9);

    // cleanup reassembled message buffer
    delete[] reassembledMessage;
    reassembledMessage = nullptr;
    reassembledMessageIndex = 0;
    reassembledMessageRemaining = 0;

    if (err) {
      ESP_LOGD("Listener", "Failed to parse MsgPack: %s", err.c_str());
      return;
    }

    // process JSON message
    serializeJsonPretty(doc, Serial);
    Serial.println();
  });

  WiFi.mode(WIFI_STA);
  WiFi.begin("IoT");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.print("Listener IP Address: ");
  Serial.println(WiFi.localIP());

  udp.listen(MYCILA_UDP_PORT);
}

// Destroy default Arduino async task
void loop() { vTaskDelete(NULL); }
