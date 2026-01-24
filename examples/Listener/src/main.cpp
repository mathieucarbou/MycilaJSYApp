// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) Mathieu Carbou
 */
#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>

#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include <FastCRC32.h>   // https://github.com/RobTillaart/CRC

#include <algorithm>

// #define MYCILA_UDP_MSG_TYPE_JSY_DATA 0x01 // old json format
#define YASOLR_UDP_MSG_TYPE_JSY_DATA 0x03 // supports all JSY models
#define MYCILA_UDP_PORT              53964
#define MYCILA_MAX_UDP_MESSAGE_SIZE  4096

#define TAG "Listener"

static AsyncUDP udp;

class UDPMessage {
  public:
    explicit UDPMessage(size_t size) : data(new uint8_t[size]), remaining(size), index(0), lastPacketTime(millis()) {}

    ~UDPMessage() {
      delete[] data;
    }

    void append(const uint8_t* buffer, size_t len) {
      const size_t n = std::min(remaining, len);
      // ESP_LOGD(TAG, "[UDP] Appending packet of size %" PRIu32 " to message buffer at position %" PRIu32, n, index);
      memcpy(data + index, buffer, n);
      index += n;
      remaining -= n;
    }

    bool crcValid() {
      FastCRC32 crc32;
      crc32.add(data, index - 4);
      uint32_t crc = crc32.calc();
      return memcmp(&crc, data + index - 4, 4) == 0;
    }

    DeserializationError parseMsgPack(JsonDocument& doc) {
      return deserializeMsgPack(doc, data + 9, index - 13);
    }

    uint8_t* data = nullptr;
    size_t remaining = 0;
    size_t index = 0;
    uint32_t lastPacketTime = 0;
    uint8_t sourceMAC[6] = {0};
    IPAddress interface;
};

static uint32_t lastMessageID = 0;
static UDPMessage* reassembledMessage = nullptr;

static void processJSON(const JsonDocument& doc) {
  // Example processing: print the JSON document to Serial
  serializeJson(doc, Serial);
  Serial.println();
}

static void onData(AsyncUDPPacket& packet) {
  // buffer[0] == YASOLR_UDP_MSG_TYPE_JSY_DATA (1)
  // buffer[1] == message ID (4) - uint32_t
  // buffer[5] == jsonSize (4) - size_t
  // buffer[9] == MsgPack (?)
  // buffer[9 + size] == CRC32 (4)

  const size_t len = packet.length();
  const uint8_t* buffer = packet.data();

  ESP_LOGD(TAG, "[UDP] Received packet of size %" PRIu32, len);

  // if we are waiting for more packets, check for timeout (10 seconds) in case a sender disappears mid-message
  // we free the buffer and reset state before processing the new packet
  if (reassembledMessage && reassembledMessage->remaining && millis() - reassembledMessage->lastPacketTime > 10000) {
    ESP_LOGD(TAG, "[UDP] Timeout waiting for more packets from sender: %02X:%02X:%02X:%02X:%02X:%02X", reassembledMessage->sourceMAC[0], reassembledMessage->sourceMAC[1], reassembledMessage->sourceMAC[2], reassembledMessage->sourceMAC[3], reassembledMessage->sourceMAC[4], reassembledMessage->sourceMAC[5]);
    delete reassembledMessage;
    reassembledMessage = nullptr;
  }

  // this is a new message ?
  if (reassembledMessage == nullptr) {
    // check message validity
    if (len <= 13 || buffer[0] != YASOLR_UDP_MSG_TYPE_JSY_DATA) {
      ESP_LOGD(TAG, "[UDP] Invalid packet received of size %" PRIu32 ", not coming from a supported Mycila JSY App version!", len);
      return;
    } else {
      ESP_LOGD(TAG, "[UDP] New UDP packet detected!");
    }

    // extract message ID
    uint32_t messageID = 0;
    memcpy(&messageID, buffer + 1, 4);

    // check for duplicate message ID
    if (messageID == lastMessageID) {
      ESP_LOGD(TAG, "[UDP] Received duplicate message ID: %" PRIu32 ". Please remove WiFi SSID or disconnect ETH!", messageID);
      return;
    }

    // extract message size
    size_t jsonSize = 0;
    memcpy(&jsonSize, buffer + 5, 4);

    // validate message size
    if (!jsonSize) {
      ESP_LOGD(TAG, "[UDP] Invalid message size: 0");
      return;
    } else {
      ESP_LOGD(TAG, "[UDP] Payload size: %" PRIu32, jsonSize);
    }

    // arbitrary limit to avoid memory exhaustion
    if (jsonSize > 4096) {
      ESP_LOGD(TAG, "[UDP] Message size too large: %" PRIu32, jsonSize);
      return;
    }

    // compute total reassembled message size and allocate buffer
    reassembledMessage = new (std::nothrow) UDPMessage(jsonSize + 13);
    if (reassembledMessage == nullptr) {
      ESP_LOGD(TAG, "[UDP] Failed to allocate memory for reassembled message of size %" PRIu32, jsonSize + 13);
      return;
    } else {
      ESP_LOGD(TAG, "[UDP] Allocated new message buffer of size %" PRIu32, jsonSize + 13);
    }

    // save last message ID
    lastMessageID = messageID;

    // save sender MAC address
    packet.remoteMac(reassembledMessage->sourceMAC);

    // save interface IP address
    reassembledMessage->interface = packet.localIP();
    if (reassembledMessage->interface == IPAddress()) {
      reassembledMessage->interface = packet.localIPv6();
    }

  } else {
    ESP_LOGD(TAG, "[UDP] Validating next packet");

    // check that the packet comes from the same sender
    uint8_t mac[6];
    packet.remoteMac(mac);
    if (memcmp(mac, reassembledMessage->sourceMAC, 6) != 0) {
      ESP_LOGD(TAG, "[UDP] Discarding packet from different sender. Expected: %02X:%02X:%02X:%02X:%02X:%02X, got %02X:%02X:%02X:%02X:%02X:%02X", reassembledMessage->sourceMAC[0], reassembledMessage->sourceMAC[1], reassembledMessage->sourceMAC[2], reassembledMessage->sourceMAC[3], reassembledMessage->sourceMAC[4], reassembledMessage->sourceMAC[5], mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      return;
    } else {
      ESP_LOGD(TAG, "[UDP] Packet sender validated: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    // check if the packet comes by the same interface
    if (packet.localIP() != reassembledMessage->interface && packet.localIPv6() != reassembledMessage->interface) {
      ESP_LOGD(TAG, "[UDP] Discarding packet from different interface. Expected: %s, got IPv4: %s, IPv6: %s", reassembledMessage->interface.toString().c_str(), packet.localIP().toString().c_str(), packet.localIPv6().toString().c_str());
      return;
    } else {
      ESP_LOGD(TAG, "[UDP] Packet interface validated");
    }

    // additional packet validated
    reassembledMessage->lastPacketTime = millis();
  }

  // assemble packets
  reassembledMessage->append(buffer, len);

  if (reassembledMessage->remaining) {
    ESP_LOGD(TAG, "[UDP] Waiting for more packets to complete message. Remaining size: %" PRIu32, reassembledMessage->remaining);
    return;
  } else {
    ESP_LOGD(TAG, "[UDP] Reassembled complete message of size %" PRIu32, reassembledMessage->index);
  }

  // we have finished reassembling packets
  // verify CRC32
  if (!reassembledMessage->crcValid()) {
    ESP_LOGD(TAG, "[UDP] CRC32 mismatch");
    delete reassembledMessage;
    reassembledMessage = nullptr;
    return;
  } else {
    ESP_LOGD(TAG, "[UDP] CRC32 valid");
  }

  // extract message
  JsonDocument doc;
  DeserializationError err = reassembledMessage->parseMsgPack(doc);

  if (err) {
    ESP_LOGD(TAG, "[UDP] Failed to parse MsgPack: %s", err.c_str());
    delete reassembledMessage;
    reassembledMessage = nullptr;
    return;
  } else {
    ESP_LOGD(TAG, "[UDP] MsgPack parsed successfully");
  }

  processJSON(doc);

  delete reassembledMessage;
  reassembledMessage = nullptr;
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

  WiFi.mode(WIFI_STA);
  WiFi.begin("IoT");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.print("Listener IP Address: ");
  Serial.println(WiFi.localIP());

  udp.onPacket(onData);
  udp.listen(MYCILA_UDP_PORT);
}

// Destroy default Arduino async task
void loop() { vTaskDelete(NULL); }
