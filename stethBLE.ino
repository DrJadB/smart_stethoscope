// Steth BLE streamer (ESP32-S3 SuperMini, Arduino-ESP32 core 2.0.x)
//
// Samples the mic amplifier output (steth PCB J4 pin 3 "OUT" -> GPIO13) at a
// fixed rate and streams it as BLE notifications. Receiver: steth_ble_receiver.py
//
// Notification format (little-endian):
//   uint32 index of the first sample (counts up from boot; gaps = lost packets)
//   uint16 samples[]  12-bit ADC counts, as many as fit in the negotiated MTU
//
// GPIO13 is on ADC2, which is fine with Bluetooth but must not be used with Wi-Fi.
//
// Arduino IDE board settings: "ESP32S3 Dev Module", USB CDC On Boot: Enabled,
// Flash Size: 4MB, Partition Scheme: Huge APP (BLE does not fit the default 1.2MB app).
// arduino-cli: -b esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PartitionScheme=huge_app,FlashSize=4M
// LED: blue = advertising, green = connected. Serial (115200) prints rate/MTU/drops every 2 s.

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const int PIN_MIC = 13;
static const int PIN_RGB = 48;            // SuperMini WS2812 status LED
static const uint32_t SAMPLE_HZ = 4000;   // 4 kHz: heart + most lung sounds
static const uint16_t PKT_SAMPLES = 100;  // 25 ms per packet
static const char *DEV_NAME = "Steth";

#define SVC_UUID   "5e7a0001-2c4b-4f6e-9d3a-7b1e5c0a9f10"
#define AUDIO_UUID "5e7a0002-2c4b-4f6e-9d3a-7b1e5c0a9f10"  // notify
#define INFO_UUID  "5e7a0003-2c4b-4f6e-9d3a-7b1e5c0a9f10"  // read

static const int64_t PERIOD_US = 1000000LL / SAMPLE_HZ;

struct Packet {
  uint32_t first;
  uint16_t s[PKT_SAMPLES];
};

static QueueHandle_t pktQueue;
static BLEServer *server;
static BLECharacteristic *audioChr;
static volatile bool connected = false;
static volatile uint16_t connId = 0;
static volatile uint32_t queueDrops = 0, sentNotifies = 0;

static void setLed(uint8_t r, uint8_t g, uint8_t b) { neopixelWrite(PIN_RGB, r, g, b); }

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s, esp_ble_gatts_cb_param_t *param) override {
    connId = param->connect.conn_id;
    connected = true;
    // Ask for a 7.5-15 ms connection interval; the central may choose otherwise.
    s->updateConnParams(param->connect.remote_bda, 6, 12, 0, 400);
    setLed(0, 6, 0);
  }
  void onDisconnect(BLEServer *s) override {
    connected = false;
    setLed(0, 0, 6);
    BLEDevice::startAdvertising();
  }
};

// Core 0: drains the packet queue into notifications, split to fit the MTU.
static void bleSendTask(void *) {
  static Packet p;
  static uint8_t buf[4 + 2 * PKT_SAMPLES];
  for (;;) {
    if (xQueueReceive(pktQueue, &p, portMAX_DELAY) != pdTRUE || !connected) continue;
    uint16_t mtu = server->getPeerMTU(connId);
    uint16_t per = (mtu > 9) ? (mtu - 3 - 4) / 2 : 1;
    if (per > PKT_SAMPLES) per = PKT_SAMPLES;
    for (uint16_t off = 0; off < PKT_SAMPLES; off += per) {
      uint16_t k = min<uint16_t>(per, PKT_SAMPLES - off);
      uint32_t first = p.first + off;
      memcpy(buf, &first, 4);
      memcpy(buf + 4, &p.s[off], 2 * k);
      audioChr->setValue(buf, 4 + 2 * k);
      audioChr->notify();
      sentNotifies++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  setLed(0, 0, 6);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MIC, ADC_11db);  // ~0-3.1 V
  pinMode(PIN_MIC, INPUT);

  pktQueue = xQueueCreate(16, sizeof(Packet));  // 400 ms of slack

  BLEDevice::init(DEV_NAME);
  BLEDevice::setMTU(247);
  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *svc = server->createService(SVC_UUID);
  audioChr = svc->createCharacteristic(AUDIO_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  audioChr->addDescriptor(new BLE2902());
  BLECharacteristic *info = svc->createCharacteristic(INFO_UUID, BLECharacteristic::PROPERTY_READ);
  char infoStr[64];
  snprintf(infoStr, sizeof(infoStr), "fs=%u;bits=12;pin=%d;fmt=u32first,u16le", SAMPLE_HZ, PIN_MIC);
  info->setValue(infoStr);
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  xTaskCreatePinnedToCore(bleSendTask, "bleSend", 4096, nullptr, 5, nullptr, 0);
  Serial.printf("Steth BLE: advertising as \"%s\", %u Hz on GPIO%d\n", DEV_NAME, SAMPLE_HZ, PIN_MIC);
}

// Core 1: paced sampling loop (busy-waits between samples, like adcReaderESP).
void loop() {
  static Packet pkt;
  static uint16_t n = 0;
  static uint32_t idx = 0;
  static int64_t next = esp_timer_get_time();
  static int64_t lastStats = 0;
  static uint32_t statSamples = 0;

  int64_t now;
  while ((now = esp_timer_get_time()) < next) {}
  next += PERIOD_US;
  if (now - next > 20 * PERIOD_US) next = now + PERIOD_US;  // fell far behind: resync

  if (n == 0) pkt.first = idx;
  pkt.s[n++] = analogRead(PIN_MIC);
  idx++;
  statSamples++;

  if (n == PKT_SAMPLES) {
    n = 0;
    if (connected && xQueueSend(pktQueue, &pkt, 0) != pdTRUE) queueDrops++;
  }

  if (now - lastStats > 2000000) {
    float secs = (now - lastStats) / 1e6f;
    Serial.printf("rate %.0f Hz | last %u | %s | mtu %u | notifies %u | queue drops %u\n",
                  statSamples / secs, pkt.s[(n + PKT_SAMPLES - 1) % PKT_SAMPLES],
                  connected ? "connected" : "advertising",
                  connected ? server->getPeerMTU(connId) : 0, sentNotifies, queueDrops);
    lastStats = now;
    statSamples = 0;
  }
}
