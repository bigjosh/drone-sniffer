// Drone ID sniffer — display node for Heltec WiFi LoRa 32 V3.
//
// Reads JSON detection lines from the XIAO sniffer over UART1 (one line per
// detection: {"id":..,"mac":..,"rssi":..,"tp":"ble"|"wifi",["ch":n,][dlat,dlon,plat,plon]})
// and shows a rotating view of recently seen drones on the SSD1306 OLED.
//
// It also sniffs Wi-Fi itself, hopping channels the XIAO cannot cover (see
// SCAN_CHANNELS), so Beacon-method Remote ID off channel 6 is picked up too.
// Set ENABLE_WIFI_SCAN to 0 to build the display-only firmware.
//
// The same JSON lines are accepted on the USB serial port so the whole UI can
// be exercised from a PC without RF. USB also accepts test commands:
//   dump        print the drone table state
//   status      print scanner state (channel, hop count, frames heard)
//   reset       clear the drone table
//   backdate N  age every record by N seconds (expiry testing)
//
// LoRa (SX1262) is intentionally left uninitialized.

#include <Arduino.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>

#ifndef ENABLE_WIFI_SCAN
#define ENABLE_WIFI_SCAN 1
#endif

#if ENABLE_WIFI_SCAN
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_random.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "opendroneid.h"
#include "odid_wifi.h"
#endif

// ---- Heltec WiFi LoRa 32 V3 pins ----
static const int PIN_VEXT     = 36;  // active LOW, powers the OLED
static const int PIN_OLED_RST = 21;
static const int PIN_OLED_SDA = 17;
static const int PIN_OLED_SCL = 18;
static const int PIN_BUTTON   = 0;   // PRG button, LOW when pressed
static const int PIN_LED      = 35;
static const int PIN_LINK_RX  = 19;  // from XIAO GPIO5 (TX)
static const int PIN_LINK_TX  = 20;  // to XIAO GPIO6 (RX)

static const uint32_t EXPIRE_MS  = 5UL * 60UL * 1000UL;
static const uint32_t DWELL_MS   = 4000;  // auto-advance interval
static const uint32_t RENDER_MS  = 250;
static const uint32_t LED_MS     = 80;    // RX blink duration

// U8G2_R0 = default orientation, U8G2_R2 = rotated 180 degrees.
#define DISPLAY_ROTATION U8G2_R2

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(DISPLAY_ROTATION, PIN_OLED_RST, PIN_OLED_SCL, PIN_OLED_SDA);

struct DroneRec {
  bool     used;
  char     id[21];    // UAS Basic ID, may be empty
  char     mac[18];
  bool     isBle;
  uint8_t  channel;   // Wi-Fi channel heard on; 0 = unknown or not applicable
  int      rssi;
  bool     hasPilot;
  double   plat, plon;
  bool     hasDrone;
  double   dlat, dlon;
  uint32_t lastSeen;
};

// One decoded broadcast, from either the serial link or our own radio.
struct Detection {
  char     id[21];
  char     mac[18];
  bool     isBle;
  uint8_t  channel;
  int      rssi;
  bool     hasDrone;
  double   dlat, dlon;
  bool     hasPilot;
  double   plat, plon;
};

static const int MAX_DRONES = 32;
static DroneRec drones[MAX_DRONES];
static int      curPos = 0;          // position within the list of used slots
static uint32_t lastAdvance = 0;
static uint32_t lastRender = 0;
static uint32_t lastExpiryCheck = 0;
static uint32_t lastRx = 0;

static int collectUsed(int *list) {
  int m = 0;
  for (int i = 0; i < MAX_DRONES; i++)
    if (drones[i].used) list[m++] = i;
  return m;
}

static void lowercaseMac(char *mac) {
  for (char *p = mac; *p; p++) *p = tolower((unsigned char)*p);
}

static void upsert(const Detection &det) {
  int slot = -1;

  if (det.id[0] != '\0') {
    for (int i = 0; i < MAX_DRONES; i++)
      if (drones[i].used && strcmp(drones[i].id, det.id) == 0) { slot = i; break; }
  }
  if (slot < 0) {
    for (int i = 0; i < MAX_DRONES; i++)
      if (drones[i].used && strcmp(drones[i].mac, det.mac) == 0) { slot = i; break; }
  }
  if (slot < 0) {
    for (int i = 0; i < MAX_DRONES; i++)
      if (!drones[i].used) { slot = i; break; }
  }
  if (slot < 0) {
    // Table full: evict the least recently seen.
    uint32_t oldest = 0;
    slot = 0;
    for (int i = 0; i < MAX_DRONES; i++) {
      uint32_t age = millis() - drones[i].lastSeen;
      if (age > oldest) { oldest = age; slot = i; }
    }
    Serial.printf("[EVICT] %s\n", drones[slot].mac);
    drones[slot].used = false;
  }

  DroneRec &d = drones[slot];
  if (!d.used) {
    memset(&d, 0, sizeof(d));
    d.used = true;
  }
  if (det.id[0] != '\0') {
    strncpy(d.id, det.id, sizeof(d.id) - 1);
    d.id[sizeof(d.id) - 1] = '\0';
  }
  strncpy(d.mac, det.mac, sizeof(d.mac) - 1);
  d.mac[sizeof(d.mac) - 1] = '\0';
  d.isBle = det.isBle;
  d.channel = det.channel;
  d.rssi = det.rssi;
  if (det.hasDrone) { d.hasDrone = true; d.dlat = det.dlat; d.dlon = det.dlon; }
  if (det.hasPilot) { d.hasPilot = true; d.plat = det.plat; d.plon = det.plon; }
  d.lastSeen = millis();
}

// Remote ID strings are CTA-2063 ASCII; keep anything unprintable off the OLED.
static void sanitizeId(const char *in, size_t inLen, char *out, size_t outsz) {
  size_t j = 0;
  for (size_t i = 0; i < inLen && in[i] != '\0' && j + 1 < outsz; i++) {
    char c = in[i];
    out[j++] = (c < 0x20 || c == 0x7F) ? '?' : c;
  }
  out[j] = '\0';
}

#if ENABLE_WIFI_SCAN

// Channel 6 is deliberately excluded: the XIAO sniffer sits there permanently,
// and it is also the Wi-Fi NAN social channel, so NAN Remote ID is already fully
// covered. Sweeping the rest adds Beacon-method coverage. 12-14 are omitted
// because they are region-restricted.
static const uint8_t SCAN_CHANNELS[] = {1, 2, 3, 4, 5, 7, 8, 9, 10, 11};
static const int NUM_SCAN_CHANNELS = sizeof(SCAN_CHANNELS) / sizeof(SCAN_CHANNELS[0]);
static const uint32_t HOP_INTERVAL_MS = 1500;

static uint8_t hopOrder[NUM_SCAN_CHANNELS];
static int hopIndex = 0;
static uint8_t currentChannel = 0;
static uint32_t lastHop = 0;
static uint32_t hopCount = 0;
static uint32_t rfHeard = 0;
static QueueHandle_t detQueue = nullptr;
static ODID_UAS_Data rfUas;  // decoder scratch; kept off the Wi-Fi task stack

// Shuffled sweep rather than independent random picks: same lack of a fixed
// pattern, but every channel is guaranteed to be visited once per cycle.
static void shuffleHopOrder() {
  for (int i = NUM_SCAN_CHANNELS - 1; i > 0; i--) {
    int j = (int)(esp_random() % (uint32_t)(i + 1));
    uint8_t t = hopOrder[i];
    hopOrder[i] = hopOrder[j];
    hopOrder[j] = t;
  }
}

static void queueFromUas(const uint8_t *mac, int rssi) {
  Detection det;
  memset(&det, 0, sizeof(det));
  snprintf(det.mac, sizeof(det.mac), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  det.isBle = false;
  det.channel = currentChannel;
  det.rssi = rssi;

  if (rfUas.BasicIDValid[0]) {
    sanitizeId(rfUas.BasicID[0].UASID, ODID_ID_SIZE, det.id, sizeof(det.id));
  }
  if (rfUas.LocationValid) {
    det.hasDrone = true;
    det.dlat = rfUas.Location.Latitude;
    det.dlon = rfUas.Location.Longitude;
  }
  if (rfUas.SystemValid) {
    det.hasPilot = true;
    det.plat = rfUas.System.OperatorLatitude;
    det.plon = rfUas.System.OperatorLongitude;
  }
  xQueueSend(detQueue, &det, 0);  // drop rather than block the Wi-Fi task
}

// Runs in the Wi-Fi task, so it only decodes and queues; the table is owned by
// loop(). The driver is configured to hand us management frames only.
static void wifiSniffCallback(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;

  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  uint8_t *payload = pkt->payload;
  int length = pkt->rx_ctrl.sig_len;
  if (length < 24) return;

  static const uint8_t nan_dest[6] = {0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00};
  if (memcmp(nan_dest, &payload[4], 6) == 0) {
    memset(&rfUas, 0, sizeof(rfUas));
    if (odid_wifi_receive_message_pack_nan_action_frame(&rfUas, nullptr, payload, length) == 0) {
      queueFromUas(&payload[10], pkt->rx_ctrl.rssi);
    }
    return;
  }

  if (payload[0] != 0x80) return;  // beacon only

  int offset = 36;  // 24 byte header + 12 byte fixed beacon params
  while (offset + 2 <= length) {
    int ieLen = payload[offset + 1];
    if (payload[offset] == 0xdd && offset + 7 <= length &&
        ((payload[offset + 2] == 0x90 && payload[offset + 3] == 0x3a && payload[offset + 4] == 0xe6) ||
         (payload[offset + 2] == 0xfa && payload[offset + 3] == 0x0b && payload[offset + 4] == 0xbc))) {
      int j = offset + 7;  // skip element id, length, OUI, OUI type, msg counter
      if (j < length) {
        memset(&rfUas, 0, sizeof(rfUas));
        odid_message_process_pack(&rfUas, &payload[j], length - j);
        queueFromUas(&payload[10], pkt->rx_ctrl.rssi);
      }
      return;
    }
    offset += ieLen + 2;
  }
}

static void wifiScanStart() {
  detQueue = xQueueCreate(8, sizeof(Detection));
  for (int i = 0; i < NUM_SCAN_CHANNELS; i++) hopOrder[i] = SCAN_CHANNELS[i];
  shuffleHopOrder();

  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();  // may already exist; harmless if so

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_NULL);  // receive only, never transmits
  esp_wifi_start();

  // Filtering in the driver rather than the callback: without this we take an
  // interrupt for every data frame in the air.
  wifi_promiscuous_filter_t filter;
  memset(&filter, 0, sizeof(filter));
  filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(&wifiSniffCallback);
  esp_wifi_set_promiscuous(true);

  currentChannel = hopOrder[0];
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  lastHop = millis();

  Serial.printf("[BOOT] wifi scan on %d channels, %lu ms dwell, starting ch%u\n",
                NUM_SCAN_CHANNELS, (unsigned long)HOP_INTERVAL_MS, currentChannel);
}

static void wifiScanPump() {
  Detection det;
  while (detQueue && xQueueReceive(detQueue, &det, 0) == pdTRUE) {
    upsert(det);
    lastRx = millis();
    rfHeard++;
    Serial.printf("[RF] ch%u rssi=%d id=%s mac=%s\n",
                  det.channel, det.rssi, det.id[0] ? det.id : "-", det.mac);
  }

  uint32_t now = millis();
  if (now - lastHop >= HOP_INTERVAL_MS) {
    lastHop = now;
    if (++hopIndex >= NUM_SCAN_CHANNELS) {
      hopIndex = 0;
      shuffleHopOrder();
    }
    currentChannel = hopOrder[hopIndex];
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
    hopCount++;
  }
}

#endif  // ENABLE_WIFI_SCAN

static void printDump() {
  int list[MAX_DRONES];
  int m = collectUsed(list);
  for (int n = 0; n < m; n++) {
    DroneRec &d = drones[list[n]];
    Serial.printf("[DUMP] %d/%d id=%s mac=%s tp=%s rssi=%d age=%lus pilot=%d drone=%d\n",
                  n + 1, m, d.id[0] ? d.id : "-", d.mac, d.isBle ? "ble" : "wifi",
                  d.rssi, (unsigned long)((millis() - d.lastSeen) / 1000),
                  d.hasPilot, d.hasDrone);
  }
  Serial.printf("[DUMP] count=%d cur=%d\n", m, m ? curPos + 1 : 0);
}

static void handleCommand(const char *line) {
  if (strcmp(line, "dump") == 0) {
    printDump();
  } else if (strcmp(line, "status") == 0) {
#if ENABLE_WIFI_SCAN
    Serial.printf("[STATUS] wifi scan on, ch%u, hops=%lu, frames=%lu, dwell=%lums\n",
                  currentChannel, (unsigned long)hopCount, (unsigned long)rfHeard,
                  (unsigned long)HOP_INTERVAL_MS);
#else
    Serial.println("[STATUS] wifi scan disabled at build time");
#endif
  } else if (strcmp(line, "reset") == 0) {
    memset(drones, 0, sizeof(drones));
    curPos = 0;
    Serial.println("[OK] table cleared");
  } else if (strncmp(line, "backdate ", 9) == 0) {
    uint32_t secs = strtoul(line + 9, nullptr, 10);
    for (int i = 0; i < MAX_DRONES; i++)
      if (drones[i].used) drones[i].lastSeen -= secs * 1000UL;
    Serial.printf("[OK] backdated all records %lus\n", (unsigned long)secs);
  }
}

static void handleLine(const char *line, const char *src) {
  if (line[0] != '{') {
    handleCommand(line);
    return;
  }

  StaticJsonDocument<384> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.printf("[ERR] bad json from %s: %s\n", src, err.c_str());
    return;
  }
  const char *mac = doc["mac"] | "";
  if (mac[0] == '\0') {
    Serial.printf("[ERR] missing mac from %s\n", src);
    return;
  }
  char macLower[18];
  strncpy(macLower, mac, sizeof(macLower) - 1);
  macLower[sizeof(macLower) - 1] = '\0';
  lowercaseMac(macLower);

  const char *id = doc["id"] | "";
  const char *tp = doc["tp"] | "wifi";

  Detection det;
  memset(&det, 0, sizeof(det));
  strncpy(det.id, id, sizeof(det.id) - 1);
  strncpy(det.mac, macLower, sizeof(det.mac) - 1);
  det.isBle = strcmp(tp, "ble") == 0;
  det.channel = (uint8_t)(doc["ch"] | 0);  // absent on older sniffer firmware
  det.rssi = doc["rssi"] | 0;
  det.hasDrone = doc.containsKey("dlat") && doc.containsKey("dlon");
  det.dlat = doc["dlat"] | 0.0;
  det.dlon = doc["dlon"] | 0.0;
  det.hasPilot = doc.containsKey("plat") && doc.containsKey("plon");
  det.plat = doc["plat"] | 0.0;
  det.plon = doc["plon"] | 0.0;

  upsert(det);
  lastRx = millis();

  int list[MAX_DRONES];
  int m = collectUsed(list);
  Serial.printf("[RX] %s %s rssi=%d id=%s mac=%s (tracked=%d)\n",
                src, tp, det.rssi, id[0] ? id : "-", macLower, m);
}

// Line-buffered reader; overlong lines are discarded.
static void pumpStream(Stream &s, const char *src, char *buf, size_t bufsz, size_t &len) {
  while (s.available()) {
    char c = (char)s.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handleLine(buf, src);
        len = 0;
      }
    } else if (len < bufsz - 1) {
      buf[len++] = c;
    } else {
      len = 0;
    }
  }
}

static void formatAge(uint32_t ageSecs, char *out, size_t outsz) {
  if (ageSecs < 60)
    snprintf(out, outsz, "seen %lus ago", (unsigned long)ageSecs);
  else
    snprintf(out, outsz, "seen %lum%02lus ago",
             (unsigned long)(ageSecs / 60), (unsigned long)(ageSecs % 60));
}

static void render() {
  int list[MAX_DRONES];
  int m = collectUsed(list);
  if (curPos >= m) curPos = (m > 0) ? m - 1 : 0;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13B_tr);
  char line[26];

  if (m == 0) {
    u8g2.drawStr(0, 12, "0/0");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 38, "No drones detected");
  } else {
    DroneRec &d = drones[list[curPos]];
    snprintf(line, sizeof(line), "%d/%d", curPos + 1, m);
    u8g2.drawStr(0, 12, line);

    u8g2.setFont(u8g2_font_6x10_tf);
    formatAge((millis() - d.lastSeen) / 1000, line, sizeof(line));
    u8g2.drawStr(0, 26, line);
    u8g2.drawStr(0, 38, d.id[0] ? d.id : d.mac);
    if (d.isBle)
      snprintf(line, sizeof(line), "BLE  %d dBm", d.rssi);
    else if (d.channel)
      snprintf(line, sizeof(line), "WiFi ch%u  %d dBm", d.channel, d.rssi);
    else
      snprintf(line, sizeof(line), "WiFi  %d dBm", d.rssi);
    u8g2.drawStr(0, 50, line);
    if (d.hasPilot)
      snprintf(line, sizeof(line), "P: %.4f,%.4f", d.plat, d.plon);
    else
      snprintf(line, sizeof(line), "P: --");
    u8g2.drawStr(0, 62, line);
  }
  u8g2.sendBuffer();
}

static void handleButton() {
  static int lastState = HIGH;
  static uint32_t lastChange = 0;
  int b = digitalRead(PIN_BUTTON);
  if (b != lastState && millis() - lastChange > 50) {
    lastChange = millis();
    lastState = b;
    if (b == LOW) {
      int list[MAX_DRONES];
      int m = collectUsed(list);
      if (m > 1) curPos = (curPos + 1) % m;
      lastAdvance = millis();
      lastRender = 0;  // redraw immediately
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);  // Vext on -> OLED powered
  delay(100);

  u8g2.setBusClock(400000);
  u8g2.begin();

  memset(drones, 0, sizeof(drones));
  render();
  Serial.println("[BOOT] drone sniffer display node ready");

#if ENABLE_WIFI_SCAN
  wifiScanStart();
#endif
}

void loop() {
  static char usbBuf[300], linkBuf[300];
  static size_t usbLen = 0, linkLen = 0;

  pumpStream(Serial, "usb", usbBuf, sizeof(usbBuf), usbLen);
  pumpStream(Serial1, "link", linkBuf, sizeof(linkBuf), linkLen);
  handleButton();
#if ENABLE_WIFI_SCAN
  wifiScanPump();
#endif

  uint32_t now = millis();

  if (now - lastExpiryCheck >= 1000) {
    lastExpiryCheck = now;
    for (int i = 0; i < MAX_DRONES; i++) {
      if (drones[i].used && now - drones[i].lastSeen > EXPIRE_MS) {
        Serial.printf("[EXPIRE] %s\n", drones[i].mac);
        drones[i].used = false;
      }
    }
  }

  {
    int list[MAX_DRONES];
    int m = collectUsed(list);
    if (m > 1 && now - lastAdvance >= DWELL_MS) {
      lastAdvance = now;
      curPos = (curPos + 1) % m;
      lastRender = 0;
    }
  }

  digitalWrite(PIN_LED, (now - lastRx < LED_MS && lastRx != 0) ? HIGH : LOW);

  if (lastRender == 0 || now - lastRender >= RENDER_MS) {
    lastRender = now;
    render();
  }

  delay(2);
}
