// Drone ID sniffer — display node for Heltec WiFi LoRa 32 V3.
//
// Reads JSON detection lines from the XIAO sniffer over UART1 (one line per
// detection: {"id":..,"mac":..,"rssi":..,"tp":"ble"|"wifi",[dlat,dlon,plat,plon]})
// and shows a rotating view of recently seen drones on the SSD1306 OLED.
//
// The same JSON lines are accepted on the USB serial port so the whole UI can
// be exercised from a PC without RF. USB also accepts test commands:
//   dump        print the drone table state
//   reset       clear the drone table
//   backdate N  age every record by N seconds (expiry testing)
//
// LoRa (SX1262) is intentionally left uninitialized.

#include <Arduino.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>

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

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, PIN_OLED_RST, PIN_OLED_SCL, PIN_OLED_SDA);

struct DroneRec {
  bool     used;
  char     id[21];    // UAS Basic ID, may be empty
  char     mac[18];
  bool     isBle;
  int      rssi;
  bool     hasPilot;
  double   plat, plon;
  bool     hasDrone;
  double   dlat, dlon;
  uint32_t lastSeen;
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

static void upsert(const char *id, const char *mac, int rssi, bool isBle,
                   bool hasD, double dlat, double dlon,
                   bool hasP, double plat, double plon) {
  int slot = -1;

  if (id[0] != '\0') {
    for (int i = 0; i < MAX_DRONES; i++)
      if (drones[i].used && strcmp(drones[i].id, id) == 0) { slot = i; break; }
  }
  if (slot < 0) {
    for (int i = 0; i < MAX_DRONES; i++)
      if (drones[i].used && strcmp(drones[i].mac, mac) == 0) { slot = i; break; }
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
  if (id[0] != '\0') {
    strncpy(d.id, id, sizeof(d.id) - 1);
    d.id[sizeof(d.id) - 1] = '\0';
  }
  strncpy(d.mac, mac, sizeof(d.mac) - 1);
  d.mac[sizeof(d.mac) - 1] = '\0';
  d.isBle = isBle;
  d.rssi = rssi;
  if (hasD) { d.hasDrone = true; d.dlat = dlat; d.dlon = dlon; }
  if (hasP) { d.hasPilot = true; d.plat = plat; d.plon = plon; }
  d.lastSeen = millis();
}

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
  int rssi = doc["rssi"] | 0;
  bool hasD = doc.containsKey("dlat") && doc.containsKey("dlon");
  bool hasP = doc.containsKey("plat") && doc.containsKey("plon");

  upsert(id, macLower, rssi, strcmp(tp, "ble") == 0,
         hasD, doc["dlat"] | 0.0, doc["dlon"] | 0.0,
         hasP, doc["plat"] | 0.0, doc["plon"] | 0.0);
  lastRx = millis();

  int list[MAX_DRONES];
  int m = collectUsed(list);
  Serial.printf("[RX] %s %s rssi=%d id=%s mac=%s (tracked=%d)\n",
                src, tp, rssi, id[0] ? id : "-", macLower, m);
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
    snprintf(line, sizeof(line), "%s  %d dBm", d.isBle ? "BLE" : "WiFi", d.rssi);
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
}

void loop() {
  static char usbBuf[300], linkBuf[300];
  static size_t usbLen = 0, linkLen = 0;

  pumpStream(Serial, "usb", usbBuf, sizeof(usbBuf), usbLen);
  pumpStream(Serial1, "link", linkBuf, sizeof(linkBuf), linkLen);
  handleButton();

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
