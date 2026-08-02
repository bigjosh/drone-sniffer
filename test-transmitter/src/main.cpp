// Bench test transmitter — synthetic Remote ID beacons for validating the sniffer.
//
// Broadcasts TWO independent fake drones, one per transport, so both receiver
// paths can be told apart at a glance:
//   WiFi Beacon (ch 6)  -> BENCHTEST0000001
//   BLE advertisement   -> BENCHTEST0000002
// Both are built with the same OpenDroneID reference library the receiver uses
// to decode. WiFi carries a full message pack per frame; BLE legacy ads fit only
// one 25-byte message each, so the four message types are cycled.
//
// This exists so the receiver can be tested without a real drone. The IDs and
// coordinates below are deliberately synthetic. Bench use only: keep it indoors
// at low power, and don't leave it running — anything nearby that listens for
// Remote ID will report these as aircraft.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>

#include "opendroneid.h"
#include "odid_wifi.h"

static const int      TX_CHANNEL   = 6;     // receiver listens here
static const uint32_t TX_PERIOD_MS = 1000;
static const char    *AP_SSID      = "ODID-BENCH-TX";

// Obviously-synthetic test identities and positions (round numbers, rural farmland).
static const char  *TEST_UAS_ID   = "BENCHTEST0000001";
static const char  *TEST_OP_ID    = "BENCH-TEST-OP";
static const double HOME_LAT      = 45.000000;
static const double HOME_LON      = -93.000000;
static const double PILOT_LAT     = 45.001000;
static const double PILOT_LON     = -93.001000;
static const float  ORBIT_RADIUS_DEG = 0.0005f;  // ~55 m, so the position visibly moves

// Second drone, BLE only — distinct ID and a pilot location a clear step away
// from the WiFi one so the two records are unmistakable on the display.
static const char    *BLE_UAS_ID    = "BENCHTEST0000002";
static const char    *BLE_OP_ID     = "BENCH-TEST-BLE";
static const double   BLE_HOME_LAT  = 45.010000;
static const double   BLE_HOME_LON  = -93.010000;
static const double   BLE_PILOT_LAT = 45.011000;
static const double   BLE_PILOT_LON = -93.011000;
static const uint32_t BLE_ROTATE_MS = 250;   // one message type per step

static ODID_UAS_Data uas;
static uint8_t txMac[6];
static uint8_t sendCounter = 0;
static uint32_t lastTx = 0;
static uint32_t txOk = 0, txFail = 0;

static ODID_UAS_Data bleUas;
static BLEAdvertising *bleAdv = nullptr;
static uint8_t bleMsgCounter = 0;
static int bleMsgIndex = 0;
static uint32_t lastBleRotate = 0;
static uint32_t bleOk = 0, bleFail = 0;

// Shared field values; `which` picks the identity/position set.
static void fillUasData(ODID_UAS_Data *d, const char *uasId, const char *opId,
                        double pilotLat, double pilotLon) {
  odid_initUasData(d);

  d->BasicID[0].IDType = ODID_IDTYPE_SERIAL_NUMBER;
  d->BasicID[0].UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
  strncpy(d->BasicID[0].UASID, uasId, sizeof(d->BasicID[0].UASID) - 1);
  d->BasicIDValid[0] = 1;

  d->Location.Status = ODID_STATUS_AIRBORNE;
  d->Location.SpeedHorizontal = 5.0f;
  d->Location.SpeedVertical = 0.0f;
  d->Location.AltitudeBaro = 150.0f;
  d->Location.AltitudeGeo = 150.0f;
  d->Location.HeightType = ODID_HEIGHT_REF_OVER_TAKEOFF;
  d->Location.Height = 100.0f;
  d->Location.HorizAccuracy = ODID_HOR_ACC_3_METER;
  d->Location.VertAccuracy = ODID_VER_ACC_3_METER;
  d->Location.BaroAccuracy = ODID_VER_ACC_10_METER;
  d->Location.SpeedAccuracy = ODID_SPEED_ACC_1_METERS_PER_SECOND;
  d->Location.TSAccuracy = ODID_TIME_ACC_1_0_SECOND;
  d->LocationValid = 1;

  d->System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
  d->System.ClassificationType = ODID_CLASSIFICATION_TYPE_UNDECLARED;
  d->System.OperatorLatitude = pilotLat;
  d->System.OperatorLongitude = pilotLon;
  d->System.OperatorAltitudeGeo = 130.0f;
  d->System.AreaCount = 1;
  d->System.AreaRadius = 0;
  d->SystemValid = 1;

  d->OperatorID.OperatorIdType = ODID_OPERATOR_ID;
  strncpy(d->OperatorID.OperatorId, opId, sizeof(d->OperatorID.OperatorId) - 1);
  d->OperatorIDValid = 1;
}

static void buildStaticData() {
  fillUasData(&uas, TEST_UAS_ID, TEST_OP_ID, PILOT_LAT, PILOT_LON);
  fillUasData(&bleUas, BLE_UAS_ID, BLE_OP_ID, BLE_PILOT_LAT, BLE_PILOT_LON);
}

// Walk a fake drone around a small circle so successive packets differ.
static void orbit(ODID_UAS_Data *d, double homeLat, double homeLon, float phase) {
  float t = (float)millis() / 30000.0f * 2.0f * PI + phase;  // one lap per 30 s
  d->Location.Latitude = homeLat + ORBIT_RADIUS_DEG * sinf(t);
  d->Location.Longitude = homeLon + ORBIT_RADIUS_DEG * cosf(t);
  d->Location.Direction = fmodf(degrees(t) + 90.0f, 360.0f);
  d->Location.TimeStamp = (float)((millis() / 100) % 36000) / 10.0f;
}

static void updateFlight() {
  orbit(&uas, HOME_LAT, HOME_LON, 0.0f);
  orbit(&bleUas, BLE_HOME_LAT, BLE_HOME_LON, PI);  // opposite side of its circle
}

// ASTM F3411 BLE advertisement: a single Service Data AD structure (UUID 0xFFFA)
// carrying app code 0x0D, a counter, and one 25-byte ODID message. That is
// exactly 31 bytes — the legacy advertisement maximum — so it must be the only
// structure in the payload, with no Flags field ahead of it.
static void bleAdvertiseNext() {
  uint8_t payload[31];
  uint8_t *msg = &payload[6];

  payload[0] = 0x1E;  // length of the rest of this AD structure
  payload[1] = 0x16;  // Service Data - 16-bit UUID
  payload[2] = 0xFA;  // UUID 0xFFFA, little endian
  payload[3] = 0xFF;
  payload[4] = 0x0D;  // OpenDroneID application code
  payload[5] = bleMsgCounter++;
  memset(msg, 0, ODID_MESSAGE_SIZE);

  int rc = ODID_FAIL;
  const char *what = "";
  switch (bleMsgIndex) {
    case 0:
      rc = encodeBasicIDMessage((ODID_BasicID_encoded *)msg, &bleUas.BasicID[0]);
      what = "BasicID";
      break;
    case 1:
      rc = encodeLocationMessage((ODID_Location_encoded *)msg, &bleUas.Location);
      what = "Location";
      break;
    case 2:
      rc = encodeSystemMessage((ODID_System_encoded *)msg, &bleUas.System);
      what = "System";
      break;
    default:
      rc = encodeOperatorIDMessage((ODID_OperatorID_encoded *)msg, &bleUas.OperatorID);
      what = "OperatorID";
      break;
  }
  bleMsgIndex = (bleMsgIndex + 1) & 3;

  if (rc != ODID_SUCCESS) {
    bleFail++;
    Serial.printf("[ERR] BLE encode %s failed\n", what);
    return;
  }

  // Maps to the stack's raw-payload call, so these 31 bytes go out verbatim
  // with no Flags structure prepended.
  BLEAdvertisementData advData;
  advData.addData((char *)payload, sizeof(payload));
  if (bleAdv->setAdvertisementData(advData)) {
    bleOk++;
  } else {
    bleFail++;
    Serial.printf("[ERR] BLE set adv data failed (%s)\n", what);
  }
}

static void bleStart() {
  BLEDevice::init("");  // brings up the controller and host stack
  bleAdv = BLEDevice::getAdvertising();

  // Non-connectable + no scan response => ADV_NONCONN_IND, per ASTM F3411.
#if defined(CONFIG_NIMBLE_ENABLED)
  bleAdv->setAdvertisementType(BLE_GAP_CONN_MODE_NON);
#else
  bleAdv->setAdvertisementType(ADV_TYPE_NONCONN_IND);
#endif
  bleAdv->setScanResponse(false);
  bleAdv->setMinInterval(0x20);  // 20 ms
  bleAdv->setMaxInterval(0x40);  // 40 ms

  bleAdvertiseNext();  // load the first payload before advertising starts
  bool started = bleAdv->start();

  Serial.printf("[INFO] BLE advertising started: %s, addr %s\n",
                started ? "yes" : "NO", BLEDevice::getAddress().toString().c_str());
  Serial.printf("[INFO] BLE uas id \"%s\", operator \"%s\"\n", BLE_UAS_ID, BLE_OP_ID);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[BOOT] OpenDroneID bench transmitter (synthetic test drone)");

  buildStaticData();

  WiFi.mode(WIFI_AP);
  // Hidden AP purely to own the radio on a fixed channel; the payload we care
  // about is the injected beacon below, not this SSID.
  WiFi.softAP(AP_SSID, nullptr, TX_CHANNEL, 1 /* hidden */, 1 /* max conn */);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  esp_wifi_set_channel(TX_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_get_mac(WIFI_IF_AP, txMac);

  Serial.printf("[INFO] channel %d, tx mac %02x:%02x:%02x:%02x:%02x:%02x\n",
                TX_CHANNEL, txMac[0], txMac[1], txMac[2], txMac[3], txMac[4], txMac[5]);
  Serial.printf("[INFO] WiFi uas id \"%s\", operator \"%s\"\n", TEST_UAS_ID, TEST_OP_ID);

  bleStart();
}

void loop() {
  uint32_t now = millis();

  if (now - lastBleRotate >= BLE_ROTATE_MS) {
    lastBleRotate = now;
    orbit(&bleUas, BLE_HOME_LAT, BLE_HOME_LON, PI);
    bleAdvertiseNext();
  }

  if (now - lastTx < TX_PERIOD_MS) {
    delay(5);
    return;
  }
  lastTx = now;

  updateFlight();

  uint8_t frame[512];
  int len = odid_wifi_build_message_pack_beacon_frame(
      &uas, (char *)txMac, AP_SSID, strlen(AP_SSID), 0x64 /* 100 TU */,
      sendCounter++, frame, sizeof(frame));
  if (len < 0) {
    Serial.printf("[ERR] frame build failed: %d\n", len);
    return;
  }

  esp_err_t err = esp_wifi_80211_tx(WIFI_IF_AP, frame, len, true);
  if (err == ESP_OK) {
    txOk++;
  } else {
    txFail++;
    Serial.printf("[ERR] tx failed: %s\n", esp_err_to_name(err));
  }

  if (txOk % 10 == 1 || txFail || bleFail) {
    Serial.printf("[TX] wifi ok=%lu fail=%lu len=%d lat=%.6f lon=%.6f | ble ok=%lu fail=%lu\n",
                  (unsigned long)txOk, (unsigned long)txFail, len,
                  uas.Location.Latitude, uas.Location.Longitude,
                  (unsigned long)bleOk, (unsigned long)bleFail);
  }
}
