#pragma once
#include <stdint.h>
#include <string.h>
#include <string>

#define WIFI_CHANNEL  1          // must match the Hub's softAP channel
#define COMPANY_ID    0xFFFF     // test company ID, fine for a prototype
#define TRACKER_MAGIC 0xF7       // marks a broadcast as coming from one of our Trackers

#define FLAG_MOVING  0x01
#define FLAG_FAULT   0x02
#define FLAG_MANDOWN 0x04
#define FLAG_HEAT    0x08

// ---------- Tracker status (inside the Tracker's BLE broadcast) ----------
struct __attribute__((packed)) TrackerStatus {
  uint8_t  magic;        // always TRACKER_MAGIC
  uint8_t  trackerId;
  uint8_t  seq;          // +1 every update
  uint8_t  battery;      // %
  uint8_t  flags;        // FLAG_* bits
  int16_t  altDm;        // altitude above start point, decimetres
  int16_t  tempC10;      // temperature, tenths of °C
  uint16_t pressHpa10;   // pressure, tenths of hPa
  uint16_t heading;      // degrees 0–359, 0 = magnetic north
  uint16_t steps;        // steps since boot
  int16_t  pdrXdm;       // distance walked east since boot, decimetres
  int16_t  pdrYdm;       // distance walked north since boot, decimetres
};
// Broadcast budget: 31 bytes - 3 (flags) - 4 (header + company ID) = 24 max
static_assert(sizeof(TrackerStatus) == 19, "TrackerStatus size changed");
static_assert(sizeof(TrackerStatus) <= 24, "TrackerStatus too big for a BLE broadcast");

// Returns true if this manufacturer data is a valid Tracker broadcast
inline bool parseTrackerAdvert(const std::string& m, TrackerStatus& s) {
  if (m.size() != 2 + sizeof(TrackerStatus)) return false;
  if ((uint8_t)m[0] != (COMPANY_ID & 0xFF) || (uint8_t)m[1] != (COMPANY_ID >> 8)) return false;
  memcpy(&s, m.data() + 2, sizeof(s));
  return s.magic == TRACKER_MAGIC;
}

// ---------- Anchor -> Hub report (ESP-NOW) ----------
struct __attribute__((packed)) AnchorReport {
  uint8_t anchorId;
  int8_t  rssi;          // averaged dBm
  uint8_t samples;
  TrackerStatus status;
};

// ---------- Node beacons (Hub/Anchors announce themselves to Trackers) ----------
#define NODE_SERVICE_UUID "5f524546-4c45-5f42-4c45-5f4e4f444500"
#define NODE_HUB    0
#define NODE_ANCHOR 1

struct __attribute__((packed)) NodeBeacon {
  uint8_t nodeType;      // NODE_HUB or NODE_ANCHOR
  uint8_t nodeId;        // 0 for Hub, 1–3 for Anchors
};