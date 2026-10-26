/*
This file is part of Rebelia-Robotic-Firmware (Yeah Hand).
Copyright (C) 2026 Vittorio Lumare
License: GNU GPL v3 or later (see COPYING.txt).
*/

#include "security.h"

#include <Preferences.h>
#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>
#include <esp_mac.h>

namespace HandSecurity {

namespace {

constexpr const char *NVS_NAMESPACE = "yhsec";
constexpr const char *NVS_KEY_PASSWORD = "pw";
constexpr const char *NVS_KEY_SERIAL = "sn";
constexpr size_t MIN_PASSWORD_LEN = 10;
constexpr size_t MAX_PASSWORD_LEN = 32;
constexpr size_t MAX_SERIAL_LEN = 16;
constexpr uint8_t FREE_ATTEMPTS = 5;         // failed attempts before lockout
constexpr uint32_t BASE_LOCKOUT_MS = 30000;  // first lockout: 30 s, then doubled
constexpr uint32_t MAX_LOCKOUT_MS = 900000;  // capped at 15 min
constexpr int MAX_TRUSTED_PEERS = 8;

BluetoothSerial *s_bt = nullptr;
Preferences s_prefs;
String s_password;
String s_serial;

Session s_btSession{ nullptr, false };
Session s_usbSession{ &Serial, false };

volatile bool s_btLinkClosed = false;
volatile bool s_serverSecured = false;
volatile bool s_pairingOpen = false;
uint32_t s_pairingOpenedAt = 0;

uint8_t s_failedAttempts = 0;
uint32_t s_lockedUntil = 0;

// Peers bonded when the pairing window closed. Outside the window only these may connect.
esp_bd_addr_t s_trustedPeers[MAX_TRUSTED_PEERS];
volatile int s_trustedCount = 0;

// Comparison time does not depend on where the strings differ.
bool constantTimeEquals(const String &a, const String &b) {
  size_t len = max(a.length(), b.length());
  uint8_t diff = a.length() == b.length() ? 0 : 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t ca = i < a.length() ? a[i] : 0;
    uint8_t cb = i < b.length() ? b[i] : 0;
    diff |= ca ^ cb;
  }
  return diff == 0;
}

bool isValidPassword(const String &pw) {
  if (pw.length() < MIN_PASSWORD_LEN || pw.length() > MAX_PASSWORD_LEN) return false;
  for (size_t i = 0; i < pw.length(); i++) {
    char c = pw[i];
    if (c <= ' ' || c > '~') return false;  // printable ASCII, no spaces
  }
  return true;
}

bool isValidSerial(const String &sn) {
  if (sn.length() == 0 || sn.length() > MAX_SERIAL_LEN) return false;
  for (size_t i = 0; i < sn.length(); i++) {
    char c = sn[i];
    if (!isAlphaNumeric(c) && c != '-') return false;
  }
  return true;
}

void snapshotTrustedPeers() {
  int n = MAX_TRUSTED_PEERS;
  if (esp_bt_gap_get_bond_device_list(&n, s_trustedPeers) != ESP_OK) n = 0;
  s_trustedCount = n;
}

bool isTrustedPeer(const esp_bd_addr_t addr) {
  for (int i = 0; i < s_trustedCount; i++) {
    if (memcmp(s_trustedPeers[i], addr, sizeof(esp_bd_addr_t)) == 0) return true;
  }
  return false;
}

void openPairingWindow() {
  s_pairingOpen = true;
  s_pairingOpenedAt = millis();
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
}

void closePairingWindow() {
  s_pairingOpen = false;
  snapshotTrustedPeers();
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
}

uint32_t lockoutRemainingMs() {
  if (s_lockedUntil == 0) return 0;
  int32_t remaining = (int32_t)(s_lockedUntil - millis());
  return remaining > 0 ? (uint32_t)remaining : 0;
}

void registerFailure() {
  s_failedAttempts++;
  if (s_failedAttempts >= FREE_ATTEMPTS) {
    uint32_t shift = s_failedAttempts - FREE_ATTEMPTS;
    uint32_t lock = shift >= 5 ? MAX_LOCKOUT_MS : min(MAX_LOCKOUT_MS, BASE_LOCKOUT_MS << shift);
    s_lockedUntil = millis() + lock;
  }
}

// Runs in the Bluetooth task, after the BluetoothSerial library has handled the event.
void sppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
    case ESP_SPP_START_EVT:
      // The library starts the server with ESP_SPP_SEC_NONE: restart it requiring
      // authentication (pairing) and encryption. The second START event is ignored.
      if (!s_serverSecured && param->start.status == ESP_SPP_SUCCESS) {
        s_serverSecured = true;
        esp_spp_stop_srv_scn(param->start.scn);
        esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE | ESP_SPP_SEC_ENCRYPT, ESP_SPP_ROLE_SLAVE, 0, "ESP32SPP");
      }
      break;

    case ESP_SPP_SRV_OPEN_EVT:
      s_btSession.authenticated = false;  // every connection starts unauthenticated
      if (!s_pairingOpen && !isTrustedPeer(param->srv_open.rem_bda)) {
        // Device bonded outside the pairing window: refuse it and forget the bond.
        esp_spp_disconnect(param->srv_open.handle);
        esp_bt_gap_remove_bond_device(param->srv_open.rem_bda);
      }
      break;

    case ESP_SPP_CLOSE_EVT:
      s_btSession.authenticated = false;
      s_btLinkClosed = true;
      break;

    default:
      break;
  }
}

}  // namespace

void loadProvisioning() {
  s_prefs.begin(NVS_NAMESPACE, false);
  s_password = s_prefs.getString(NVS_KEY_PASSWORD, "");
  s_serial = s_prefs.getString(NVS_KEY_SERIAL, "");
}

bool isProvisioned() {
  return s_password.length() > 0;
}

String serialNumber() {
  return s_serial;
}

String deviceName() {
  if (s_serial.length() > 0) return "YeahHand-" + s_serial;
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  char name[20];
  snprintf(name, sizeof(name), "YeahHand-%02X%02X", mac[4], mac[5]);
  return String(name);
}

void preBeginBluetooth(BluetoothSerial &bt) {
  s_bt = &bt;
  s_btSession.out = &bt;
  bt.enableSSP(false, false);  // Secure Simple Pairing, no display / no keyboard on the hand
  bt.onConfirmRequest([](uint32_t) {
    // Accept a new pairing only while the pairing window is open.
    s_bt->confirmReply(s_pairingOpen);
  });
  bt.register_callback(sppCallback);
  s_pairingOpen = true;  // window opens at power-on
  s_pairingOpenedAt = millis();
}

void loop() {
  if (s_bt && s_pairingOpen && millis() - s_pairingOpenedAt > PAIRING_WINDOW_MS) {
    closePairingWindow();
  }
}

Session &bluetoothSession() {
  return s_btSession;
}

Session &usbSession() {
  return s_usbSession;
}

bool takeBluetoothLinkClosedEvent() {
  if (s_btLinkClosed) {
    s_btLinkClosed = false;
    return true;
  }
  return false;
}

bool handleCommand(const String &line, Session &session) {
  Stream &out = *session.out;

  if (line.startsWith("AUTH ")) {
    if (!isProvisioned()) {
      out.println("ERR NOT PROVISIONED");
      return true;
    }
    uint32_t wait = lockoutRemainingMs();
    if (wait > 0) {
      out.printf("ERR LOCKED %lu s\n", (unsigned long)((wait + 999) / 1000));
      return true;
    }
    if (constantTimeEquals(line.substring(5), s_password)) {
      session.authenticated = true;
      s_failedAttempts = 0;
      s_lockedUntil = 0;
      out.println("OK AUTH");
    } else {
      session.authenticated = false;
      registerFailure();
      out.println("ERR AUTH");
    }
    return true;
  }

  if (line == "LOGOUT") {
    session.authenticated = false;
    out.println("OK LOGOUT");
    return true;
  }

  if (line.startsWith("SETPASS ")) {
    if (!session.authenticated) {
      out.println("ERR AUTH REQUIRED");
      return true;
    }
    String args = line.substring(8);
    int sep = args.indexOf(' ');
    if (sep < 0) {
      out.println("ERR USAGE: SETPASS <old> <new>");
      return true;
    }
    String oldPw = args.substring(0, sep);
    String newPw = args.substring(sep + 1);
    if (!constantTimeEquals(oldPw, s_password)) {
      registerFailure();
      out.println("ERR AUTH");
    } else if (!isValidPassword(newPw)) {
      out.printf("ERR PASSWORD: %u-%u printable characters, no spaces\n", (unsigned)MIN_PASSWORD_LEN, (unsigned)MAX_PASSWORD_LEN);
    } else {
      s_prefs.putString(NVS_KEY_PASSWORD, newPw);
      s_password = newPw;
      out.println("OK SETPASS");
    }
    return true;
  }

  if (line == "PAIRING") {
    if (!session.authenticated) {
      out.println("ERR AUTH REQUIRED");
    } else if (!s_bt) {
      out.println("ERR BLUETOOTH OFF");
    } else {
      openPairingWindow();
      out.printf("OK PAIRING %lu s\n", (unsigned long)(PAIRING_WINDOW_MS / 1000));
    }
    return true;
  }

  if (line == "UNPAIR ALL") {
    if (!session.authenticated) {
      out.println("ERR AUTH REQUIRED");
    } else if (!s_bt) {
      out.println("ERR BLUETOOTH OFF");
    } else {
      out.println("OK UNPAIR ALL");
      s_trustedCount = 0;
      s_bt->deleteAllBondedDevices();
    }
    return true;
  }

  return false;
}

bool handleFactoryCommand(const String &line, Stream &out) {
  if (!line.startsWith("FACTORY")) return false;

  if (line == "FACTORY STATUS") {
    out.printf("FACTORY STATUS provisioned=%s serial=%s name=%s\n", isProvisioned() ? "yes" : "no",
               s_serial.length() ? s_serial.c_str() : "-", deviceName().c_str());
    return true;
  }

  if (line.startsWith("FACTORY PROVISION ")) {
    if (isProvisioned()) {
      out.println("ERR ALREADY PROVISIONED (erase the flash to provision again)");
      return true;
    }
    String args = line.substring(18);
    int sep = args.indexOf(' ');
    if (sep < 0) {
      out.println("ERR USAGE: FACTORY PROVISION <serial> <password>");
      return true;
    }
    String sn = args.substring(0, sep);
    String pw = args.substring(sep + 1);
    if (!isValidSerial(sn)) {
      out.printf("ERR SERIAL: 1-%u characters A-Z a-z 0-9 -\n", (unsigned)MAX_SERIAL_LEN);
      return true;
    }
    if (!isValidPassword(pw)) {
      out.printf("ERR PASSWORD: %u-%u printable characters, no spaces\n", (unsigned)MIN_PASSWORD_LEN, (unsigned)MAX_PASSWORD_LEN);
      return true;
    }
    s_prefs.putString(NVS_KEY_SERIAL, sn);
    s_prefs.putString(NVS_KEY_PASSWORD, pw);
    s_serial = sn;
    s_password = pw;
    out.printf("OK PROVISIONED serial=%s name=%s, restarting\n", sn.c_str(), deviceName().c_str());
    out.flush();
    delay(200);
    ESP.restart();
    return true;
  }

  out.println("ERR FACTORY: FACTORY STATUS | FACTORY PROVISION <serial> <password>");
  return true;
}

}  // namespace HandSecurity
