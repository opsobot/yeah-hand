/*
This file is part of Rebelia-Robotic-Firmware (Yeah Hand).
Copyright (C) 2026 Vittorio Lumare
License: GNU GPL v3 or later (see COPYING.txt).

Access control for RED Art. 3.3(d) / EN 18031-1.

Provisioning (manufacturer, wired USB, once per device):
  FACTORY PROVISION <serial> <password>   accepted only while no password is stored
  FACTORY STATUS                          serial, Bluetooth name, provisioned yes/no (never the password)
  No command can read the password back. A lost password is recovered from the
  manufacturer's records, or by erasing the flash and provisioning again.

Until the device is provisioned, Bluetooth is not started and no command is accepted.

Every command channel (Bluetooth SPP and USB serial) has its own session:
  only "AUTH <password>" is accepted until the session is authenticated.
Bluetooth:
  - SPP server restarted with authentication + encryption required
    (BluetoothSerial 3.3.x starts it with ESP_SPP_SEC_NONE)
  - new pairings accepted only inside a pairing window
  - session closed when the link drops
Brute-force protection: increasing lockout after repeated failures (shared by all channels).
*/

#pragma once

#include <Arduino.h>
#include "BluetoothSerial.h"

namespace HandSecurity {

// Pairing window length after power-on or after the PAIRING command.
constexpr uint32_t PAIRING_WINDOW_MS = 120000;

struct Session {
  Stream *out;
  volatile bool authenticated;
};

// Load serial number and password from NVS. Call first in setup().
void loadProvisioning();

bool isProvisioned();
String serialNumber();

// Bluetooth name: "YeahHand-<serial>" (or derived from the MAC if not provisioned).
String deviceName();

// Must be called BEFORE SerialBT.begin().
void preBeginBluetooth(BluetoothSerial &bt);

// Call every loop iteration. Manages the pairing window.
void loop();

Session &bluetoothSession();
Session &usbSession();

// True once after the Bluetooth link was closed (then resets).
bool takeBluetoothLinkClosedEvent();

// Handles AUTH, LOGOUT, SETPASS, PAIRING, UNPAIR ALL. Returns true if the line was one of them.
bool handleCommand(const String &line, Session &session);

// Handles FACTORY ... commands. Call only for the wired USB serial port. Returns true if handled.
bool handleFactoryCommand(const String &line, Stream &out);

}  // namespace HandSecurity
