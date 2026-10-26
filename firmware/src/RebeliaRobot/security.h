/*
This file is part of Rebelia-Robotic-Firmware (Yeah Hand).
Copyright (C) 2026 Vittorio Lumare
License: GNU GPL v3 or later (see COPYING.txt).

Bluetooth access control for RED Art. 3.3(d) / EN 18031-1:
  - SPP server restarted with authentication + encryption required
    (BluetoothSerial 3.3.x starts it with ESP_SPP_SEC_NONE)
  - new pairings accepted only inside a pairing window
  - per-device password, required by the AUTH command before any other command
  - brute-force protection with increasing lockout
  - session closed when the Bluetooth link drops
*/

#pragma once

#include <Arduino.h>
#include "BluetoothSerial.h"

namespace HandSecurity {

// Pairing window length after power-on or after the PAIRING command.
constexpr uint32_t PAIRING_WINDOW_MS = 120000;

// Must be called BEFORE SerialBT.begin().
void preBegin(BluetoothSerial &bt);

// Must be called AFTER SerialBT.begin(). Loads or creates the device password.
void begin();

// Call every loop iteration. Manages the pairing window.
void loop();

// Unique Bluetooth name, derived from the Bluetooth MAC address.
String deviceName();

// True when the connected peer has sent a valid AUTH command in this session.
bool isAuthenticated();

// True once after the Bluetooth link was closed (then resets).
bool takeLinkClosedEvent();

// Handles AUTH, LOGOUT, SETPASS, PAIRING, UNPAIR. Returns true if the line was one of them.
bool handleCommand(const String &line, Stream &out);

// Factory commands on the wired USB serial port only (physical access).
void handleUsbCommand(const String &line, Stream &out);

}  // namespace HandSecurity
