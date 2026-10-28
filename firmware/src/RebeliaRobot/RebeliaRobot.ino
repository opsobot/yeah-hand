/*
Rebelia-Robotic-Firmware is the control software for the Rebelia Robotic Hand, a robotic end-effector device (see https://www.robotgarage.org).

The Copyright Notice
Copyright (C)  2023 Vittorio Lumare

The License Notices
    This file is part of Rebelia-Robotic-Firmware.

    Rebelia-Robotic-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

    Rebelia-Robotic-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with Rebelia-Hand-Firmware. If not, see <https://www.gnu.org/licenses/>.
*/

/*
Build environment used for the CE-assessed firmware:
  Arduino IDE, board package "esp32" by Espressif Systems 3.3.12 (do not update without re-testing).

Command channels: Bluetooth Classic SPP and USB serial (115200), one command per line.
  Each channel has its own session: until "AUTH <password>" succeeds no other command is accepted.
  After power-on the hand does not move until it receives CALIBRATE (no automatic restart).
  Normal commands:   CALIBRATE | POWER | BAR | MONKEY | PINCH1 | PINCH2 | <0..100> | ROS a b c d e
                     FI|FM|FR|FT|RT <0..100> | INFO <1..5> | POS | CONTROL
                     LOGOUT | SETPASS <old> <new> | PAIRING | UNPAIR ALL | SERVICE ON | HELP
  Service commands (only after SERVICE ON):
                     INSTALL | TEST | CALIBRATE | SETCENTER <1..5> | SETPINCH <-40..40>
                     LIM <1..5> <0..1000> | MOVE <1..5> <0..4095> | SUPPLY [<preset>] | SERVICE OFF
  Power presets (SUPPLY): torque limits that keep the supply current within a budget,
  stored in the hand and applied at start-up. Built in: 5A (default), 1A.
  Every other line is rejected with "ERR" and causes no motion.
Factory provisioning, USB serial only: FACTORY STATUS | FACTORY PROVISION <serial> <L|R> <password>
  (once per device; the password cannot be read back. Until provisioned, Bluetooth stays off.)
*/

#include <SCServo.h>
#include <BluetoothSerial.h>
#include "fingers_controller.h"
#include "security.h"
#include <Preferences.h>

#define S_RXD 18
#define S_TXD 19
#define DEBUG false

static const size_t MAX_LINE_LEN = 64;

BluetoothSerial SerialBT;
SMS_STS st;
FingersController fc(&SerialBT);

void servoIdDiscovery() {
  SerialBT.println("Servo ID Discovery");
  for (int i = 0; i <= 5; i++) {
    int ID = st.Ping(i);
    if (ID != -1) {
      SerialBT.print("Servo ID:");
      SerialBT.println(ID, DEC);
      delay(100);
    } else {
      SerialBT.print("Servo ID:");
      SerialBT.print(ID, DEC);
      SerialBT.println(" ERROR!");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  HandSecurity::loadProvisioning();
  if (HandSecurity::isProvisioned()) {
    // Bluetooth: Wi-Fi is never initialised; BLE is released (unused interface).
    HandSecurity::preBeginBluetooth(SerialBT);
    SerialBT.begin(HandSecurity::deviceName(), false, true);
    SerialBT.setTimeout(50);
    Serial.printf("%s started.\n", HandSecurity::deviceName().c_str());
  } else {
    // Not provisioned: no radio, no calibration, no motion. Only FACTORY commands
    // on USB are served (see loop), so provisioning works even without 12 V.
    Serial.println("Bluetooth off.");
    printProvisioningHelp(Serial);
    return;
  }

  // Hand side from the model stored at provisioning (L = left, R = right)
  if (HandSecurity::model() == "R") {
    fc.setHandSide(FingersController::HandSide::RightSide);
  } else {
    fc.setHandSide(FingersController::HandSide::LeftSide);
    if (HandSecurity::model() != "L") Serial.println("WARNING: model not provisioned, assuming left hand (L)");
  }

  // Power preset (torque limits for the available supply current)
  String preset = loadPowerPreset();
  if (!fc.setPowerPreset(preset)) {
    Serial.printf("WARNING: unknown power preset %s, using 5A\n", preset.c_str());
    fc.setPowerPreset("5A");
  }
  Serial.printf("Power preset: %s\n", fc.powerPresetName());

  // No movement at power-on (Machinery Directive Annex I 1.2.3 / 1.2.6): the hand
  // stays still until it receives CALIBRATE from an authenticated client.
  fc.setOutput(&Serial);
  Serial.println("Hand ready, NOT CALIBRATED: send AUTH <password>, then CALIBRATE.");
}

FingersController::GraspType g_grasp_type = FingersController::GraspType::MONKEY;
FingersController::GraspType g_new_grasp_type = g_grasp_type;
int g_closurePercent = 0;
int g_newClosurePercent = 0;
bool g_preparation = true;
bool g_closure = false;
bool g_serviceMode = false;
bool g_calibrated = false;  // false after every power-on: no motion until CALIBRATE

// Deliberate calibration, the only way the hand starts moving after power-on.
void doCalibrate() {
  reply("OK CALIBRATE: calibrating, keep clear of the fingers");
  fc.calibrateHand();
  g_calibrated = true;
  g_preparation = true;  // then move to the selected grasp's start position
  reply("OK CALIBRATED");
}

// Motion commands are refused until the hand is calibrated.
bool refuseIfNotCalibrated() {
  if (g_calibrated) return false;
  reply("ERR NOT CALIBRATED: send CALIBRATE first");
  return true;
}
enum class FSM {
  Control,
  TendonInstall,
  Testing,
  DoNothing
};
FSM gFsmState = FSM::Control;

// ---------- Strict argument parsing ----------

// Whole string must be an integer within [minV, maxV].
bool parseIntStrict(const String &s, int minV, int maxV, int &value) {
  if (s.length() == 0 || s.length() > 6) return false;
  size_t i = 0;
  if (s[0] == '-' || s[0] == '+') {
    if (s.length() == 1) return false;
    i = 1;
  }
  for (; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  long v = s.toInt();
  if (v < minV || v > maxV) return false;
  value = (int)v;
  return true;
}

// "<prefix><int>" with the int in range.
bool parseOneArg(const String &line, const char *prefix, int minV, int maxV, int &value) {
  if (!line.startsWith(prefix)) return false;
  return parseIntStrict(line.substring(strlen(prefix)), minV, maxV, value);
}

// "<prefix><int> <int>" with both ints in range.
bool parseTwoArgs(const String &line, const char *prefix, int min1, int max1, int min2, int max2, int &a, int &b) {
  if (!line.startsWith(prefix)) return false;
  String args = line.substring(strlen(prefix));
  int sep = args.indexOf(' ');
  if (sep < 0) return false;
  return parseIntStrict(args.substring(0, sep), min1, max1, a) && parseIntStrict(args.substring(sep + 1), min2, max2, b);
}

Stream *g_out = &Serial;  // channel of the command being executed

void reply(const char *msg) {
  g_out->println(msg);
}

// ---------- Commands ----------

void moveFinger(FingersController::VectorIdx idx, int factor) {
  fc.moveUntilLoadLimitHit(idx, factor, 3073, 200);
}

// Power preset stored in NVS (namespace "yhcfg"), applied before calibration.
String loadPowerPreset() {
  Preferences cfg;
  cfg.begin("yhcfg", true);
  String name = cfg.getString("power", "5A");
  cfg.end();
  return name;
}

void savePowerPreset(const String &name) {
  Preferences cfg;
  cfg.begin("yhcfg", false);
  cfg.putString("power", name);
  cfg.end();
}

void printPowerPresets() {
  const SafetyLimits &l = fc.safetyLimits();
  g_out->printf("SUPPLY %s: MAX_LOAD=%d TF_MAX_LOAD=%d OVER_CURRENT=%d OVER_LOAD=%d OVER_TEMP=%d\n", fc.powerPresetName(),
                l.MAX_LOAD, l.TF_MAX_LOAD, l.OVER_CURRENT, l.OVER_LOAD, l.OVER_TEMP);
  g_out->print("Available presets:");
  for (int i = 0; i < FingersController::POWER_PRESETS_COUNT; i++) g_out->printf(" %s", FingersController::POWER_PRESETS[i].name);
  g_out->println();
}

bool processServiceCommand(const String &cmd) {
  int a, b;
  if (cmd == "SERVICE OFF") {
    g_serviceMode = false;
    gFsmState = FSM::Control;
    reply("OK SERVICE OFF");
  } else if (cmd == "INSTALL") {
    gFsmState = FSM::TendonInstall;
    reply("OK INSTALL");
  } else if (cmd == "TEST") {
    gFsmState = FSM::Testing;
    reply("OK TEST");
  } else if (cmd == "CALIBRATE") {
    doCalibrate();
  } else if (parseOneArg(cmd, "SETCENTER ", 1, 5, a)) {
    fc.setCenterOfRange(a);
  } else if (cmd == "SUPPLY") {
    printPowerPresets();
  } else if (cmd.startsWith("SUPPLY ")) {
    String name = cmd.substring(7);
    if (!FingersController::findPowerPreset(name)) {
      reply("ERR SUPPLY: unknown preset");
      printPowerPresets();
    } else {
      savePowerPreset(name);
      g_out->printf("OK SUPPLY %s, restarting to apply\n", name.c_str());
      g_out->flush();
      delay(200);
      ESP.restart();
    }
  } else if (parseOneArg(cmd, "SETPINCH ", -40, 40, a)) {
    fc.setPinchOffset(a);
  } else if (parseTwoArgs(cmd, "LIM ", 1, 5, 0, 1000, a, b)) {
    fc.setMaxTorque(a, b);
    reply("OK LIM");
  } else if (parseTwoArgs(cmd, "MOVE ", 1, 5, 0, 4095, a, b)) {
    fc.moveUntilLoadLimitHit(a, b, 2000, 200);
    reply("OK MOVE");
  } else {
    return false;
  }
  return true;
}

void processStringCmd(const String &cmd, HandSecurity::Session &session) {
  g_out = session.out;
  fc.setOutput(session.out);

  // Authentication commands are always evaluated first.
  if (HandSecurity::handleCommand(cmd, session)) {
    if (cmd.startsWith("AUTH ") && session.authenticated && !g_calibrated) {
      reply("INFO NOT CALIBRATED: the hand will not move until you send CALIBRATE");
    }
    return;
  }

  if (!session.authenticated) {
    reply("ERR AUTH REQUIRED");
    return;
  }

  int a;

  // Calibration on request (also allowed outside service mode)
  if (cmd == "CALIBRATE") {
    doCalibrate();
    return;
  }

  // Grasp selection (exact match)
  FingersController::GraspType grasp = FingersController::getGraspTypeByString(cmd);
  if (grasp != FingersController::GraspType::_MAX && cmd == FingersController::getGraspStringByType(grasp)) {
    if (refuseIfNotCalibrated()) return;
    g_new_grasp_type = grasp;
    reply("OK GRASP");
    return;
  }

  // Closure percentage: the whole line must be an integer 0..100
  if (parseIntStrict(cmd, 0, 100, a)) {
    if (refuseIfNotCalibrated()) return;
    g_newClosurePercent = a;
    g_out->printf("OK CLOSURE %d\n", a);
    return;
  }

  // ROS a b c d e: all five values validated before any motion
  if (cmd.startsWith("ROS ")) {
    int v[5];
    int consumed = 0;
    int n = sscanf(cmd.c_str(), "ROS %d %d %d %d %d%n", &v[0], &v[1], &v[2], &v[3], &v[4], &consumed);
    bool valid = (n == 5) && ((size_t)consumed == cmd.length());
    for (int i = 0; valid && i < 5; ++i) {
      if (v[i] < 0 || v[i] > 100) valid = false;
    }
    if (!valid) {
      reply("ERR ROS");
      return;
    }
    if (refuseIfNotCalibrated()) return;
    const u8 IDN = 5;
    FingersController::VectorIdx IDXs[IDN] = { FingersController::VectorIdx::Index, FingersController::VectorIdx::Middle, FingersController::VectorIdx::Ring, FingersController::VectorIdx::Thumb, FingersController::VectorIdx::ThumbRot };
    u8 Factor[IDN] = { (u8)v[0], (u8)v[1], (u8)v[2], (u8)v[3], (u8)v[4] };
    u16 Speed[IDN] = { 3000, 3000, 3000, 3000, 3000 };
    u8 Acc[IDN] = { 250, 250, 250, 250, 250 };
    fc.moveUntilLoadLimitHit(IDN, IDXs, Factor, Speed, Acc);  // sync
    return;
  }

  // Single finger moves (factor 0..100)
  if (parseOneArg(cmd, "FI ", 0, 100, a)) { if (refuseIfNotCalibrated()) return; moveFinger(FingersController::VectorIdx::Index, a); return; }
  if (parseOneArg(cmd, "FM ", 0, 100, a)) { if (refuseIfNotCalibrated()) return; moveFinger(FingersController::VectorIdx::Middle, a); return; }
  if (parseOneArg(cmd, "FR ", 0, 100, a)) { if (refuseIfNotCalibrated()) return; moveFinger(FingersController::VectorIdx::Ring, a); return; }
  if (parseOneArg(cmd, "FT ", 0, 100, a)) { if (refuseIfNotCalibrated()) return; moveFinger(FingersController::VectorIdx::Thumb, a); return; }
  if (parseOneArg(cmd, "RT ", 0, 100, a)) { if (refuseIfNotCalibrated()) return; moveFinger(FingersController::VectorIdx::ThumbRot, a); return; }

  // Read-only commands
  if (parseOneArg(cmd, "INFO ", 1, 5, a)) {
    int IDX = fc.getVectorIndexByMotorID(a);
    if (IDX < 0 || IDX >= FingersController::SERVOS_SIZE) {
      g_out->printf("ERR ID %d\n", a);
      return;
    }
    fc.printFeedback(a);
    g_out->printf("Factor: %d\n", fc.getFactorFromPos(IDX, fc.readPos(a)));
    return;
  }
  if (cmd == "POS") {
    u8 IDN = 4;
    u8 IDs[IDN] = { 1, 2, 3, 4 };
    s16 pos[IDN];
    fc.readPositions(IDN, IDs, pos);
    g_out->printf("Positions: %d %d %d %d\n", pos[0], pos[1], pos[2], pos[3]);
    return;
  }
  if (cmd == "CONTROL") {
    gFsmState = FSM::Control;
    reply("OK CONTROL");
    return;
  }
  if (cmd == "HELP") {
    reply("Commands: CALIBRATE, POWER|BAR|MONKEY|PINCH1|PINCH2, 0..100, ROS a b c d e, FI|FM|FR|FT|RT n, INFO id, POS, CONTROL,");
    reply("LOGOUT, SETPASS old new, PAIRING, UNPAIR ALL, SERVICE ON");
    return;
  }

  // Service mode: calibration, tendon installation, raw positions, torque limits
  if (cmd == "SERVICE ON") {
    g_serviceMode = true;
    reply("OK SERVICE ON");
    return;
  }
  if (g_serviceMode && processServiceCommand(cmd)) return;

  reply(g_serviceMode ? "ERR UNKNOWN" : "ERR UNKNOWN (service commands need SERVICE ON)");
}

void prepareGrasp() {

  if (g_grasp_type < FingersController::GraspType::_MAX) {
    fc.prepareGrasp(g_grasp_type);
  }
  g_preparation = false;
}

void sendRosFeedback(Stream &out) {
  u8 factors[FingersController::SERVOS_SIZE];
  s16 load[FingersController::SERVOS_SIZE];
  u8 volt[FingersController::SERVOS_SIZE];
  u8 temp[FingersController::SERVOS_SIZE];
  s16 ampr[FingersController::SERVOS_SIZE];
  fc.readFactors(factors);
  fc.readFeedback(load, volt, temp, ampr);
  out.printf("ROSPOS %d %d %d %d %d\n", factors[0], factors[1], factors[2], factors[3], factors[4]);
  out.printf("ROSLOAD %d %d %d %d %d\n", load[0], load[1], load[2], load[3], load[4]);
  out.printf("ROSVOLT %d %d %d %d %d\n", volt[0], volt[1], volt[2], volt[3], volt[4]);
  out.printf("ROSTEMP %d %d %d %d %d\n", temp[0], temp[1], temp[2], temp[3], temp[4]);
  out.printf("ROSAMPR %d %d %d %d %d\n", ampr[0], ampr[1], ampr[2], ampr[3], ampr[4]);
}

// Reads one line, strips '\r' and surrounding spaces. Over-long lines are rejected.
bool readLine(Stream &in, String &line) {
  line = in.readStringUntil('\n');
  line.trim();
  if (line.length() > MAX_LINE_LEN) {
    line = "";
    return false;
  }
  return line.length() > 0;
}

// Reads all pending lines of one channel. FACTORY commands are accepted only on USB.
// Shown on USB while the hand is not provisioned (manufacturer setup, done once).
void printProvisioningHelp(Stream &out) {
  out.println("NOT PROVISIONED - manufacturer setup, one time only:");
  out.println("  FACTORY PROVISION <serial> <L|R> <password>");
  out.println("  serial:   year-number as on the nameplate, e.g. 2026-00001");
  out.println("  L|R:      model, Yeah Hand L (left) or Yeah Hand R (right)");
  out.println("  password: 10-32 characters, no spaces (use a random one, write it on the label)");
  out.println("  example:  FACTORY PROVISION 2026-00001 L <random password>");
  out.println("  FACTORY STATUS shows the current state.");
}

void readChannel(Stream &in, HandSecurity::Session &session, bool isUsb) {
  String lastPosCmd = "";
  String cmd;
  while (in.available()) {
    if (!readLine(in, cmd)) continue;
    if (isUsb && HandSecurity::handleFactoryCommand(cmd, in)) continue;
    if (!HandSecurity::isProvisioned()) {
      printProvisioningHelp(in);  // any other line on an unprovisioned hand
      continue;
    }
    if (!isUsb && cmd.startsWith("FACTORY")) {
      session.out->println("ERR FACTORY COMMANDS ONLY ON USB");
      continue;
    }
    if (cmd.startsWith("ROS ")) {
      lastPosCmd = cmd;  // only the most recent position command is executed
    } else {
      processStringCmd(cmd, session);
    }
  }
  if (lastPosCmd.length() > 0) {
    processStringCmd(lastPosCmd, session);
  }
}

// Bluetooth link lost: end the session and the service mode, start no new motion.
// The hand holds its current position (an object being grasped is not released).
void enterSafeStateAfterLinkLoss() {
  g_serviceMode = false;
  gFsmState = FSM::Control;
  g_closure = false;
  g_preparation = false;
  g_new_grasp_type = g_grasp_type;
  g_newClosurePercent = g_closurePercent;
}

void loop() {
  if (!HandSecurity::isProvisioned()) {
    readChannel(Serial, HandSecurity::usbSession(), true);  // FACTORY commands only
    return;
  }

  HandSecurity::loop();
  if (HandSecurity::takeBluetoothLinkClosedEvent()) enterSafeStateAfterLinkLoss();

  readChannel(Serial, HandSecurity::usbSession(), true);
  readChannel(SerialBT, HandSecurity::bluetoothSession(), false);

  if (gFsmState == FSM::Control && g_calibrated) {  // no motion before CALIBRATE
    if (g_new_grasp_type != g_grasp_type) {
      auto full_cmd = FingersController::getGraspStringByType(g_new_grasp_type);
      String msg = full_cmd + " GRASP ENABLED";

      g_out->println(msg);

      g_preparation = true;
      g_closure = false;
      g_grasp_type = g_new_grasp_type;
    }

    if (g_newClosurePercent != g_closurePercent) {

      g_preparation = false;
      g_closure = true;
      g_closurePercent = g_newClosurePercent;
    }

    if (g_preparation) {
      prepareGrasp();
    }

    if (g_closure) {
      fc.action(g_grasp_type, g_closurePercent);
      g_closure = false;
    }

  } else if (gFsmState == FSM::TendonInstall) {

    g_out->println("Tendon Install..");
    fc.tendonInstallation();
    g_out->println("Please install tendons.");

  } else if (gFsmState == FSM::Testing) {
    // test();
  } else if (gFsmState == FSM::DoNothing) {
    delay(10);
  }

  // Telemetry only to authenticated sessions
  static auto t_prev_ros_feedback = millis();
  auto t_now_ros_feedback = millis();
  if (t_now_ros_feedback - t_prev_ros_feedback > 100) {
    if (HandSecurity::bluetoothSession().authenticated) sendRosFeedback(SerialBT);
    if (HandSecurity::usbSession().authenticated) sendRosFeedback(Serial);
    t_prev_ros_feedback = t_now_ros_feedback;
  }

  static auto t_safety = millis();
  if (millis() - t_safety > 1000) {
    fc.safetyFeature();
    t_safety = millis();
  }

  static long t_prev_loop = micros();
  long t_cur_loop = micros();
  while (micros() - t_prev_loop < 100)
    ;
  t_prev_loop = t_cur_loop;
}
