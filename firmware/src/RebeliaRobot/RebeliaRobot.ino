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

#include <SCServo.h>
#include <BTAddress.h>
#include <BTAdvertisedDevice.h>
#include <BTScan.h>
#include <BluetoothSerial.h>
#include "fingers_controller.h"
#include "calibration_storage.h"
#include "gesture_storage.h"


#define S_RXD 18
#define S_TXD 19
#define BLUETOOTH true

static const bool SHOW_FEEDBACK = false;
static const bool CALIBRATE_CENTER = false;

BluetoothSerial SerialBT;
FingersController fc(&SerialBT);

enum class FSM {
  Control,
  TendonInstall,
  Testing,
  DoNothing,
  CalibrationRequired
};

FingersController::GraspType g_grasp_type = FingersController::GraspType::MONKEY;
FingersController::GraspType g_new_grasp_type = g_grasp_type;
int g_closurePercent = 0;
int g_newClosurePercent = 0;
bool g_preparation = true;
bool g_closure = false;
bool g_motor_enabled[5] = { true, true, true, true, true };
FSM gFsmState = FSM::Control;

static const int CALIB_DEFAULT_MARGIN_STEPS = 30;
static const int CALIB_MAX_JOG_STEPS = 200;
static const int CALIB_IDENTIFY_DEFAULT_STEPS = 25;
static const int CALIB_VERIFY_TOLERANCE_STEPS = 80;
static const uint16_t CALIB_SAFE_SPEED = 300;
static const uint8_t CALIB_SAFE_ACCEL = 20;
static const uint16_t CALIB_SAFE_TORQUE = 150;

struct GuidedCalibrationSession {
  bool active;
  bool raw_open_set[NUM_MOTORS];
  bool raw_close_set[NUM_MOTORS];
  int raw_open[NUM_MOTORS];
  int raw_close[NUM_MOTORS];
  int margin[NUM_MOTORS];
};

GuidedCalibrationSession g_guidedCalibration;

void logRobotFlow(const String& message);

void servoIdDiscovery() {
  SerialBT.println("=== SERVO ID DISCOVERY ===");
  SerialBT.println("Watch the hand only after using CALIB_IDENTIFY <id>; scan only checks replies.");
  for (int id = 0; id <= MAX_SERVOS; id++) {
    fc.pingServo(id);
    delay(100);
  }
  SerialBT.println("==========================");
}

void setup() {

  Serial.begin(115200);
  SerialBT.begin("YeahHand");
  SerialBT.setTimeout(1);
  SerialBT.println("Yeah Hand Started!");

  // servoIdDiscovery();

  //fc.moveAllFingersToMiddlePosition()
  if (CALIBRATE_CENTER) {
    SerialBT.println("All servos center position calibrated!");
    while (1);
  }

  // ========== CALIBRATION STORAGE INITIALIZATION ==========
  CalibrationState calibState = g_calibStorage.initialize();

  switch (calibState) {
    case CalibrationState::VALID:
    case CalibrationState::BACKUP_USED:
      // Calibration loaded successfully
      SerialBT.printf("Calibration loaded: %s\n", g_calibStorage.getStateString());
      fc.applyCalibrationFromStorage(&g_calibStorage);
      gFsmState = FSM::Control;
      break;

    case CalibrationState::MISSING:
      SerialBT.println("No calibration found.");
      SerialBT.println("Guided Bluetooth calibration is required before normal motion.");
      SerialBT.println("Send CALIB_HELP, then CALIB_SCAN and CALIB_IDENTIFY before marking limits.");
      gFsmState = FSM::CalibrationRequired;
      break;

    case CalibrationState::CORRUPT:
    case CalibrationState::INVALID_VALUES:
    case CalibrationState::INCOMPLETE:
      // Calibration data is bad - require recalibration
      SerialBT.println("ERROR: Calibration data invalid!");
      SerialBT.println("Send CALIB_HELP for guided recalibration.");
      gFsmState = FSM::CalibrationRequired;
      break;

    case CalibrationState::VERSION_MISMATCH:
      SerialBT.println("Calibration version mismatch.");
      SerialBT.println("Guided recalibration is required. Send CALIB_HELP.");
      gFsmState = FSM::CalibrationRequired;
      break;

    default:
      SerialBT.println("ERROR: Unknown calibration state!");
      gFsmState = FSM::CalibrationRequired;
      break;
  }

  g_gestureStorage.initialize(&SerialBT);
  logRobotFlow("[Robot] Setup complete. FSM=" + String(static_cast<int>(gFsmState)));
}

void processStringCmd(const String& cmd);
void prepareGrasp();
void limitLoad();
int sign(int val);
bool handleGestureCommand(const String& trimmed_cmd);
bool handleCalibrationCommand(const String& trimmed_cmd);
bool executeGestureRecord(const GestureRecord& record);
bool playStoredGesture(uint8_t slot);
bool playEditorGesture();
void printGestureHelp();
void printCalibrationHelp();
void resetGuidedCalibrationSession();
void logRobotFlow(const String& message);
bool tryParseIntValue(const String& text, int* out_value);
bool tryParseSlotArgument(const String& text, uint8_t* out_slot);
bool parseFingerOrServoTarget(const String& token, FingersController::VectorIdx* out_idx, u8* out_id);
bool takeToken(String* text, String* out_token);
bool markGuidedCalibrationLimit(FingersController::VectorIdx idx, bool is_open, int margin);
bool applyGuidedSafeRange(FingersController::VectorIdx idx);
void printGuidedCalibrationTable();
bool verifyFingerRepeatability(FingersController::VectorIdx idx, uint8_t cycles);
bool parsePoseFactorsCsv(const String& csv, uint8_t factors[5], String* error);
bool handlePoseCommand(const String& trimmed_cmd);
void printNormalizedMotionHelp();

void logRobotFlow(const String& message) {
  Serial.println(message);
  if (BLUETOOTH) {
    SerialBT.println(message);
  }
}

bool tryParseIntValue(const String& text, int* out_value) {
  String trimmed = text;
  trimmed.trim();
  if (trimmed.isEmpty()) {
    return false;
  }

  int start = 0;
  if (trimmed.charAt(0) == '-' || trimmed.charAt(0) == '+') {
    start = 1;
  }
  if (start >= trimmed.length()) {
    return false;
  }

  for (int i = start; i < trimmed.length(); ++i) {
    const char ch = trimmed.charAt(i);
    if (ch < '0' || ch > '9') {
      return false;
    }
  }

  *out_value = trimmed.toInt();
  return true;
}

bool tryParseSlotArgument(const String& text, uint8_t* out_slot) {
  int parsed = 0;
  if (!tryParseIntValue(text, &parsed)) {
    return false;
  }
  if (parsed < 0 || parsed >= MAX_CUSTOM_GESTURES) {
    return false;
  }
  *out_slot = static_cast<uint8_t>(parsed);
  return true;
}

void resetGuidedCalibrationSession() {
  memset(&g_guidedCalibration, 0, sizeof(g_guidedCalibration));
  g_guidedCalibration.active = true;
  for (int i = 0; i < NUM_MOTORS; ++i) {
    g_guidedCalibration.margin[i] = CALIB_DEFAULT_MARGIN_STEPS;
  }
  logRobotFlow("[CalibGuide] Session started. Use CALIB_SCAN, CALIB_IDENTIFY, CALIB_JOG, CALIB_MARK, CALIB_VERIFY, CALIB_SAVE.");
}

bool takeToken(String* text, String* out_token) {
  if (!text || !out_token) {
    return false;
  }

  text->trim();
  if (text->isEmpty()) {
    return false;
  }

  const int split = text->indexOf(' ');
  if (split < 0) {
    *out_token = *text;
    *text = "";
    return true;
  }

  *out_token = text->substring(0, split);
  *text = text->substring(split + 1);
  out_token->trim();
  text->trim();
  return !out_token->isEmpty();
}

bool parseFingerOrServoTarget(const String& token, FingersController::VectorIdx* out_idx, u8* out_id) {
  String normalized = token;
  normalized.trim();
  normalized.toUpperCase();

  int servo_id = 0;
  if (tryParseIntValue(normalized, &servo_id)) {
    if (servo_id < 1 || servo_id > MAX_SERVOS) {
      return false;
    }
    if (out_id) {
      *out_id = static_cast<u8>(servo_id);
    }
    if (out_idx) {
      *out_idx = static_cast<FingersController::VectorIdx>(servo_id - 1);
    }
    return true;
  }

  FingersController::VectorIdx idx = FingersController::VectorIdx::Index;
  if (normalized == "INDEX" || normalized == "I" || normalized == "FI") {
    idx = FingersController::VectorIdx::Index;
  } else if (normalized == "MIDDLE" || normalized == "M" || normalized == "FM") {
    idx = FingersController::VectorIdx::Middle;
  } else if (normalized == "RING" || normalized == "R" || normalized == "FR" || normalized == "RING_LITTLE") {
    idx = FingersController::VectorIdx::Ring;
  } else if (normalized == "THUMB" || normalized == "T" || normalized == "FT") {
    idx = FingersController::VectorIdx::Thumb;
  } else if (normalized == "THUMB_ROT" || normalized == "THUMBROT" || normalized == "TR" || normalized == "ROT") {
    idx = FingersController::VectorIdx::ThumbRot;
  } else {
    return false;
  }

  if (out_idx) {
    *out_idx = idx;
  }
  if (out_id) {
    *out_id = static_cast<u8>(fc.getMotorIdByVectorIndex(idx));
  }
  return true;
}

bool parsePoseFactorsCsv(const String& csv, uint8_t factors[5], String* error) {
  int values[NUM_MOTORS] = { 0 };
  int count = 0;
  int start = 0;

  while (true) {
    const int comma = csv.indexOf(',', start);
    String token = comma < 0 ? csv.substring(start) : csv.substring(start, comma);
    token.trim();

    if (token.isEmpty()) {
      if (error) {
        *error = "POSE contains an empty field";
      }
      return false;
    }
    if (count >= NUM_MOTORS) {
      if (error) {
        *error = "POSE has too many values";
      }
      return false;
    }
    if (!tryParseIntValue(token, &values[count])) {
      if (error) {
        *error = "POSE fields must be integer percentages";
      }
      return false;
    }

    values[count] = constrain(values[count], 0, 100);
    factors[count] = static_cast<uint8_t>(values[count]);
    ++count;

    if (comma < 0) {
      break;
    }
    start = comma + 1;
  }

  if (count != NUM_MOTORS) {
    if (error) {
      *error = "POSE needs 5 comma-separated values: index,middle,ring,thumb,thumbRot";
    }
    return false;
  }

  return true;
}

void printNormalizedMotionHelp() {
  SerialBT.println("=== NORMALIZED MOTION COMMANDS ===");
  SerialBT.println("0 means safe open. 100 means safe closed. Values are converted through saved calibration.");
  SerialBT.println("MOVE <finger> <percent>         Example: MOVE INDEX 70");
  SerialBT.println("MOVE <servoId> <rawPosition>    Example: MOVE 3 705 (service/raw mode)");
  SerialBT.println("MAP <finger> <percent>          Example: MAP INDEX 70");
  SerialBT.println("POSE i,m,r,t,tr [speed] [accel] Example: POSE 70,75,72,60,35");
  SerialBT.println("POSE_SAVE <slot> <name> i,m,r,t,tr [moveMs] [holdMs] [speed] [accel]");
  SerialBT.println("Finger names: INDEX, MIDDLE, RING, THUMB, THUMB_ROT");
  SerialBT.println("==================================");
}

void printCalibrationHelp() {
  SerialBT.println("=== SAFE CALIBRATION GUIDE ===");
  SerialBT.println("1. 0% and 100% are safe software limits, not full servo travel. Use natural open and useful closed positions.");
  SerialBT.println("2. Bus servos share one UART line. Run CALIB_SCAN, then CALIB_IDENTIFY <id> to see which physical finger moves.");
  SerialBT.println("3. Move slowly: CALIB_JOG <finger|id> <delta>. Default calibration moves use low speed, low accel, and low torque.");
  SerialBT.println("4. Open limit: mark only a natural repeatable open posture. Do not force the joint backward.");
  SerialBT.println("5. Closed limit: mark useful grip closure before buzzing, heat, blocked joints, or high tendon tension.");
  SerialBT.println("6. Thumb curl and thumb rotation are separate: calibrate THUMB and THUMB_ROT independently.");
  SerialBT.println("7. CALIB_TABLE shows servo ID, raw observed endpoints, software-safe endpoints, travel, and margin.");
  SerialBT.println("8. CALIB_VERIFY <finger|ALL> repeats 0% and 100% moves and reports position repeatability.");
  SerialBT.println("9. CALIB_MARK applies a margin. Example: raw 300..850 with margin 30 becomes safe 330..820.");
  SerialBT.println("10. Bluetooth flow: CALIB_START -> CALIB_SCAN -> CALIB_IDENTIFY -> CALIB_JOG -> CALIB_MARK -> CALIB_VERIFY -> CALIB_SAVE.");
  SerialBT.println("");
  SerialBT.println("Commands:");
  SerialBT.println("  CALIB_START");
  SerialBT.println("  CALIB_SCAN");
  SerialBT.println("  CALIB_MAP");
  SerialBT.println("  CALIB_IDENTIFY <servoId> [delta]");
  SerialBT.println("  CALIB_JOG <finger|servoId> <delta>");
  SerialBT.println("  CALIB_MARK <finger> OPEN|CLOSE [marginSteps]");
  SerialBT.println("  CALIB_TEST <finger> <0-100>");
  SerialBT.println("  CALIB_VERIFY <finger|ALL> [cycles]");
  SerialBT.println("  CALIB_TABLE");
  SerialBT.println("  CALIB_SAVE");
  SerialBT.println("  CALIB_REID <currentId> <newId>");
  SerialBT.println("Finger names: INDEX, MIDDLE, RING, THUMB, THUMB_ROT");
  SerialBT.println("==============================");
}

bool applyGuidedSafeRange(FingersController::VectorIdx idx) {
  const int i = static_cast<int>(idx);
  if (!g_guidedCalibration.raw_open_set[i] || !g_guidedCalibration.raw_close_set[i]) {
    return false;
  }

  const int rawOpen = g_guidedCalibration.raw_open[i];
  const int rawClose = g_guidedCalibration.raw_close[i];
  const int margin = constrain(g_guidedCalibration.margin[i], 0, 500);
  const int rawTravel = abs(rawClose - rawOpen);

  if (rawTravel < MOTOR_RANGE_MIN_TRAVEL + (2 * margin)) {
    logRobotFlow("[CalibGuide] " + String(FingersController::getVectorName(idx)) +
                 " range too small after margin. Raw travel=" + String(rawTravel) +
                 " margin=" + String(margin));
    return false;
  }

  int safeOpen = rawOpen;
  int safeClose = rawClose;
  if (rawClose >= rawOpen) {
    safeOpen = rawOpen + margin;
    safeClose = rawClose - margin;
  } else {
    safeOpen = rawOpen - margin;
    safeClose = rawClose + margin;
  }

  safeOpen = constrain(safeOpen, MOTOR_POS_ABSOLUTE_MIN, MOTOR_POS_ABSOLUTE_MAX);
  safeClose = constrain(safeClose, MOTOR_POS_ABSOLUTE_MIN, MOTOR_POS_ABSOLUTE_MAX);

  g_calibStorage.setMotorMin(i, safeOpen);
  g_calibStorage.setMotorMax(i, safeClose);
  fc.setCalibratedRange(idx, safeOpen, safeClose);

  logRobotFlow("[CalibGuide] Safe range stored for " + String(FingersController::getVectorName(idx)) +
               ": rawOpen=" + String(rawOpen) +
               " rawClose=" + String(rawClose) +
               " margin=" + String(margin) +
               " safe0=" + String(safeOpen) +
               " safe100=" + String(safeClose));
  return true;
}

bool markGuidedCalibrationLimit(FingersController::VectorIdx idx, bool is_open, int margin) {
  if (!g_guidedCalibration.active) {
    resetGuidedCalibrationSession();
  }

  const int i = static_cast<int>(idx);
  const u8 id = static_cast<u8>(fc.getMotorIdByVectorIndex(idx));
  const int current = fc.readPos(id);
  if (current < MOTOR_POS_ABSOLUTE_MIN || current > MOTOR_POS_ABSOLUTE_MAX) {
    logRobotFlow("[CalibGuide] Cannot mark " + String(FingersController::getVectorName(idx)) +
                 ": servo position read failed");
    return false;
  }

  g_guidedCalibration.margin[i] = constrain(margin, 0, 500);
  if (is_open) {
    g_guidedCalibration.raw_open[i] = current;
    g_guidedCalibration.raw_open_set[i] = true;
    logRobotFlow("[CalibGuide] Marked raw OPEN for " + String(FingersController::getVectorName(idx)) +
                 " at " + String(current) + ". Confirm it is natural open, not a forced backward stop.");
  } else {
    g_guidedCalibration.raw_close[i] = current;
    g_guidedCalibration.raw_close_set[i] = true;
    logRobotFlow("[CalibGuide] Marked raw CLOSE for " + String(FingersController::getVectorName(idx)) +
                 " at " + String(current) + ". Confirm it is useful closed grip, not crushing force.");
  }

  if (g_guidedCalibration.raw_open_set[i] && g_guidedCalibration.raw_close_set[i]) {
    return applyGuidedSafeRange(idx);
  }

  logRobotFlow("[CalibGuide] Mark the other endpoint for " + String(FingersController::getVectorName(idx)) +
               " before saving this finger.");
  return true;
}

void printGuidedCalibrationTable() {
  if (!g_guidedCalibration.active) {
    resetGuidedCalibrationSession();
  }
  SerialBT.println("=== GUIDED CALIBRATION TABLE ===");
  SerialBT.println("0%=safe open, 100%=safe closed. Raw values are operator observations.");
  SerialBT.println("Finger      ID RawOpen RawClose Margin Safe0 Safe100 Travel");
  for (int i = 0; i < NUM_MOTORS; ++i) {
    const FingersController::VectorIdx idx = static_cast<FingersController::VectorIdx>(i);
    const int safeOpen = g_calibStorage.getMotorMin(i);
    const int safeClose = g_calibStorage.getMotorMax(i);
    SerialBT.printf("%-10s %2d %7s %8s %6d %5d %7d %6d\n",
                    FingersController::getVectorName(idx),
                    fc.getMotorIdByVectorIndex(idx),
                    g_guidedCalibration.raw_open_set[i] ? String(g_guidedCalibration.raw_open[i]).c_str() : "-",
                    g_guidedCalibration.raw_close_set[i] ? String(g_guidedCalibration.raw_close[i]).c_str() : "-",
                    g_guidedCalibration.margin[i],
                    safeOpen,
                    safeClose,
                    abs(safeClose - safeOpen));
  }
  SerialBT.println("================================");
}

bool verifyFingerRepeatability(FingersController::VectorIdx idx, uint8_t cycles) {
  cycles = constrain(cycles, 1, 5);
  const int travel = abs(fc.getRangeMax(idx) - fc.getRangeMin(idx));
  if (travel < MOTOR_RANGE_MIN_TRAVEL) {
    logRobotFlow("[CalibGuide] Verify blocked for " + String(FingersController::getVectorName(idx)) +
                 ": safe range travel is too small (" + String(travel) + ")");
    return false;
  }

  const u8 id = static_cast<u8>(fc.getMotorIdByVectorIndex(idx));
  const int targetOpen = fc.getPosFromFactor(idx, 0);
  const int targetClose = fc.getPosFromFactor(idx, 100);
  int firstOpen = 0;
  int firstClose = 0;
  bool ok = true;

  for (uint8_t cycle = 0; cycle < cycles; ++cycle) {
    fc.calibrationMoveFactor(idx, 0, CALIB_SAFE_SPEED, CALIB_SAFE_ACCEL, CALIB_SAFE_TORQUE);
    delay(150);
    const int openPos = fc.readPos(id);

    fc.calibrationMoveFactor(idx, 100, CALIB_SAFE_SPEED, CALIB_SAFE_ACCEL, CALIB_SAFE_TORQUE);
    delay(150);
    const int closePos = fc.readPos(id);

    if (cycle == 0) {
      firstOpen = openPos;
      firstClose = closePos;
    }

    const int openTargetError = abs(openPos - targetOpen);
    const int closeTargetError = abs(closePos - targetClose);
    const int openRepeatError = abs(openPos - firstOpen);
    const int closeRepeatError = abs(closePos - firstClose);

    const bool cycleOk = openTargetError <= CALIB_VERIFY_TOLERANCE_STEPS &&
                         closeTargetError <= CALIB_VERIFY_TOLERANCE_STEPS &&
                         openRepeatError <= CALIB_VERIFY_TOLERANCE_STEPS &&
                         closeRepeatError <= CALIB_VERIFY_TOLERANCE_STEPS;
    ok = ok && cycleOk;

    SerialBT.printf("[CalibGuide] Verify %s cycle=%u open=%d target=%d err=%d repeat=%d close=%d target=%d err=%d repeat=%d result=%s\n",
                    FingersController::getVectorName(idx),
                    cycle + 1,
                    openPos,
                    targetOpen,
                    openTargetError,
                    openRepeatError,
                    closePos,
                    targetClose,
                    closeTargetError,
                    closeRepeatError,
                    cycleOk ? "OK" : "CHECK");
  }

  logRobotFlow("[CalibGuide] Verify " + String(FingersController::getVectorName(idx)) +
               (ok ? " completed OK" : " found repeatability or target error"));
  return ok;
}

bool handleCalibrationCommand(const String& trimmed_cmd) {
  String upper_cmd = trimmed_cmd;
  upper_cmd.toUpperCase();

  if (upper_cmd == "CALIB_HELP") {
    printCalibrationHelp();
    return true;
  }

  if (upper_cmd == "CALIB_START") {
    resetGuidedCalibrationSession();
    gFsmState = FSM::CalibrationRequired;
    return true;
  }

  if (upper_cmd == "CALIB_ABORT") {
    memset(&g_guidedCalibration, 0, sizeof(g_guidedCalibration));
    logRobotFlow("[CalibGuide] Guided calibration session cleared");
    return true;
  }

  if (upper_cmd == "CALIB_SCAN") {
    servoIdDiscovery();
    return true;
  }

  if (upper_cmd == "CALIB_MAP") {
    SerialBT.println("=== EXPECTED SERVO MAP ===");
    for (int i = 0; i < NUM_MOTORS; ++i) {
      const FingersController::VectorIdx idx = static_cast<FingersController::VectorIdx>(i);
      SerialBT.printf("Servo ID %d -> %s\n", fc.getMotorIdByVectorIndex(idx), FingersController::getVectorName(idx));
    }
    SerialBT.println("If CALIB_IDENTIFY shows a different physical finger, re-label the servo or use CALIB_REID carefully.");
    SerialBT.println("==========================");
    return true;
  }

  if (upper_cmd.startsWith("CALIB_IDENTIFY ")) {
    String args = trimmed_cmd.substring(15);
    String idToken;
    String deltaToken;
    if (!takeToken(&args, &idToken)) {
      logRobotFlow("[CalibGuide] Format: CALIB_IDENTIFY <servoId> [delta]");
      return true;
    }

    int id = 0;
    if (!tryParseIntValue(idToken, &id) || id < 1 || id > MAX_SERVOS) {
      logRobotFlow("[CalibGuide] CALIB_IDENTIFY servoId must be between 1 and " + String(MAX_SERVOS));
      return true;
    }

    int delta = CALIB_IDENTIFY_DEFAULT_STEPS;
    if (takeToken(&args, &deltaToken) && !tryParseIntValue(deltaToken, &delta)) {
      logRobotFlow("[CalibGuide] CALIB_IDENTIFY delta must be numeric");
      return true;
    }
    if (delta == 0 || abs(delta) > CALIB_MAX_JOG_STEPS) {
      logRobotFlow("[CalibGuide] CALIB_IDENTIFY delta must be nonzero and within +/-" + String(CALIB_MAX_JOG_STEPS));
      return true;
    }

    fc.calibrationWiggle(static_cast<u8>(id), delta, CALIB_SAFE_SPEED, CALIB_SAFE_ACCEL, CALIB_SAFE_TORQUE);
    logRobotFlow("[CalibGuide] Watch which finger moved for servo ID " + String(id));
    return true;
  }

  if (upper_cmd.startsWith("CALIB_JOG ")) {
    String args = trimmed_cmd.substring(10);
    String targetToken;
    String deltaToken;
    if (!takeToken(&args, &targetToken) || !takeToken(&args, &deltaToken)) {
      logRobotFlow("[CalibGuide] Format: CALIB_JOG <finger|servoId> <delta>");
      return true;
    }

    FingersController::VectorIdx idx;
    u8 id = 0;
    int delta = 0;
    if (!parseFingerOrServoTarget(targetToken, &idx, &id) || !tryParseIntValue(deltaToken, &delta)) {
      logRobotFlow("[CalibGuide] CALIB_JOG needs a valid finger/servo and numeric delta");
      return true;
    }
    if (delta == 0 || abs(delta) > CALIB_MAX_JOG_STEPS) {
      logRobotFlow("[CalibGuide] CALIB_JOG delta must be nonzero and within +/-" + String(CALIB_MAX_JOG_STEPS));
      return true;
    }

    fc.calibrationJog(id, delta, CALIB_SAFE_SPEED, CALIB_SAFE_ACCEL, CALIB_SAFE_TORQUE);
    return true;
  }

  if (upper_cmd.startsWith("CALIB_MARK ")) {
    String args = trimmed_cmd.substring(11);
    String fingerToken;
    String edgeToken;
    String marginToken;
    if (!takeToken(&args, &fingerToken) || !takeToken(&args, &edgeToken)) {
      logRobotFlow("[CalibGuide] Format: CALIB_MARK <finger> OPEN|CLOSE [marginSteps]");
      return true;
    }

    FingersController::VectorIdx idx;
    u8 id = 0;
    if (!parseFingerOrServoTarget(fingerToken, &idx, &id)) {
      logRobotFlow("[CalibGuide] CALIB_MARK finger must be INDEX, MIDDLE, RING, THUMB, or THUMB_ROT");
      return true;
    }

    edgeToken.trim();
    edgeToken.toUpperCase();
    const bool isOpen = edgeToken == "OPEN" || edgeToken == "0";
    const bool isClose = edgeToken == "CLOSE" || edgeToken == "CLOSED" || edgeToken == "100";
    if (!isOpen && !isClose) {
      logRobotFlow("[CalibGuide] CALIB_MARK edge must be OPEN or CLOSE");
      return true;
    }

    int margin = g_guidedCalibration.margin[static_cast<int>(idx)];
    if (takeToken(&args, &marginToken) && !tryParseIntValue(marginToken, &margin)) {
      logRobotFlow("[CalibGuide] CALIB_MARK margin must be numeric");
      return true;
    }
    margin = constrain(margin, 0, 500);
    markGuidedCalibrationLimit(idx, isOpen, margin);
    return true;
  }

  if (upper_cmd.startsWith("CALIB_TEST ")) {
    String args = trimmed_cmd.substring(11);
    String fingerToken;
    String factorToken;
    if (!takeToken(&args, &fingerToken) || !takeToken(&args, &factorToken)) {
      logRobotFlow("[CalibGuide] Format: CALIB_TEST <finger> <0-100>");
      return true;
    }

    FingersController::VectorIdx idx;
    u8 id = 0;
    int factor = 0;
    if (!parseFingerOrServoTarget(fingerToken, &idx, &id) || !tryParseIntValue(factorToken, &factor)) {
      logRobotFlow("[CalibGuide] CALIB_TEST needs a valid finger and numeric factor");
      return true;
    }

    factor = constrain(factor, 0, 100);
    fc.calibrationMoveFactor(idx, factor, CALIB_SAFE_SPEED, CALIB_SAFE_ACCEL, CALIB_SAFE_TORQUE);
    return true;
  }

  if (upper_cmd.startsWith("CALIB_VERIFY ")) {
    String args = trimmed_cmd.substring(13);
    String targetToken;
    String cyclesToken;
    if (!takeToken(&args, &targetToken)) {
      logRobotFlow("[CalibGuide] Format: CALIB_VERIFY <finger|ALL> [cycles]");
      return true;
    }

    int cycles = 2;
    if (takeToken(&args, &cyclesToken) && !tryParseIntValue(cyclesToken, &cycles)) {
      logRobotFlow("[CalibGuide] CALIB_VERIFY cycles must be numeric");
      return true;
    }
    cycles = constrain(cycles, 1, 5);

    String targetUpper = targetToken;
    targetUpper.trim();
    targetUpper.toUpperCase();
    if (targetUpper == "ALL") {
      bool allOk = true;
      for (int i = 0; i < NUM_MOTORS; ++i) {
        allOk = verifyFingerRepeatability(static_cast<FingersController::VectorIdx>(i), static_cast<uint8_t>(cycles)) && allOk;
      }
      logRobotFlow(String("[CalibGuide] Verify ALL result: ") + (allOk ? "OK" : "CHECK OUTPUT"));
      return true;
    }

    FingersController::VectorIdx idx;
    u8 id = 0;
    if (!parseFingerOrServoTarget(targetToken, &idx, &id)) {
      logRobotFlow("[CalibGuide] CALIB_VERIFY target must be a finger name or ALL");
      return true;
    }

    verifyFingerRepeatability(idx, static_cast<uint8_t>(cycles));
    return true;
  }

  if (upper_cmd == "CALIB_TABLE") {
    printGuidedCalibrationTable();
    return true;
  }

  if (upper_cmd == "CALIB_SAVE") {
    g_calibStorage.markStageComplete(CALIB_COMPLETE);
    if (g_calibStorage.save()) {
      fc.applyCalibrationFromStorage(&g_calibStorage);
      gFsmState = FSM::Control;
      logRobotFlow("[CalibGuide] Calibration saved. FSM -> Control");
    } else {
      logRobotFlow("[CalibGuide] ERROR: Calibration save failed. Check CALIB_TABLE for missing or unsafe ranges.");
    }
    return true;
  }

  if (upper_cmd.startsWith("CALIB_REID ")) {
    String args = trimmed_cmd.substring(11);
    String currentToken;
    String newToken;
    int currentId = 0;
    int newId = 0;
    if (!takeToken(&args, &currentToken) ||
        !takeToken(&args, &newToken) ||
        !tryParseIntValue(currentToken, &currentId) ||
        !tryParseIntValue(newToken, &newId) ||
        currentId < 1 || currentId > MAX_SERVOS ||
        newId < 1 || newId > MAX_SERVOS) {
      logRobotFlow("[CalibGuide] Format: CALIB_REID <currentId> <newId>, both 1-" + String(MAX_SERVOS));
      return true;
    }

    logRobotFlow("[CalibGuide] Reassigning servo ID " + String(currentId) + " -> " + String(newId));
    fc.changeID(currentId, newId);
    return true;
  }

  return false;
}

bool handlePoseCommand(const String& trimmed_cmd) {
  String upper_cmd = trimmed_cmd;
  upper_cmd.toUpperCase();

  if (upper_cmd == "MOTION_HELP" || upper_cmd == "POSE_HELP") {
    printNormalizedMotionHelp();
    return true;
  }

  if (upper_cmd.startsWith("MAP ")) {
    String args = trimmed_cmd.substring(4);
    String fingerToken;
    String percentToken;
    if (!takeToken(&args, &fingerToken) || !takeToken(&args, &percentToken)) {
      logRobotFlow("[Robot] MAP format: MAP <finger> <percent>");
      return true;
    }

    FingersController::VectorIdx idx;
    u8 id = 0;
    int percent = 0;
    if (!parseFingerOrServoTarget(fingerToken, &idx, &id) || !tryParseIntValue(percentToken, &percent)) {
      logRobotFlow("[Robot] MAP requires a valid finger and numeric percent");
      return true;
    }

    const int clampedPercent = constrain(percent, 0, 100);
    const int target = fc.getPosFromFactor(idx, clampedPercent);
    SerialBT.printf("[Robot] MAP %s percent=%d servo=%u open=%d closed=%d target=%d\n",
                    FingersController::getVectorName(idx),
                    clampedPercent,
                    id,
                    fc.getRangeMin(idx),
                    fc.getRangeMax(idx),
                    target);
    return true;
  }

  if (upper_cmd.startsWith("POSE_SAVE ")) {
    String args = trimmed_cmd.substring(10);
    String slotToken;
    String nameToken;
    String csvToken;
    if (!takeToken(&args, &slotToken) ||
        !takeToken(&args, &nameToken) ||
        !takeToken(&args, &csvToken)) {
      logRobotFlow("[Robot] POSE_SAVE format: POSE_SAVE <slot> <name> index,middle,ring,thumb,thumbRot [moveMs] [holdMs] [speed] [accel]");
      return true;
    }

    uint8_t slot = 0;
    if (!tryParseSlotArgument(slotToken, &slot)) {
      logRobotFlow("[Robot] POSE_SAVE slot must be between 0 and " + String(MAX_CUSTOM_GESTURES - 1));
      return true;
    }

    uint8_t factors[NUM_MOTORS] = { 0 };
    String error;
    if (!parsePoseFactorsCsv(csvToken, factors, &error)) {
      logRobotFlow("[Robot] POSE_SAVE rejected: " + error);
      return true;
    }

    int moveMs = 1000;
    int holdMs = 0;
    int speed = 2000;
    int accel = 200;
    String token;
    if (takeToken(&args, &token) && !tryParseIntValue(token, &moveMs)) {
      logRobotFlow("[Robot] POSE_SAVE moveMs must be numeric");
      return true;
    }
    if (takeToken(&args, &token) && !tryParseIntValue(token, &holdMs)) {
      logRobotFlow("[Robot] POSE_SAVE holdMs must be numeric");
      return true;
    }
    if (takeToken(&args, &token) && !tryParseIntValue(token, &speed)) {
      logRobotFlow("[Robot] POSE_SAVE speed must be numeric");
      return true;
    }
    if (takeToken(&args, &token) && !tryParseIntValue(token, &accel)) {
      logRobotFlow("[Robot] POSE_SAVE accel must be numeric");
      return true;
    }

    GestureStep step;
    memset(&step, 0, sizeof(step));
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
      step.factor[i] = factors[i];
    }
    moveMs = constrain(moveMs, static_cast<int>(MIN_GESTURE_MOVE_TIME_MS), static_cast<int>(MAX_GESTURE_MOVE_TIME_MS));
    holdMs = constrain(holdMs, 0, static_cast<int>(MAX_GESTURE_HOLD_TIME_MS));
    speed = constrain(speed, static_cast<int>(MIN_GESTURE_SPEED), static_cast<int>(MAX_GESTURE_SPEED));
    accel = constrain(accel, 0, static_cast<int>(MAX_GESTURE_ACCEL));
    step.move_time_ms = static_cast<uint16_t>(moveMs);
    step.hold_time_ms = static_cast<uint16_t>(holdMs);
    step.speed = static_cast<uint16_t>(speed);
    step.accel = static_cast<uint8_t>(accel);

    if (!GestureStorage::validateStep(step, &error)) {
      logRobotFlow("[Robot] POSE_SAVE rejected: " + error);
      return true;
    }
    if (!g_gestureStorage.beginEditor(nameToken, &error)) {
      logRobotFlow("[Robot] POSE_SAVE editor start failed: " + error);
      return true;
    }
    if (!g_gestureStorage.appendEditorStep(step, &error)) {
      logRobotFlow("[Robot] POSE_SAVE append failed: " + error);
      return true;
    }
    if (!g_gestureStorage.saveEditorToSlot(slot, &error)) {
      logRobotFlow("[Robot] POSE_SAVE storage failed: " + error);
      return true;
    }

    logRobotFlow("[Robot] POSE_SAVE stored normalized pose '" + nameToken + "' in slot " + String(slot));
    return true;
  }

  if (upper_cmd.startsWith("POSE ")) {
    String args = trimmed_cmd.substring(5);
    String csvToken;
    if (!takeToken(&args, &csvToken)) {
      logRobotFlow("[Robot] POSE format: POSE index,middle,ring,thumb,thumbRot [speed] [accel]");
      return true;
    }

    uint8_t factors[NUM_MOTORS] = { 0 };
    String error;
    if (!parsePoseFactorsCsv(csvToken, factors, &error)) {
      logRobotFlow("[Robot] POSE rejected: " + error);
      return true;
    }

    int speed = 2000;
    int accel = 200;
    String token;
    if (takeToken(&args, &token) && !tryParseIntValue(token, &speed)) {
      logRobotFlow("[Robot] POSE speed must be numeric");
      return true;
    }
    if (takeToken(&args, &token) && !tryParseIntValue(token, &accel)) {
      logRobotFlow("[Robot] POSE accel must be numeric");
      return true;
    }

    speed = constrain(speed, 100, 4000);
    accel = constrain(accel, 0, 255);
    fc.movePosePercent(factors, static_cast<u16>(speed), static_cast<u8>(accel));
    logRobotFlow("[Robot] POSE executed as normalized factors");
    return true;
  }

  return false;
}

void printGestureHelp() {
  SerialBT.println("=== GESTURE COMMANDS ===");
  SerialBT.println("MOTION_HELP");
  SerialBT.println("MOVE <finger> <percent>");
  SerialBT.println("MAP <finger> <percent>");
  SerialBT.println("POSE idx,mid,ring,thumb,thumbRot [speed] [accel]");
  SerialBT.println("POSE_SAVE <slot> <name> idx,mid,ring,thumb,thumbRot [moveMs] [holdMs] [speed] [accel]");
  SerialBT.println("GEST_BEGIN <name>");
  SerialBT.println("GEST_STEP idx,mid,ring,thumb,thumbRot,moveMs,holdMs,speed,accel");
  SerialBT.println("GEST_EDITOR");
  SerialBT.println("GEST_SAVE <slot>");
  SerialBT.println("GEST_LOAD <slot>");
  SerialBT.println("GEST_PLAY <slot>");
  SerialBT.println("GEST_PLAY_RAM");
  SerialBT.println("GEST_DUMP <slot>");
  SerialBT.println("GEST_LIST");
  SerialBT.println("GEST_DELETE <slot>");
  SerialBT.println("GEST_ABORT");
  SerialBT.println("========================");
}

bool executeGestureRecord(const GestureRecord& record) {
  if (!g_calibStorage.isValid()) {
    logRobotFlow("[Robot] Custom gesture playback blocked: calibration is invalid");
    return false;
  }

  if (gFsmState != FSM::Control) {
    logRobotFlow("[Robot] Custom gesture playback blocked: controller is not in CONTROL state");
    return false;
  }

  logRobotFlow("[Robot] Playing gesture '" + String(record.name) + "' with " + String(record.step_count) + " steps");

  g_preparation = false;
  g_closure = false;

  for (uint8_t step_index = 0; step_index < record.step_count; ++step_index) {
    const GestureStep& step = record.steps[step_index];
    logRobotFlow("[Robot] Gesture step " + String(step_index) + " started");
    const bool step_ok = fc.executeTimedStep(step.factor, step.speed, step.accel,
                                             step.move_time_ms, step.hold_time_ms);
    if (!step_ok) {
      logRobotFlow("[Robot] Gesture playback aborted at step " + String(step_index));
      return false;
    }
  }

  logRobotFlow("[Robot] Gesture playback completed");
  return true;
}

bool playStoredGesture(uint8_t slot) {
  GestureRecord record;
  if (!g_gestureStorage.loadGesture(slot, &record)) {
    logRobotFlow("[Robot] No saved gesture found in slot " + String(slot));
    return false;
  }
  return executeGestureRecord(record);
}

bool playEditorGesture() {
  const GestureEditor& editor = g_gestureStorage.getEditor();
  if (!editor.has_data || editor.step_count == 0) {
    logRobotFlow("[Robot] Gesture editor is empty");
    return false;
  }

  GestureRecord record;
  memset(&record, 0, sizeof(record));
  strncpy(record.name, editor.name, GESTURE_NAME_CAPACITY - 1);
  record.name[GESTURE_NAME_CAPACITY - 1] = '\0';
  record.step_count = editor.step_count;
  memcpy(record.steps, editor.steps, sizeof(editor.steps));
  return executeGestureRecord(record);
}

bool handleGestureCommand(const String& trimmed_cmd) {
  String upper_cmd = trimmed_cmd;
  upper_cmd.toUpperCase();

  if (upper_cmd == "GEST_HELP") {
    printGestureHelp();
    return true;
  }

  if (upper_cmd == "GEST_LIST") {
    g_gestureStorage.listGestures(&SerialBT);
    return true;
  }

  if (upper_cmd == "GEST_EDITOR") {
    g_gestureStorage.dumpEditor(&SerialBT);
    return true;
  }

  if (upper_cmd == "GEST_PLAY_RAM") {
    playEditorGesture();
    return true;
  }

  if (upper_cmd == "GEST_ABORT") {
    g_gestureStorage.clearEditor();
    logRobotFlow("[Robot] Gesture editor cleared");
    return true;
  }

  if (upper_cmd.startsWith("GEST_BEGIN ")) {
    String error;
    if (!g_gestureStorage.beginEditor(trimmed_cmd.substring(11), &error)) {
      logRobotFlow("[Robot] Gesture editor start failed: " + error);
    }
    return true;
  }

  if (upper_cmd.startsWith("GEST_STEP ")) {
    GestureStep step;
    String error;
    if (!GestureStorage::parseGestureStepCsv(trimmed_cmd.substring(10), &step, &error)) {
      logRobotFlow("[Robot] Gesture step rejected: " + error);
      return true;
    }
    if (!g_gestureStorage.appendEditorStep(step, &error)) {
      logRobotFlow("[Robot] Gesture step append failed: " + error);
      return true;
    }
    return true;
  }

  if (upper_cmd.startsWith("GEST_SAVE ")) {
    uint8_t slot = 0;
    if (!tryParseSlotArgument(trimmed_cmd.substring(10), &slot)) {
      logRobotFlow("[Robot] GEST_SAVE requires a slot between 0 and " + String(MAX_CUSTOM_GESTURES - 1));
      return true;
    }
    String error;
    if (!g_gestureStorage.saveEditorToSlot(slot, &error)) {
      logRobotFlow("[Robot] Gesture save failed: " + error);
      return true;
    }
    logRobotFlow("[Robot] Gesture saved to slot " + String(slot));
    return true;
  }

  if (upper_cmd.startsWith("GEST_LOAD ")) {
    uint8_t slot = 0;
    if (!tryParseSlotArgument(trimmed_cmd.substring(10), &slot)) {
      logRobotFlow("[Robot] GEST_LOAD requires a slot between 0 and " + String(MAX_CUSTOM_GESTURES - 1));
      return true;
    }
    String error;
    if (!g_gestureStorage.loadGestureIntoEditor(slot, &error)) {
      logRobotFlow("[Robot] Gesture load failed: " + error);
      return true;
    }
    g_gestureStorage.dumpEditor(&SerialBT);
    return true;
  }

  if (upper_cmd.startsWith("GEST_PLAY ")) {
    uint8_t slot = 0;
    if (!tryParseSlotArgument(trimmed_cmd.substring(10), &slot)) {
      logRobotFlow("[Robot] GEST_PLAY requires a slot between 0 and " + String(MAX_CUSTOM_GESTURES - 1));
      return true;
    }
    playStoredGesture(slot);
    return true;
  }

  if (upper_cmd.startsWith("GEST_DUMP ")) {
    uint8_t slot = 0;
    if (!tryParseSlotArgument(trimmed_cmd.substring(10), &slot)) {
      logRobotFlow("[Robot] GEST_DUMP requires a slot between 0 and " + String(MAX_CUSTOM_GESTURES - 1));
      return true;
    }
    g_gestureStorage.dumpGesture(slot, &SerialBT);
    return true;
  }

  if (upper_cmd.startsWith("GEST_DELETE ")) {
    uint8_t slot = 0;
    if (!tryParseSlotArgument(trimmed_cmd.substring(12), &slot)) {
      logRobotFlow("[Robot] GEST_DELETE requires a slot between 0 and " + String(MAX_CUSTOM_GESTURES - 1));
      return true;
    }
    String error;
    if (!g_gestureStorage.deleteGesture(slot, &error)) {
      logRobotFlow("[Robot] Gesture delete failed: " + error);
    }
    return true;
  }

  return false;
}

int sign(int val) {
  if (val == 0) {
    return 0;
  }
  return val / abs(val);
}

int gMaxOverLoadCount = 40;
int gOverLoadCounters[5];  // 0: index, ..

// void limitTemp() {

//   for (int i = 0; i <= 4; i++) {
//     int id = fc.getMotorIdByVectorIndex(static_cast<FingersController::VectorIdx>(i));
//     auto temp = fc.readTemper(id);
//     if (temp > 65) {
//       fc.enableTorque(id, false);
//       g_motor_enabled[i] = false;
//       SerialBT.printf("Motor %d OVERHEAT! => Motor disabled! \n", id);
//     } else if (!g_motor_enabled[i] && temp < 60) {
//       fc.enableTorque(id, true);
//       g_motor_enabled[i] = true;
//       SerialBT.printf("Motor %d Cooled Down => Motor enabled! \n", id);
//     }
//   }
// }

void processStringCmd(const String& cmd) {
  String trimmed_cmd = cmd;
  trimmed_cmd.trim();
  if (trimmed_cmd.isEmpty()) {
    return;
  }

  logRobotFlow("[Robot] RX command: " + trimmed_cmd);

  if (handleGestureCommand(trimmed_cmd)) {
    return;
  }

  if (handleCalibrationCommand(trimmed_cmd)) {
    return;
  }

  String upper_cmd = trimmed_cmd;
  upper_cmd.toUpperCase();

  if (gFsmState == FSM::CalibrationRequired && !upper_cmd.startsWith("CALIB")) {
    logRobotFlow("[Robot] Command blocked until calibration is valid. Send CALIB_HELP.");
    return;
  }

  if (handlePoseCommand(trimmed_cmd)) {
    return;
  }

  FingersController::GraspType graspType = FingersController::getGraspTypeByString(trimmed_cmd);
  if (graspType < FingersController::GraspType::_MAX) {
    g_new_grasp_type = graspType;
    logRobotFlow("[Robot] Requested grasp: " + FingersController::getGraspStringByType(graspType));
    return;
  }

  if (upper_cmd == "INSTALL") {
    gFsmState = FSM::TendonInstall;
    logRobotFlow("[Robot] FSM -> TendonInstall");
    return;
  }

  if (upper_cmd == "TEST") {
    gFsmState = FSM::Testing;
    logRobotFlow("[Robot] FSM -> Testing");
    return;
  }

  if (upper_cmd == "CONTROL") {
    gFsmState = FSM::Control;
    logRobotFlow("[Robot] FSM -> Control");
    return;
  }

  if (upper_cmd.startsWith("LIM ")) {
    String args = trimmed_cmd.substring(4);
    args.trim();
    const int split = args.indexOf(' ');
    if (split < 0) {
      logRobotFlow("[Robot] LIM format: LIM <servoId> <torque>");
      return;
    }

    int id = 0;
    int torque = 0;
    if (!tryParseIntValue(args.substring(0, split), &id) ||
        !tryParseIntValue(args.substring(split + 1), &torque)) {
      logRobotFlow("[Robot] LIM requires numeric values");
      return;
    }
    if (id < 1 || id > 5) {
      logRobotFlow("[Robot] LIM servoId must be between 1 and 5");
      return;
    }

    fc.setMaxTorque(id, torque);
    logRobotFlow("[Robot] Torque limit updated for servo " + String(id) + " -> " + String(torque));
    return;
  }

  if (upper_cmd.startsWith("INFO ")) {
    int idx = 0;
    if (!tryParseIntValue(trimmed_cmd.substring(5), &idx) || idx < 0 || idx > 4) {
      logRobotFlow("[Robot] INFO requires a vector index between 0 and 4");
      return;
    }

    FingersController::VectorIdx vector_idx = static_cast<FingersController::VectorIdx>(idx);
    const int id = fc.getMotorIdByVectorIndex(vector_idx);
    fc.printFeedback(id);
    SerialBT.printf("Factor: %d\n", fc.getFactorFromPos(vector_idx, fc.readPos(id)));
    return;
  }

  if (upper_cmd == "POS") {
    const u8 IDN = 5;
    u8 IDs[IDN] = { 1, 2, 3, 4, 5 };
    s16 pos[IDN];
    fc.readPositions(IDN, IDs, pos);
    SerialBT.printf("Positions: %d %d %d %d %d\n", pos[0], pos[1], pos[2], pos[3], pos[4]);
    return;
  }

  if (upper_cmd.startsWith("MOVE ")) {
    String args = trimmed_cmd.substring(5);
    String targetToken;
    String valueToken;
    if (!takeToken(&args, &targetToken) || !takeToken(&args, &valueToken)) {
      logRobotFlow("[Robot] MOVE format: MOVE <finger> <percent> or MOVE <servoId> <rawPosition>");
      return;
    }

    int rawServoId = 0;
    if (tryParseIntValue(targetToken, &rawServoId)) {
      int rawPosition = 0;
      if (!tryParseIntValue(valueToken, &rawPosition)) {
        logRobotFlow("[Robot] Raw MOVE position must be numeric");
        return;
      }
      if (rawServoId < 1 || rawServoId > MAX_SERVOS) {
        logRobotFlow("[Robot] Raw MOVE servoId must be between 1 and " + String(MAX_SERVOS));
        return;
      }

      rawPosition = constrain(rawPosition, MOTOR_POS_ABSOLUTE_MIN, MOTOR_POS_ABSOLUTE_MAX);
      fc.moveUntilLoadLimitHit(static_cast<u8>(rawServoId), static_cast<s16>(rawPosition), 2000, 200);
      logRobotFlow("[Robot] Raw MOVE executed for servo " + String(rawServoId) + " -> " + String(rawPosition));
      return;
    }

    FingersController::VectorIdx idx;
    u8 id = 0;
    int percent = 0;
    if (!parseFingerOrServoTarget(targetToken, &idx, &id) || !tryParseIntValue(valueToken, &percent)) {
      logRobotFlow("[Robot] Normalized MOVE needs a valid finger and numeric percent");
      return;
    }

    const int clampedPercent = constrain(percent, 0, 100);
    fc.moveFingerPercent(idx, clampedPercent, 2000, 200);
    logRobotFlow("[Robot] Normalized MOVE executed for " + String(FingersController::getVectorName(idx)) +
                 " servo=" + String(id) +
                 " percent=" + String(clampedPercent));
    return;
  }

  if (upper_cmd.startsWith("RT ")) {
    int factor = 0;
    if (!tryParseIntValue(trimmed_cmd.substring(3), &factor)) {
      logRobotFlow("[Robot] RT requires a numeric factor");
      return;
    }
    factor = constrain(factor, 0, 100);
    fc.moveUntilLoadLimitHit(FingersController::VectorIdx::ThumbRot, factor, 2000, 200);
    return;
  }

  if (upper_cmd.startsWith("FI ")) {
    int factor = 0;
    if (!tryParseIntValue(trimmed_cmd.substring(3), &factor)) {
      logRobotFlow("[Robot] FI requires a numeric factor");
      return;
    }
    factor = constrain(factor, 0, 100);
    fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Index, factor, 2000, 200);
    return;
  }

  if (upper_cmd.startsWith("FM ")) {
    int factor = 0;
    if (!tryParseIntValue(trimmed_cmd.substring(3), &factor)) {
      logRobotFlow("[Robot] FM requires a numeric factor");
      return;
    }
    factor = constrain(factor, 0, 100);
    fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Middle, factor, 2000, 200);
    return;
  }

  if (upper_cmd.startsWith("FR ")) {
    int factor = 0;
    if (!tryParseIntValue(trimmed_cmd.substring(3), &factor)) {
      logRobotFlow("[Robot] FR requires a numeric factor");
      return;
    }
    factor = constrain(factor, 0, 100);
    fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Ring, factor, 2000, 200);
    return;
  }

  if (upper_cmd.startsWith("FT ")) {
    int factor = 0;
    if (!tryParseIntValue(trimmed_cmd.substring(3), &factor)) {
      logRobotFlow("[Robot] FT requires a numeric factor");
      return;
    }
    factor = constrain(factor, 0, 100);
    fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Thumb, factor, 2000, 200);
    return;
  }

  if (upper_cmd.startsWith("SETCENTER ")) {
    int id = 0;
    if (!tryParseIntValue(trimmed_cmd.substring(10), &id) || id < 1 || id > 5) {
      logRobotFlow("[Robot] SETCENTER requires a servoId between 1 and 5");
      return;
    }
    fc.setCenterOfRange(id);
    return;
  }

  if (upper_cmd == "CALIBRATE") {
    logRobotFlow("[Robot] Starting legacy automatic calibration flow");
    logRobotFlow("[Robot] For first-time hand setup, prefer CALIB_HELP guided calibration so you identify servo IDs and choose safe visual limits.");
    if (fc.calibrateHand()) {
      fc.saveCalibrationToStorage(&g_calibStorage);
      if (g_calibStorage.save()) {
        logRobotFlow("[Robot] Calibration saved successfully");
        gFsmState = FSM::Control;
      } else {
        logRobotFlow("[Robot] WARNING: Failed to save calibration");
      }
    } else {
      logRobotFlow("[Robot] ERROR: Calibration failed");
    }
    return;
  }

  if (upper_cmd == "CALIB_DUMP") {
    g_calibStorage.dumpToSerial(&SerialBT);
    return;
  }

  if (upper_cmd == "CALIB_JSON") {
    g_calibStorage.dumpAsJSON(&SerialBT);
    return;
  }

  if (upper_cmd == "CALIB_EXPORT") {
    g_calibStorage.exportAsCommands(&SerialBT);
    return;
  }

  if (upper_cmd.startsWith("CALIB_SET ")) {
    String args = trimmed_cmd.substring(10);
    args.trim();
    g_calibStorage.parseSetCommand(args, &SerialBT);
    return;
  }

  if (upper_cmd == "CALIB_RESET") {
    g_calibStorage.factoryReset();
    logRobotFlow("[Robot] Calibration reset - restart required");
    return;
  }

  if (upper_cmd == "CALIB_RELOAD") {
    if (g_calibStorage.load()) {
      fc.applyCalibrationFromStorage(&g_calibStorage);
      gFsmState = FSM::Control;
      logRobotFlow("[Robot] Calibration reloaded from flash");
    } else {
      logRobotFlow("[Robot] ERROR: Calibration reload failed");
    }
    return;
  }

  if (upper_cmd == "CALIB_STATUS") {
    SerialBT.printf("Calibration State: %s\n", g_calibStorage.getStateString());
    SerialBT.printf("Valid for operation: %s\n",
                    g_calibStorage.isValid() ? "YES" : "NO");
    return;
  }

  int closure_percent = 0;
  if (tryParseIntValue(trimmed_cmd, &closure_percent)) {
    const int constrained_percent = constrain(closure_percent, 0, 100);
    if (constrained_percent != closure_percent) {
      logRobotFlow("[Robot] Closure percent clamped from " + String(closure_percent) +
                   " to " + String(constrained_percent));
    }
    g_newClosurePercent = constrained_percent;
    logRobotFlow("[Robot] Requested closure percent -> " + String(g_newClosurePercent));
    return;
  }

  logRobotFlow("[Robot] Unknown command: " + trimmed_cmd);
}

void prepareGrasp() {

  if (g_grasp_type < FingersController::GraspType::_FINGER_CTRL_SEPARATOR) {
    fc.prepareGrasp(g_grasp_type);
  }
  g_preparation = false;
}

// // TESTING FSM
// enum class TestFSM {
//   OPEN,
//   CLOSE
// };

// TestFSM gTestFsmState = TestFSM::OPEN;
// int gFsmTimer = millis();
// int gTestCounter = -1;
// void test() {
//   switch (gTestFsmState) {

//     case TestFSM::OPEN:
//       fc.action(FingersController::GraspType::POWER, 0);
//       delay(2000);
//       gTestFsmState = TestFSM::CLOSE;
//       break;
//     case TestFSM::CLOSE:
//       fc.action(FingersController::GraspType::POWER, 100);
//       delay(2000);
//       gTestFsmState = TestFSM::OPEN;
//       gTestCounter++;
//       break;
//   }
//   int tI = fc.readTemper(fc.getMotorIdByVectorIndex(FingersController::VectorIdx::Index));
//   int tM = fc.readTemper(fc.getMotorIdByVectorIndex(FingersController::VectorIdx::Middle));
//   int tR = fc.readTemper(fc.getMotorIdByVectorIndex(FingersController::VectorIdx::Ring));
//   int tT = fc.readTemper(fc.getMotorIdByVectorIndex(FingersController::VectorIdx::Thumb));
//   int tTR = fc.readTemper(fc.getMotorIdByVectorIndex(FingersController::VectorIdx::ThumbRot));
//   SerialBT.printf("Test cycles: %d \t Temper: I:%d \t M:%d \t R:%d \t T:%d \t TR:%d \t \n", gTestCounter, tI, tM, tR, tT, tTR);
// }

void loop() {
  auto t1 = millis();

  if (BLUETOOTH && SerialBT.available()) {
    auto t1 = millis();
    auto cmd = SerialBT.readString();
    auto t2 = millis();
    SerialBT.printf("dt1: %d ms\n", t2 - t1);
    SerialBT.printf("string: %s \n", cmd.c_str());
    processStringCmd(cmd);
  }

  if (gFsmState == FSM::Control) {
    if (g_new_grasp_type != g_grasp_type) {
      auto full_cmd = FingersController::getGraspStringByType(g_new_grasp_type);
      auto reduced_cmd = full_cmd.substring(0, full_cmd.length() - 2);
      String msg = reduced_cmd + " GRASP ENABLED";

      if (BLUETOOTH) SerialBT.println(msg);

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

    SerialBT.println("Tendon Install..");
    fc.tendonInstallation();
    SerialBT.println("Please install tendons.");

  } else if (gFsmState == FSM::Testing) {
    // test();
  } else if (gFsmState == FSM::DoNothing) {
    delay(10);
  } else if (gFsmState == FSM::CalibrationRequired) {
    // Calibration required - block operation
    static unsigned long lastWarning = 0;
    if (millis() - lastWarning > 5000) {
      SerialBT.println("=== CALIBRATION REQUIRED ===");
      SerialBT.println("Hand motion is BLOCKED until calibrated.");
      SerialBT.println("Send CALIB_HELP for guided safe calibration.");
      SerialBT.println("Legacy auto-calibration is still available with CALIBRATE.");
      SerialBT.println("============================");
      lastWarning = millis();
    }

    // Only process commands (already handled above)
    // This state blocks all motion
    delay(100);
  }  
  // limitTemp();
}
