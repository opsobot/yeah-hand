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
SMS_STS st;
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

void logRobotFlow(const String& message);

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
      // First boot or factory reset - run calibration
      SerialBT.println("No calibration found - running initial calibration...");
      if (fc.calibrateHand()) {
        fc.saveCalibrationToStorage(&g_calibStorage);
        if (g_calibStorage.save()) {
          SerialBT.println("Calibration saved successfully!");
          gFsmState = FSM::Control;
        } else {
          SerialBT.println("WARNING: Calibration save failed!");
          gFsmState = FSM::Control;  // Still allow operation
        }
      } else {
        SerialBT.println("ERROR: Calibration failed!");
        gFsmState = FSM::CalibrationRequired;
      }
      break;

    case CalibrationState::CORRUPT:
    case CalibrationState::INVALID_VALUES:
    case CalibrationState::INCOMPLETE:
      // Calibration data is bad - require recalibration
      SerialBT.println("ERROR: Calibration data invalid!");
      SerialBT.println("Send CALIBRATE command to recalibrate.");
      gFsmState = FSM::CalibrationRequired;
      break;

    case CalibrationState::VERSION_MISMATCH:
      // Schema changed - could try migration or require recalibration
      SerialBT.println("Calibration version mismatch - recalibrating...");
      if (fc.calibrateHand()) {
        fc.saveCalibrationToStorage(&g_calibStorage);
        g_calibStorage.save();
        gFsmState = FSM::Control;
      } else {
        gFsmState = FSM::CalibrationRequired;
      }
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
bool executeGestureRecord(const GestureRecord& record);
bool playStoredGesture(uint8_t slot);
bool playEditorGesture();
void printGestureHelp();
void logRobotFlow(const String& message);
bool tryParseIntValue(const String& text, int* out_value);
bool tryParseSlotArgument(const String& text, uint8_t* out_slot);

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

void printGestureHelp() {
  SerialBT.println("=== GESTURE COMMANDS ===");
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

  String upper_cmd = trimmed_cmd;
  upper_cmd.toUpperCase();

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
    args.trim();
    const int split = args.indexOf(' ');
    if (split < 0) {
      logRobotFlow("[Robot] MOVE format: MOVE <servoId> <position>");
      return;
    }

    int id = 0;
    int position = 0;
    if (!tryParseIntValue(args.substring(0, split), &id) ||
        !tryParseIntValue(args.substring(split + 1), &position)) {
      logRobotFlow("[Robot] MOVE requires numeric values");
      return;
    }
    if (id < 1 || id > 5) {
      logRobotFlow("[Robot] MOVE servoId must be between 1 and 5");
      return;
    }

    fc.moveUntilLoadLimitHit(id, position, 2000, 200);
    logRobotFlow("[Robot] MOVE executed for servo " + String(id) + " -> " + String(position));
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
    logRobotFlow("[Robot] Starting calibration flow");
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

  if (upper_cmd == "CALIB_SAVE") {
    if (g_calibStorage.save()) {
      logRobotFlow("[Robot] Calibration saved to flash");
    } else {
      logRobotFlow("[Robot] ERROR: Calibration save failed");
    }
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
      SerialBT.println("Send 'CALIBRATE' command to calibrate.");
      SerialBT.println("============================");
      lastWarning = millis();
    }

    // Only process commands (already handled above)
    // This state blocks all motion
    delay(100);
  }  
  // limitTemp();
}
