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

#define S_RXD 18
#define S_TXD 19
#define BLUETOOTH true

static const bool SHOW_FEEDBACK = false;
static const bool CALIBRATE_CENTER = false;

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
  SerialBT.begin("YeahHand");
  SerialBT.setTimeout(1);
<<<<<<< HEAD
  //delay(10000);
=======
>>>>>>> 6c704d6 (calibration storage)
  SerialBT.println("Yeah Hand Started!");

  // servoIdDiscovery();

  //fc.moveAllFingersToMiddlePosition()
  if (CALIBRATE_CENTER) {
<<<<<<< HEAD
    // fc.calibrateCenterOfRange(FingersController::INDEX_FLEX_MOTOR);
    // fc.calibrateCenterOfRange(FingersController::MIDDLE_FLEX_MOTOR);
    // fc.calibrateCenterOfRange(FingersController::RING_LITTLE_FLEX_MOTOR);
    // fc.calibrateCenterOfRange(FingersController::THUMB_FLEX_MOTOR);
    SerialBT.println("All servos center position calibrated!");
    while (1)
      ;
  }

  //Changing Motor ID
  //fc.changeID(5, RING_LITTLE_FLEX_MOTOR);

  // Hand Calibration
  fc.calibrateHand();
}

=======
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
}


>>>>>>> 6c704d6 (calibration storage)
FingersController::GraspType g_grasp_type = FingersController::GraspType::MONKEY;
FingersController::GraspType g_new_grasp_type = g_grasp_type;
int g_closurePercent = 0;
int g_newClosurePercent = 0;
bool g_preparation = true;
bool g_closure = false;
bool g_motor_enabled[5] = { true, true, true, true, true };
enum class FSM {
  Control,
  TendonInstall,
  Testing,
<<<<<<< HEAD
  DoNothing
=======
  DoNothing,
  CalibrationRequired  // NEW: Blocks operation until calibrated
>>>>>>> 6c704d6 (calibration storage)
};
FSM gFsmState = FSM::Control;

void processStringCmd(const String& cmd);
void prepareGrasp();
void limitLoad();
int sign(int val);

int sign(int val) {
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
  FingersController::GraspType graspType = FingersController::getGraspTypeByString(cmd);
  if (graspType < FingersController::GraspType::_MAX) {
    g_new_grasp_type = graspType;
    if (BLUETOOTH) SerialBT.println("Grasp by Serial Cmd");
  } else {  // Not a Grasp command..
    if (cmd == "INSTALL\r\n") {
      gFsmState = FSM::TendonInstall;
    } else if (cmd == "TEST\r\n") {
      gFsmState = FSM::Testing;
    } else if (cmd == "CONTROL\r\n") {
      gFsmState = FSM::Control;
    } else if (cmd.substring(0, 3) == "LIM") {  // Comando esempio: LIM 1 125
      auto id = cmd.substring(4, 5).toInt();
      auto val = cmd.substring(6, 9).toInt();
      fc.setMaxTorque(id, val);
    } else if (cmd.substring(0, 5) == "INFO ") {
      FingersController::VectorIdx IDX = (FingersController::VectorIdx) cmd.substring(5, cmd.lastIndexOf('\r')).toInt();
      auto ID = fc.getMotorIdByVectorIndex(IDX);
      fc.printFeedback(ID);
      SerialBT.printf("Factor: %d\n", fc.getFactorFromPos(IDX,fc.readPos(ID)));
    } else if (cmd.substring(0, 3) == "POS") {
      u8 IDN = 4;
      u8 IDs[IDN] = { 1, 2, 3, 4 };
      s16 pos[IDN];
      fc.readPositions(IDN, IDs, pos);
      SerialBT.printf("Positions: %d %d %d %d\n", pos[0], pos[1], pos[2], pos[3]);
    } else if (cmd.substring(0, 5) == "MOVE ") {
      int ID = cmd.substring(5, 6).toInt();
      int pos = cmd.substring(6, cmd.lastIndexOf('\r')).toInt();
      fc.moveUntilLoadLimitHit(ID, pos, 2000, 200);
    } else if (cmd.substring(0, 3) == "RT ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveUntilLoadLimitHit(FingersController::VectorIdx::ThumbRot, factor, 2000, 200);
    } else if (cmd.substring(0, 3) == "FI ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Index, factor, 2000, 200);
    } else if (cmd.substring(0, 3) == "FM ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Middle, factor, 2000, 200);
    } else if (cmd.substring(0, 3) == "FR ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Ring, factor, 2000, 200);
    } else if (cmd.substring(0, 3) == "FT ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Thumb, factor, 2000, 200);
    } else if (cmd.substring(0, 10) == "SETCENTER ") {
      int id = cmd.substring(10, 11).toInt();
      fc.setCenterOfRange(id);
    } else if (cmd.substring(0, 9) == "CALIBRATE") {
<<<<<<< HEAD
      fc.calibrateHand();
    } else {
=======
      SerialBT.println("Starting calibration...");
      if (fc.calibrateHand()) {
        fc.saveCalibrationToStorage(&g_calibStorage);
        if (g_calibStorage.save()) {
          SerialBT.println("Calibration saved successfully!");
          gFsmState = FSM::Control;
        } else {
          SerialBT.println("WARNING: Failed to save calibration!");
        }
      } else {
        SerialBT.println("ERROR: Calibration failed!");
      }
    }
    // ========== NEW CALIBRATION STORAGE COMMANDS ==========
    else if (cmd == "CALIB_DUMP\r\n") {
      g_calibStorage.dumpToSerial(&SerialBT);
    }
    else if (cmd == "CALIB_JSON\r\n") {
      g_calibStorage.dumpAsJSON(&SerialBT);
    }
    else if (cmd == "CALIB_EXPORT\r\n") {
      g_calibStorage.exportAsCommands(&SerialBT);
    }
    else if (cmd.startsWith("CALIB_SET ")) {
      String args = cmd.substring(10);
      args.trim();
      g_calibStorage.parseSetCommand(args, &SerialBT);
    }
    else if (cmd == "CALIB_SAVE\r\n") {
      if (g_calibStorage.save()) {
        SerialBT.println("Calibration saved to flash");
      } else {
        SerialBT.println("ERROR: Save failed");
      }
    }
    else if (cmd == "CALIB_RESET\r\n") {
      g_calibStorage.factoryReset();
      SerialBT.println("Calibration reset - restart to recalibrate");
    }
    else if (cmd == "CALIB_RELOAD\r\n") {
      if (g_calibStorage.load()) {
        fc.applyCalibrationFromStorage(&g_calibStorage);
        SerialBT.println("Calibration reloaded from flash");
      } else {
        SerialBT.println("ERROR: Reload failed");
      }
    }
    else if (cmd == "CALIB_STATUS\r\n") {
      SerialBT.printf("Calibration State: %s\n", g_calibStorage.getStateString());
      SerialBT.printf("Valid for operation: %s\n",
                      g_calibStorage.isValid() ? "YES" : "NO");
    }
    else {
>>>>>>> 6c704d6 (calibration storage)
      g_newClosurePercent = cmd.toInt();
      if (BLUETOOTH) SerialBT.printf("ClosurePercent by Serial Cmd: %d\n", g_newClosurePercent);
    }
  }
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
    SerialBT.printf("string: %s \n", cmd);
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
<<<<<<< HEAD
  }  
    
=======
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
>>>>>>> 6c704d6 (calibration storage)
  // limitTemp();
}