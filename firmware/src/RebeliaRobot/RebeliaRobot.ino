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
#define DEBUG false

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
  SerialBT.setTimeout(50);
  //delay(10000);
  SerialBT.println("Yeah Hand Started!");

  // Hand Calibration
  fc.calibrateHand();
}

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
  DoNothing
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

void processStringCmd(const String& cmd) {
  if (DEBUG) {
    if (BLUETOOTH) SerialBT.printf("string: %s \n", cmd.c_str());
    Serial.print("string: ");
    Serial.println(cmd.c_str());
  }

  FingersController::GraspType graspType = FingersController::getGraspTypeByString(cmd);
  if (graspType < FingersController::GraspType::_MAX) {
    g_new_grasp_type = graspType;
    if (BLUETOOTH) SerialBT.println("Grasp by Serial Cmd");
  } else {  // Not a Grasp command..
    if (cmd.substring(0, 4) == "ROS ") {
      int v[5];
      int n = sscanf(cmd.c_str(), "ROS %d %d %d %d %d",
                     &v[0], &v[1], &v[2], &v[3], &v[4]);
      if (n != 5) {
        // Incomplete line: log and discard
        if (BLUETOOTH && DEBUG) SerialBT.printf("BAD ROS CMD (tokens=%d): %s\n", n, cmd.c_str());
      } else {
        // range check
        for (int i = 0; i < 5; ++i) {
          if (v[i] < 0 || v[i] > 100) {
            if (BLUETOOTH && DEBUG) SerialBT.printf("OUT OF RANGE ROS CMD: %s\n", cmd.c_str());
          } else {
            const u8 IDN = 5;
            FingersController::VectorIdx IDXs[IDN] = { FingersController::VectorIdx::Index, FingersController::VectorIdx::Middle, FingersController::VectorIdx::Ring, FingersController::VectorIdx::Thumb, FingersController::VectorIdx::ThumbRot };
            u8 Factor[IDN] = { v[0], v[1], v[2], v[3], v[4] };
            u16 Speed[IDN] = { 3000, 3000, 3000, 3000, 3000 };
            u8 Acc[IDN] = { 250, 250, 250, 250, 250 };
            fc.moveUntilLoadLimitHit(IDN, IDXs, Factor, Speed, Acc); // sync
            //fc.move(IDN, IDXs, Factor, Speed, Acc); // async
          }
        }
      }
    } else if (cmd.substring(0, 7) == "INSTALL") {
      gFsmState = FSM::TendonInstall;
    } else if (cmd == "TEST\r\n") {
      gFsmState = FSM::Testing;
    } else if (cmd.substring(0, 7) == "CONTROL") {
      gFsmState = FSM::Control;
    } else if (cmd.substring(0, 3) == "LIM") {  // Comando esempio: LIM 1 125
      auto id = cmd.substring(4, 5).toInt();
      auto val = cmd.substring(6, 9).toInt();
      fc.setMaxTorque(id, val);
    } else if (cmd.substring(0, 5) == "INFO ") {
      auto ID = cmd.substring(5, cmd.lastIndexOf('\r')).toInt();
      auto IDX = fc.getVectorIndexByMotorID(ID);
      if (IDX < 0 || IDX > FingersController::SERVOS_SIZE) {
        SerialBT.printf("Wrong ID: %d, ID\n");
        return;
      }
      fc.printFeedback(ID);
      SerialBT.printf("Factor: %d\n", fc.getFactorFromPos(IDX, fc.readPos(ID)));
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
      fc.moveUntilLoadLimitHit(FingersController::VectorIdx::ThumbRot, factor, 3073, 200);
    } else if (cmd.substring(0, 3) == "FI ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Index, factor, 3073, 200);
    } else if (cmd.substring(0, 3) == "FM ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Middle, factor, 3073, 200);
    } else if (cmd.substring(0, 3) == "FR ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Ring, factor, 3073, 200);
    } else if (cmd.substring(0, 3) == "FT ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveUntilLoadLimitHit(FingersController::VectorIdx::Thumb, factor, 3073, 200);
    } else if (cmd.substring(0, 10) == "SETCENTER ") {
      int id = cmd.substring(10, 11).toInt();
      fc.setCenterOfRange(id);
    } else if (cmd.substring(0, 9) == "CALIBRATE") {
      fc.calibrateHand();
    } else if (cmd.substring(0, 9) == "SETPINCH ") {
      int offset = cmd.substring(9, cmd.lastIndexOf('\r')).toInt();
      fc.setPinchOffset(offset);
    } else {
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

void sendRosFeedback() {
  u8 factors[FingersController::SERVOS_SIZE];
  s16 load[FingersController::SERVOS_SIZE];
  u8 volt[FingersController::SERVOS_SIZE];
  u8 temp[FingersController::SERVOS_SIZE];
  s16 ampr[FingersController::SERVOS_SIZE];
  fc.readFactors(factors);
  fc.readFeedback(load, volt, temp, ampr);
  SerialBT.printf("ROSPOS %d %d %d %d %d\n", factors[0], factors[1], factors[2], factors[3], factors[4]);
  SerialBT.printf("ROSLOAD %d %d %d %d %d\n", load[0], load[1], load[2], load[3], load[4]);
  SerialBT.printf("ROSVOLT %d %d %d %d %d\n", volt[0], volt[1], volt[2], volt[3], volt[4]);
  SerialBT.printf("ROSTEMP %d %d %d %d %d\n", temp[0], temp[1], temp[2], temp[3], temp[4]);
  SerialBT.printf("ROSAMPR %d %d %d %d %d\n", ampr[0], ampr[1], ampr[2], ampr[3], ampr[4]);
}

void smartReadSerialBT() {
  String lastPosCmd = "";
  while (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    if (cmd.length() == 0) continue;
    if (cmd.substring(0, 4) == "ROS ") {
      lastPosCmd = cmd;
    } else {
      processStringCmd(cmd);
    }
  }
  if (lastPosCmd.length() > 0) {
    processStringCmd(lastPosCmd);
  }
}

void loop() {
  auto t0 = millis();

  smartReadSerialBT();

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
  }

  static auto t_prev_ros_feedback = millis();
  auto t_now_ros_feedback = millis();
  if (t_now_ros_feedback - t_prev_ros_feedback > 100) {
    sendRosFeedback();
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
  long t_loop_delta = micros() - t_prev_loop;
  t_prev_loop = t_cur_loop;
}
