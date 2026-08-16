
/*
Rebelia-Hand-Firmware is the control software for the Rebelia Hand, an Active Prosthetic Hand device (see https://www.robotgarage.org).

The Copyright Notice
Copyright (C)  2023 Vittorio Lumare

The License Notices
    This file is part of Rebelia-Hand-Firmware.

    Rebelia-Hand-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

    Rebelia-Hand-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with Rebelia-Hand-Firmware. If not, see <https://www.gnu.org/licenses/>. 
*/

#include <stdexcept>
#include <SCServo.h>
#include "BluetoothSerial.h"

#include <vector>
#include <functional>
using namespace std::placeholders;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

static const bool DEBUG = false;

static int limit(const int val, const int a, const int b) {

  if (a <= b) {
    return constrain(val, a, b);
  } else
    return constrain(val, b, a);
}

static bool isClose(int value, int target, int tolerance) {
  return abs(value - target) <= tolerance;
}

struct SafetyLimits {
  int MAX_LOAD;
  int OVER_CURRENT;
  int OVER_LOAD;
  int OVER_TEMP;
};

class FingersController {
public:

  enum VectorIdx {
    Index = 0,
    Middle = 1,
    Ring = 2,
    Thumb = 3,
    ThumbRot = 4,
    Factor = 5
  };

  enum GraspType {
    POWER = 0,
    POWERSMALL,
    MONKEY,
    PINCH,
    RELAX,
    POWERTOOL,
    _FINGER_CTRL_SEPARATOR,
    INDEX,
    MIDDLE,
    RING,
    THUMB,
    THUMB_ROT,
    _MAX
  };

  enum ClosingRotDir : int {
    CW = 1,
    CCW = -1
  };

  enum HandSide {
      LeftSide = 0,
      RightSide = 1
  };

public:
  static GraspType getGraspTypeByString(const String& cmd);
  static String getGraspStringByType(FingersController::GraspType type);

  FingersController(BluetoothSerial* _serialBT);
  ~FingersController();

  // Setup
  void tendonInstallation();
  void setCenterOfRange(int motor_id);
  //void updateCenterOfRange(IDN, IDXs, RANGE_MIN);
  void calibrateHand();
  void changeID(const int currentId, const int newId);
  void setRangeByCurrentPos(u8 IDN, VectorIdx IDXs[], u8 rangeIndex);
  void setRangeByCurrentPos(VectorIdx IDX, u8 rangeIndex);
  void setPinchOffset(int offset);

  // Conversion
  int getPosFromFactor(int finger_idx, int factor);
  int getFactorFromPos(int finger_idx, s16 pos);
  void getDataFromTrajectory(GraspType grasp_type, int frame, int data[6]);
  int getTrajectorySize(GraspType grasp_type);
  int getMotorIdByVectorIndex(const VectorIdx idx);
  int getVectorIndexByMotorID(const int motor_id);


  // High Level
  void action(const FingersController::GraspType grasp_type, const int factor);
  void prepareGrasp(GraspType grasp_type);
  void grasp(const GraspType grasp_type, const int gesture_factor);

  // Info
  bool isMoving(const u8 ID);
  int readLoad(const u8 ID);
  int readPos(const u8 ID);
  int readTemper(const u8 ID);
  int readCurrent(const u8 ID);
  void pingTest(const u8 ID);
  int readMaxTorque(const u8 ID);
  // ROS feedback
  void readFactors(u8 factors[5]);
  void readFeedback(u8 IDN, u8 IDs[], s16 load[], u8 voltage[],
                  u8 temperature[], s16 current[]);
  void readFeedback(s16 load[5], u8 voltage[5],
                  u8 temperature[5], s16 current[5]);

  // Print
  void printFeedback(const int id);
  void printLoad();


  // Motion
  int moveFingerSync(const int factor, VectorIdx idx);
  int moveFingerSync(const u8 ID, const s16 pos, const u16 speed, const u8 accel);
  void moveFingerAsync(const int factor, VectorIdx idx, u16 speed, u8 acc);
  void moveFingerAsync(const s16 pos, const u8 ID, const u16 speed, const u8 accel);
  void moveAllFingersToMiddlePosition();
  void enableTorque(u8 ID, bool enable);
  void setMaxTorque(const u8 ID, const u16 maxTorque);
  void setMaxTorque(const u8 IDN, u8 IDs[], const u16 MaxTorque[]);

  // Load Limit Motion
  void readPositions(u8 IDN, u8 IDs[], s16 positions[]);
  bool posMatch(u8 IDN, u8 IDs[], s16 target_pos[]);
  bool posMatch(u8 IDN, u8 pos1[], u8 pos2[], u8 thr);
  bool posMatchWait(u8 IDN, u8 IDs[], s16 target_pos[], const unsigned long timeout);  // Returns false if timed out
  void setCurPosAsTarget(u8 IDN, u8 IDs[]);
  bool stalled(u8 IDN, u8 IDs[]);
  void moveUntilLoadLimitHit(u8 IDN, u8 IDs[], s16 Pos[], u16 Speed[], u8 Acc[]);
  void moveUntilLoadLimitHit(u8 ID, s16 pos, u16 speed, u8 acc);
  void moveUntilLoadLimitHit(VectorIdx idx, const int factor, u16 speed, u8 acc);
  void moveUntilLoadLimitHit(u8 IDN, VectorIdx IDXs[], const u8 Factor[], u16 Speed[], u8 Acc[]);
  void move(u8 IDN, u8 IDs[], s16 Pos[], u16 Speed[], u8 Acc[]);
  void move(u8 IDN, VectorIdx IDXs[], const u8 Factor[], u16 Speed[], u8 Acc[]);


public:
  // Motors
  static const u8 INDEX_ID = 1;
  static const u8 MIDDLE_ID = 2;
  static const u8 RING_ID = 3;
  static const u8 THUMB_ID = 4;
  static const u8 THUMB_R_ID = 5;
  constexpr static int SERVO_IDs[5] = { INDEX_ID, MIDDLE_ID, RING_ID, THUMB_ID, THUMB_R_ID };
  constexpr static int SERVOS_SIZE = 5;

public:
  const int RANGE_MIN = 0;
  const int RANGE_MAX = 1;
  int MOTORS_POS_RANGE[5][2] = { { 2048, 2048 }, { 2048, 2048 }, { 2048, 2048 }, { 2048, 2048 }, { 2048, 2048 } };
  int LS_MOTORS_POS_TENDON_INSTALLATION[4] = { 512, 512, 512, 512 };  
  int RS_MOTORS_POS_TENDON_INSTALLATION[4] = { 3583, 3583, 3583, 3583 };

  // Positions
  // Factor convention: 0:Open 100:Closed
  static const int ANY_MATRIX_COLS = 6;  //[INDEX, MIDDLE, RING, THUMB, THUMB_ROT, FACTOR]

  int POWER_FRAMES = 3;
  int POWER_MATRIX[3][ANY_MATRIX_COLS] = {
    { 0, 0, 0, 0, 90, 0 },
    { 0, 0, 0, 60, 90, 30 },
    { 100, 100, 100, 100, 90, 100 }
  };

  int MONKEY_FRAMES = 7;
  int MONKEY_MATRIX[4][ANY_MATRIX_COLS] = {
    { 0, 0, 0, 0, 0, 0 },
    { 50, 50, 50, 50, 0, 70 },
    { 100, 100, 100, 50, 0, 70 },
    { 100, 100, 100, 100, 0, 100 }
  };

  int PINCH_FRAMES = 5;
  int PINCH_GRASP_THUMB_FLEXION = 65;
  int PINCH_MATRIX[7][ANY_MATRIX_COLS] = {
    { 0, 0, 0, PINCH_GRASP_THUMB_FLEXION, 78, 0 },  //60 safepinch
    { 45, 0, 0, PINCH_GRASP_THUMB_FLEXION, 78, 25 },
    { 50, 0, 0, PINCH_GRASP_THUMB_FLEXION, 78, 50 },
    { 55, 0, 0, PINCH_GRASP_THUMB_FLEXION, 78, 75 },
    { 60, 0, 0, PINCH_GRASP_THUMB_FLEXION, 78, 100 }
  };

  const bool DEBUG_LOG = true;

  // Safety
private:
  SafetyLimits limits{
    .MAX_LOAD = 600,      // * 0.02 kg·cm = 12 kg·cm
    .OVER_CURRENT = 150,  // * 6.5 mA = 900 mA
    .OVER_LOAD = 600,     // * 0.02 kg·cm = 12 kg·cm
    .OVER_TEMP = 60       // * 1 °C = 60 °C
  };

  int lastValidTemp[SERVOS_SIZE];

public:
  void safetyFeature();

private:
  SMS_STS st; 
  BluetoothSerial* SerialBT;

private:
  // ** !! SET THE HAND SIDE BEFORE PROGRAMMING !! **
  const HandSide hand_side = HandSide::LeftSide; 

};
