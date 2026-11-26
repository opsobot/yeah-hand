
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
  int TF_MAX_LOAD;  // thumb flexion
  int OVER_CURRENT;
  int OVER_LOAD;
  int OVER_TEMP;
  int HOLD_LOAD;  // torque limit once a movement is finished (holding phase), limits heat
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
    BAR,
    MONKEY,
    PINCH1,
    PINCH2,
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
  /// Grasp named by the start of a command line ("POWER", "BAR", ...); _MAX if none. The caller also checks the exact name.
  static GraspType getGraspTypeByString(const String& cmd);
  /// Command name of a grasp ("POWER", ...).
  static String getGraspStringByType(FingersController::GraspType type);

  /// Opens the servo bus (Serial1, 1 Mbit/s, GPIO 18/19). @param output channel for messages.
  FingersController(Stream* output);

  /// Stream that receives messages (the channel of the command being executed).
  void setOutput(Stream* output) { SerialBT = output; }
  ~FingersController();

  // Setup
  /// INSTALL: working torque at the preset caps, the 4 flexion servos to the tendon installation position, torque kept (no hold).
  void tendonInstallation();
  /// SETCENTER: stores the servo's current position as the centre of its range (servo EEPROM offset).
  void setCenterOfRange(int motor_id);
  //void updateCenterOfRange(IDN, IDXs, RANGE_MIN);
  /// CALIBRATE: drives every finger to its mechanical end stops with low torque limits and stores MOTORS_POS_RANGE (open = RANGE_MIN, closed = RANGE_MAX). Then sets the preset working torque. Blocking, a few seconds.
  void calibrateHand();
  /// Writes a new bus ID into a servo's EEPROM (assembly only, no command calls it).
  void changeID(const int currentId, const int newId);
  /// Stores the current positions as range end (RANGE_MIN or RANGE_MAX) of several fingers.
  void setRangeByCurrentPos(u8 IDN, VectorIdx IDXs[], u8 rangeIndex);
  /// Stores the current position as range end of one finger.
  void setRangeByCurrentPos(VectorIdx IDX, u8 rangeIndex);
  /// SETPINCH: shifts the thumb flexion of PINCH1/PINCH2 by -40..40 (until restart).
  void setPinchOffset(int offset);

  // Serial
  void waitKeyPress(String message) {

    SerialBT->printf("Press any key to continue.. (%s)\n", message.c_str());

    // Clear the serial buffer
    while (SerialBT->available())
      SerialBT->read();
    // Wait for user input
    while (!SerialBT->available())
      ;
  }

  // Conversion
  /// Factor 0..100 (0 open, 100 closed) to servo position, inside the calibrated range.
  int getPosFromFactor(int finger_idx, int factor);
  /// Servo position to factor 0..100; 0 if not calibrated.
  int getFactorFromPos(int finger_idx, s16 pos);
  /// Copies one frame of a grasp matrix: 5 finger factors + the grasp factor of that frame; -1s if out of range.
  void getDataFromTrajectory(GraspType grasp_type, int frame, int data[6]);
  /// Number of frames of a grasp matrix.
  int getTrajectorySize(GraspType grasp_type);
  /// VectorIdx (0..4) to servo bus ID (1..5).
  int getMotorIdByVectorIndex(const VectorIdx idx);
  /// Servo bus ID (1..5) to VectorIdx (0..4); -1 if invalid.
  int getVectorIndexByMotorID(const int motor_id);


  // High Level
  /// Closure command of a grasp (same as grasp()).
  void action(const FingersController::GraspType grasp_type, const int factor);
  /// Moves all fingers to frame 0 of the grasp (start pose) and waits until they stop.
  void prepareGrasp(GraspType grasp_type);
  /// Closure 0..100 of a grasp: interpolates between the two frames around the factor and starts the movement (non-blocking, move()).
  void grasp(const GraspType grasp_type, const int gesture_factor);

  // Info
  /// Servo "moving" flag.
  bool isMoving(const u8 ID);
  /// Present load (0.1 % units, signed).
  int readLoad(const u8 ID);
  /// Present position (0..4095, 0.088 deg per step).
  int readPos(const u8 ID);
  /// Internal temperature, deg C (raw, unfiltered).
  int readTemper(const u8 ID);
  /// Present current (6.5 mA units).
  int readCurrent(const u8 ID);
  /// Pings a servo and prints the result (debug).
  void pingTest(const u8 ID);
  /// Torque limit register (48) of a servo, 0..1000; -1 on error.
  int readMaxTorque(const u8 ID);
  // ROS feedback
  /// Factors 0..100 of the 5 servos (one sync read), for ROSPOS.
  void readFactors(u8 factors[5]);
  void readFeedback(u8 IDN, u8 IDs[], s16 load[], u8 voltage[],
                    u8 temperature[], s16 current[]);
  void readFeedback(s16 load[5], u8 voltage[5],
                    u8 temperature[5], s16 current[5]);

  // Print
  /// INFO: prints position, load, voltage, temperature, moving, current and torque limit of one servo.
  void printFeedback(const int id);
  /// Prints the load of the 5 servos (debug).
  void printLoad();


  // Motion
  /// Declared but not implemented (legacy).
  int moveFingerSync(const int factor, VectorIdx idx);
  int moveFingerSync(const u8 ID, const s16 pos, const u16 speed, const u8 accel);
  void moveFingerAsync(const int factor, VectorIdx idx, u16 speed, u8 acc);
  void moveFingerAsync(const s16 pos, const u8 ID, const u16 speed, const u8 accel);
  void moveAllFingersToMiddlePosition();
  /// Torque on/off of one servo (used by the safety cut).
  void enableTorque(u8 ID, bool enable);
  /// Working torque limit of one servo, capped by the preset (MAX_LOAD, TF_MAX_LOAD for the thumb). Written live, torque stays on.
  void setMaxTorque(const u8 ID, const u16 maxTorque);
  /// Working torque limits of several servos (one sync write), capped by the preset.
  void setMaxTorque(const u8 IDN, u8 IDs[], const u16 MaxTorque[]);

  // Load Limit Motion
  /// Positions of several servos (one sync read).
  void readPositions(u8 IDN, u8 IDs[], s16 positions[]);
  /// True if every servo is within 100 steps of its target.
  bool posMatch(u8 IDN, u8 IDs[], s16 target_pos[]);
  bool posMatch(u8 IDN, u8 pos1[], u8 pos2[], u8 thr);
  bool posMatchWait(u8 IDN, u8 IDs[], s16 target_pos[], const unsigned long timeout);  // Returns false if timed out
  /// Goal = present position, so the servos stop pushing.
  void setCurPosAsTarget(u8 IDN, u8 IDs[]);
  /// True when no servo of the list reports "moving".
  bool stalled(u8 IDN, u8 IDs[]);
  /// Blocking move: restores the working torque, sends the goals, waits until all servos stop (target reached or torque limit hit, max 6 s), then goal = present position and hold torque. Used by ROS, FI..RT, MOVE, CALIBRATE, INSTALL, prepareGrasp.
  void moveUntilLoadLimitHit(u8 IDN, u8 IDs[], s16 Pos[], u16 Speed[], u8 Acc[]);
  void moveUntilLoadLimitHit(u8 ID, s16 pos, u16 speed, u8 acc);
  void moveUntilLoadLimitHit(VectorIdx idx, const int factor, u16 speed, u8 acc);
  void moveUntilLoadLimitHit(u8 IDN, VectorIdx IDXs[], const u8 Factor[], u16 Speed[], u8 Acc[]);
  /// Non-blocking move: restores the working torque, sends the goals and arms the holding phase (holdUpdate()). The servos keep pushing towards the goal. Used by grasp().
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

  int BAR_FRAMES = 3;
  int BAR_MATRIX[3][ANY_MATRIX_COLS] = {
    { 0, 0, 0, 0, 50, 0 },
    { 100, 100, 100, 0, 50, 50 },
    { 100, 100, 100, 100, 50, 100 }
  };

  int MONKEY_FRAMES = 4;  // rows of MONKEY_MATRIX (was 7: frames 4..6 read past the array)
  int MONKEY_MATRIX[4][ANY_MATRIX_COLS] = {
    { 0, 0, 0, 0, 0, 0 },
    { 50, 50, 50, 50, 0, 50 },
    { 100, 100, 100, 50, 0, 75 },
    { 100, 100, 100, 100, 0, 100 }
  };

  int PINCH_GRASP_THUMB_ROTATION = 80;
  int PINCH_GRASP_THUMB_FLEXION = 45;
  int PINCH1_FRAMES = 5;
  int PINCH1_MATRIX[7][ANY_MATRIX_COLS] = {
    { 0,  90, 90, PINCH_GRASP_THUMB_FLEXION, PINCH_GRASP_THUMB_ROTATION, 0 },  //60 safepinch
    { 45, 90, 90, PINCH_GRASP_THUMB_FLEXION, PINCH_GRASP_THUMB_ROTATION, 25 },
    { 50, 90, 90, PINCH_GRASP_THUMB_FLEXION, PINCH_GRASP_THUMB_ROTATION, 50 },
    { 55, 90, 90, PINCH_GRASP_THUMB_FLEXION, PINCH_GRASP_THUMB_ROTATION, 75 },
    { 60, 90, 90, PINCH_GRASP_THUMB_FLEXION, PINCH_GRASP_THUMB_ROTATION, 100 }
  };

  int PINCH2_FRAMES = 5;
  int PINCH2_MATRIX[7][ANY_MATRIX_COLS] = {
    {  0, 0, 0, PINCH_GRASP_THUMB_FLEXION, PINCH_GRASP_THUMB_ROTATION, 0 },  //60 safepinch
    { 45, 0, 0, PINCH_GRASP_THUMB_FLEXION, PINCH_GRASP_THUMB_ROTATION, 25 },
    { 50, 0, 0, PINCH_GRASP_THUMB_FLEXION, PINCH_GRASP_THUMB_ROTATION, 50 },
    { 55, 0, 0, PINCH_GRASP_THUMB_FLEXION, PINCH_GRASP_THUMB_ROTATION, 75 },
    { 60, 0, 0, PINCH_GRASP_THUMB_FLEXION, PINCH_GRASP_THUMB_ROTATION, 100 }
  };

  const bool DEBUG_LOG = true;

  // Safety
private:
  SafetyLimits limits{
    // Defaults = preset 5A (setPowerPreset() overwrites them at start-up)
    .MAX_LOAD = 330,      // * 0.02 kg·cm * 330 = 6.6 kg·cm
    .TF_MAX_LOAD = 300,   // * 0.02 kg·cm * 300 = 6 kg·cm (thumb flexion)
    .OVER_CURRENT = 150,  // * 6.5 mA * 150 = 975 mA
    .OVER_LOAD = 600,     // * 0.02 kg·cm * 600 = 12 kg·cm
    .OVER_TEMP = 60,      // * 1 °C * 60 = 60 °C
    .HOLD_LOAD = 80       // * 0.02 kg·cm * 80 = 1.6 kg·cm (holding phase, Nov 2026 thermal tests)
  };

  int lastValidTemp[SERVOS_SIZE] = { -1, -1, -1, -1, -1 };  // index = servo id - 1
  // Median filter of the temperature readings (5 s window, one reading per second per servo)
  static const int TEMP_WIN = 5;
  int tempWin[SERVOS_SIZE][TEMP_WIN] = {};
  int tempWinPos[SERVOS_SIZE] = { 0, 0, 0, 0, 0 };
  int tempWinCount[SERVOS_SIZE] = { 0, 0, 0, 0, 0 };
  bool safetyTripped[SERVOS_SIZE] = { false, false, false, false, false };
  bool hotWarned[SERVOS_SIZE] = { false, false, false, false, false };
  // Last valid telemetry per servo (index = id - 1), returned when a sync read is missed
  s16 fbLoad[SERVOS_SIZE] = { 0, 0, 0, 0, 0 };
  u8 fbVolt[SERVOS_SIZE] = { 0, 0, 0, 0, 0 };
  u8 fbTemp[SERVOS_SIZE] = { 0, 0, 0, 0, 0 };
  s16 fbCurrent[SERVOS_SIZE] = { 0, 0, 0, 0, 0 };

public:
  /// Early warning: WARN_TEMP_MARGIN degrees below OVER_TEMP the hand sends "WARN HOT"
  /// (once, with 3 C hysteresis) and refuses new closing commands ("ERR HOT"), so the
  /// robot can put the object down before the OVER_TEMP torque cut.
  static const int WARN_TEMP_MARGIN = 5;  // 55 C with OVER_TEMP 60; restart at 52 C
  int warnTemp() const { return limits.OVER_TEMP - WARN_TEMP_MARGIN; }
  /// Id of the hottest servo at or above warnTemp(), 0 if none; its temperature in *temp.
  int hotServo(int *temp) const;
  /// Last valid temperature of a servo (id 1..5) read by safetyFeature(), -1 if none yet.
  int lastTemp(int id) const { return (id >= 1 && id <= SERVOS_SIZE) ? lastValidTemp[id - 1] : -1; }
  /// Called every second: reads current, load and temperature of each servo; WARN HOT / INFO COOL on the median temperature; cuts the torque on OVER_CURRENT, OVER_LOAD or OVER_TEMP and re-enables it when the fault is gone.
  void safetyFeature();

  /// Holding phase: when the fingers stop moving after a movement, the torque limit of
  /// every servo drops to HOLD_LOAD (if lower than its working torque); it is restored
  /// before the next movement. Call holdUpdate() from the main loop.
  void holdUpdate();
  bool holding() const { return hold_active; }
  /// Start the holding check now (the hold is applied when all fingers are still).
  void requestHold() {
    hold_pending = true;
    hold_check_at = millis();
  }
  bool anySafetyTripped() const {
    for (int i = 0; i < SERVOS_SIZE; i++)
      if (safetyTripped[i]) return true;
    return false;
  }

private:
  SMS_STS st;
  Stream* SerialBT;  // current output channel (Bluetooth or USB)

public:
  /// Power presets: torque limits that keep the hand's supply current within a budget.
  /// Each preset is validated by measurement before it is added (see technical file).
  struct PowerPreset {
    const char *name;     // nominal max supply current at 12 V
    SafetyLimits limits;  // every limit is set by the preset
  };
  // Fields: MAX_LOAD, TF_MAX_LOAD (thumb flexion), OVER_CURRENT, OVER_LOAD, OVER_TEMP, HOLD_LOAD
  // Units: load 0.02 kg·cm, current 6.5 mA, temperature 1 °C
  static constexpr PowerPreset POWER_PRESETS[] = {
    { "5A", { 330, 300, 150, 600, 60, 80 } },   // standard, factory default; hold 80: test H 05/11/2026 (all covers, about 45 C)
    { "1A", { 200, 200, 150, 250, 60, 80 } },  // supplies limited to about 1 A (e.g. 0.9 A tool outputs)
    { "TEST", { 600, 600, 250, 900, 60, 80 } },  // manufacturer/research tests only (limits up to 600 on every finger, OVER_CURRENT 1625 mA), never for delivery
  };
  static constexpr int POWER_PRESETS_COUNT = sizeof(POWER_PRESETS) / sizeof(POWER_PRESETS[0]);
  static const PowerPreset *findPowerPreset(const String &name);
  /// Applies the limits of a preset (call before calibrateHand()). False if the name is unknown.
  bool setPowerPreset(const String &name);
  const char *powerPresetName() const { return power_preset_name; }
  const SafetyLimits &safetyLimits() const { return limits; }
  /// Test only (service mode): change HOLD_LOAD until restart, not saved.
  void setHoldLoad(int value) { limits.HOLD_LOAD = value; }

  /// Set from the model stored at provisioning, before calibrateHand().
  void setHandSide(HandSide side) { hand_side = side; }

private:
  HandSide hand_side = HandSide::LeftSide;
  const char *power_preset_name = "5A";
  /// Torque capped by the preset: TF_MAX_LOAD for the thumb flexion, MAX_LOAD for the others.
  u16 respectTorqueLimits(u8 ID, u16 torque) const;

  // Holding phase state
  /// Working torque per servo id (1..5), set by setMaxTorque(); 0 until CALIBRATE or INSTALL.
  u16 work_torque[SERVOS_SIZE + 1] = { 0, 0, 0, 0, 0, 0 };  // working torque per servo id (1..5)
  bool hold_pending = false;    // a movement was started, hold when it stops
  bool hold_active = false;     // hold torque applied
  unsigned long hold_check_at = 0;
  /// Raw sync write of torque limit registers, no preset cap, torque stays on.
  void writeTorqueLimits(u8 IDN, u8 IDs[], const u16 torque[]);  // no torque off/on
  /// Before a movement: if the hold torque is active, writes back work_torque to all 5 servos.
  void restoreWorkTorque(u8 IDN, u8 IDs[]);
  /// Lowers all 5 servos to HOLD_LOAD (never above their working torque) and marks the hold active.
  void applyHoldTorque();
};
