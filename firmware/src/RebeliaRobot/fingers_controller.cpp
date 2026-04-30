
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

#include "fingers_controller.h"

#include "calibration_storage.h"

FingersController::GraspType FingersController::getGraspTypeByString(const String& cmd) {
  String normalized = cmd;
  normalized.trim();
  normalized.toUpperCase();

  if (normalized == "POWER") {
    return FingersController::GraspType::POWER;
  } else if (normalized == "POWERSMALL") {
    return FingersController::GraspType::POWERSMALL;
  } else if (normalized == "POWERTOOL") {
    return FingersController::GraspType::POWERTOOL;
  } else if (normalized == "MONKEY") {
    return FingersController::GraspType::MONKEY;
  } else if (normalized == "PINCH") {
    return FingersController::GraspType::PINCH;
  } else if (normalized == "RELAX") {
    return FingersController::GraspType::RELAX;
  } else if (normalized == "INDEX") {
    return FingersController::GraspType::INDEX;
  } else if (normalized == "MIDDLE") {
    return FingersController::GraspType::MIDDLE;
  } else if (normalized == "RING") {
    return FingersController::GraspType::RING;
  } else if (normalized == "THUMB") {
    return FingersController::GraspType::THUMB;
  } else if (normalized == "THUMB_ROT") {
    return FingersController::GraspType::THUMB_ROT;
  }

  return FingersController::GraspType::_MAX;
}

String FingersController::getGraspStringByType(FingersController::GraspType type) {

  String text;

  switch (type) {
    case FingersController::GraspType::POWER:
      text = "POWER\r\n";
      break;
    case FingersController::GraspType::POWERSMALL:
      text = "POWERSMALL\r\n";
      break;
    case FingersController::GraspType::POWERTOOL:
      text = "POWERTOOL\r\n";
      break;
    case FingersController::GraspType::MONKEY:
      text = "MONKEY\r\n";
      break;
    case FingersController::GraspType::PINCH:
      text = "PINCH\r\n";
      break;
    case FingersController::GraspType::RELAX:
      text = "RELAX\r\n";
      break;
    case FingersController::GraspType::INDEX:
      text = "INDEX\r\n";
      break;
    case FingersController::GraspType::MIDDLE:
      text = "MIDDLE\r\n";
      break;
    case FingersController::GraspType::RING:
      text = "RING\r\n";
      break;
    case FingersController::GraspType::THUMB:
      text = "THUMB\r\n";
      break;
    case FingersController::GraspType::THUMB_ROT:
      text = "THUMB_ROT\r\n";
      break;
    case FingersController::GraspType::_MAX:
      text = "INVALID_COMMAND\r\n";
      break;
  }

  return text;
}

const char* FingersController::getVectorName(VectorIdx idx) {
  switch (idx) {
    case VectorIdx::Index:
      return "INDEX";
    case VectorIdx::Middle:
      return "MIDDLE";
    case VectorIdx::Ring:
      return "RING";
    case VectorIdx::Thumb:
      return "THUMB";
    case VectorIdx::ThumbRot:
      return "THUMB_ROT";
    default:
      return "UNKNOWN";
  }
}

void FingersController::enableTorque(u8 ID, bool enable) {
  SerialBT->printf("[FingersController] Torque %s for servo %u\n", enable ? "enabled" : "disabled", ID);
  st.EnableTorque(ID, enable);
}

FingersController::FingersController(BluetoothSerial* _serialBT)
  : SerialBT(_serialBT) {
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  while (!Serial1) {
    SerialBT->println("ST3215 serial Not ready");
    delay(100);
  }
  SerialBT->println("ST3215 serial Ready!");
}

FingersController::~FingersController() {
}

void FingersController::tendonInstallation() {
  SerialBT->println("[FingersController] Tendon installation posture start");
  for (int IDX = VectorIdx::Index; IDX <= VectorIdx::Thumb; IDX++) {
    int ID = getMotorIdByVectorIndex((VectorIdx)IDX);
    SerialBT->printf("[FingersController] Tendon install move %s servo=%d target=%d\n",
                     getVectorName((VectorIdx)IDX), ID, MOTORS_POS_TENDON_INSTALLATION[IDX]);
    moveUntilLoadLimitHit(ID, MOTORS_POS_TENDON_INSTALLATION[IDX], 2000, 50);
  }
  SerialBT->println("[FingersController] Tendon installation posture complete");
}

void FingersController::setCenterOfRange(int motor_id) {
  int result = st.CalibrationOfs(motor_id);
  if (result == 0) {
    SerialBT->println("Successfully calibrated center of range! ");
  } else {
    SerialBT->printf("Failed center of range calibration! Error: %d\n", result);
  }
}

bool FingersController::calibrateHand() {
  SerialBT->println("[FingersController] Calibration sequence started");

  u8 IDN = 5;
  u8 IDN_Flex = 4;
  VectorIdx IDXs[IDN] = { VectorIdx::Index, VectorIdx::Middle, VectorIdx::Ring, VectorIdx::Thumb, VectorIdx::ThumbRot };
  u8 IDs[IDN] = { INDEX_ID, MIDDLE_ID, RING_ID, THUMB_ID, THUMB_R_ID };
  s16 START_POS_FOR_OPENING[IDN] = { 1000, 1000, 1000, 1200, 50 };
  s16 START_POS_FOR_CLOSING[IDN] = { 2500, 2500, 2500, 1200, 4045 };
  s16 END_POS_FOR_OPENING[IDN] = { 50, 50, 50, 50, 50 };
  s16 END_POS_FOR_CLOSING[IDN] = { 4045, 4045, 4045, 4045, 4045 };
  u16 Speed[IDN] = { 3000, 3000, 3000, 3000, 3000 };
  u8 Acc[IDN] = { 250, 250, 250, 250, 250 };

  // Pre-position fingers for opening
  setMaxTorque(IDN, IDs, (u16[]){ 500, 500, 500, 500, 500 });
  moveUntilLoadLimitHit(IDN, IDs, START_POS_FOR_OPENING, Speed, Acc);

  // Open Hand calibration
  setMaxTorque(IDN, IDs, (u16[]){ 200, 200, 200, 200, 200 });
  moveUntilLoadLimitHit(IDN, IDs, END_POS_FOR_OPENING, Speed, Acc);
  moveUntilLoadLimitHit(THUMB_R_ID, readPos(THUMB_R_ID) + 100, 4000, 250);  // move slightly back
  setRangeByCurrentPos(IDN, IDXs, RANGE_MIN);

  // Pre-position fingers for closing
  setMaxTorque(IDN_Flex, IDs, (u16[]){ 500, 500, 500, 500 });
  moveUntilLoadLimitHit(IDN_Flex, IDs, START_POS_FOR_CLOSING, Speed, Acc);

  // Closed flexion calibration
  setMaxTorque(IDN_Flex, IDs, (u16[]){ 300, 300, 300, 300 });
  moveUntilLoadLimitHit(IDN_Flex, IDs, END_POS_FOR_CLOSING, Speed, Acc);
  setRangeByCurrentPos(IDN_Flex, IDXs, RANGE_MAX);
  // Rotated Thumb Calibration
  moveUntilLoadLimitHit(VectorIdx::Thumb, 0, 4000, 250);
  setMaxTorque(THUMB_R_ID, 500);
  moveUntilLoadLimitHit(THUMB_R_ID, 4045, 4000, 250);
  moveUntilLoadLimitHit(THUMB_R_ID, readPos(THUMB_R_ID) - 100, 4000, 250);  // move slightly back
  setRangeByCurrentPos(VectorIdx::ThumbRot, RANGE_MAX);

  // Open all
  setMaxTorque(IDN, IDs, (u16[]){ 500, 500, 500, 500, 500 });
  moveUntilLoadLimitHit(IDN, IDXs, (u8[]){ 0, 0, 0, 0, 0 }, Speed, Acc);

  // Pinch Grasp Calibration
  moveUntilLoadLimitHit(VectorIdx::Index, 50, 4000, 200);
  moveUntilLoadLimitHit(VectorIdx::ThumbRot, PINCH_MATRIX[0][VectorIdx::ThumbRot], 4000, 200);
  moveUntilLoadLimitHit(VectorIdx::Thumb, 60, 4000, 200);
  setMaxTorque(THUMB_ID, 125);
  moveUntilLoadLimitHit(VectorIdx::Thumb, 100, 4000, 50);
  auto pinchThumbFactor = getFactorFromPos(VectorIdx::Thumb, readPos(THUMB_ID));
  auto pinchThumbRotFactor = PINCH_MATRIX[0][VectorIdx::ThumbRot] + 5;
  for (int i = 0; i < PINCH_FRAMES; i++) {
    if (i >= 2) pinchThumbFactor += 5;
    PINCH_MATRIX[i][VectorIdx::Thumb] = pinchThumbFactor;
    PINCH_MATRIX[i][VectorIdx::ThumbRot] = pinchThumbRotFactor;
  }
  SerialBT->println("Pinch Grasp Calibrated.");
  
  // Open
  moveUntilLoadLimitHit(IDN, IDXs, (u8[]){ 0, 0, 0, 0, 0 }, Speed, Acc);

  auto degreeRange = [](int min, int max) {
    return (int)(((double)abs(max - min) / 4096.0) * 360.0);
  };

  SerialBT->println("All servos calibrated!");
  auto iMin = getPosFromFactor(VectorIdx::Index, 0);
  auto iMax = getPosFromFactor(VectorIdx::Index, 100);
  auto iDeg = degreeRange(iMin, iMax);
  auto mMin = getPosFromFactor(VectorIdx::Middle, 0);
  auto mMax = getPosFromFactor(VectorIdx::Middle, 100);
  auto mDeg = degreeRange(mMin, mMax);
  auto rMin = getPosFromFactor(VectorIdx::Ring, 0);
  auto rMax = getPosFromFactor(VectorIdx::Ring, 100);
  auto rDeg = degreeRange(rMin, rMax);
  auto tMin = getPosFromFactor(VectorIdx::Thumb, 0);
  auto tMax = getPosFromFactor(VectorIdx::Thumb, 100);
  auto tDeg = degreeRange(tMin, tMax);
  auto trMin = getPosFromFactor(VectorIdx::ThumbRot, 0);
  auto trMax = getPosFromFactor(VectorIdx::ThumbRot, 100);
  auto trDeg = degreeRange(trMin, trMax);

  SerialBT->printf("I(%d,%d)[%d deg]\n", iMin, iMax, iDeg);
  SerialBT->printf("M(%d,%d)[%d deg]\n", mMin, mMax, mDeg);
  SerialBT->printf("R(%d,%d)[%d deg]\n", rMin, rMax, rDeg);
  SerialBT->printf("T(%d,%d)[%d deg]\n", tMin, tMax, tDeg);
  SerialBT->printf("TR (%d,%d)[%d deg]\n", trMin, trMax, trDeg);
  SerialBT->println("[FingersController] Calibration sequence completed");

  return true;  // Calibration successful

}

int FingersController::getPosFromFactor(int finger_idx, int factor) {
  if (finger_idx < VectorIdx::Index || finger_idx > VectorIdx::ThumbRot) {
    SerialBT->printf("[FingersController] Percent map rejected: invalid finger index=%d\n", finger_idx);
    return 2048;
  }

  const int clamped_factor = constrain(factor, 0, 100);
  if (clamped_factor != factor) {
    SerialBT->printf("[FingersController] Percent clamped %s from %d to %d\n",
                     getVectorName(static_cast<VectorIdx>(finger_idx)), factor, clamped_factor);
  }

  const int open_position = MOTORS_POS_RANGE[finger_idx][RANGE_MIN];
  const int closed_position = MOTORS_POS_RANGE[finger_idx][RANGE_MAX];
  const long travel = static_cast<long>(closed_position) - static_cast<long>(open_position);
  int absolute_pos_value = static_cast<int>(open_position + ((travel * clamped_factor) / 100L));
  absolute_pos_value = limit(absolute_pos_value, open_position, closed_position);

  SerialBT->printf("[FingersController] Percent map %s servo=%d percent=%d open=%d closed=%d target=%d\n",
                   getVectorName(static_cast<VectorIdx>(finger_idx)),
                   getMotorIdByVectorIndex(static_cast<VectorIdx>(finger_idx)),
                   clamped_factor,
                   open_position,
                   closed_position,
                   absolute_pos_value);
  return absolute_pos_value;
}

int FingersController::getFactorFromPos(int finger_idx, s16 pos) {
  int factor = map(pos, MOTORS_POS_RANGE[finger_idx][RANGE_MIN], MOTORS_POS_RANGE[finger_idx][RANGE_MAX], 0, 100);
  return limit(factor, 0, 100);
}

int FingersController::getTrajectorySize(GraspType grasp_type) {

  switch (grasp_type) {
    case POWER:
    case POWERSMALL:
    case POWERTOOL:
    case RELAX:
      return POWER_FRAMES;
    case MONKEY:
      return MONKEY_FRAMES;
    case PINCH:
      return PINCH_FRAMES;
  }
  return -1;  //grasp_type not found
}

void FingersController::getDataFromTrajectory(GraspType graspType, int frame, int data[6]) {

  // Get the correct matrix based on grasp_type
  const int(*matrix)[ANY_MATRIX_COLS] = nullptr;  // Pointer to 2D array with 6 columns
  int frameCount = 0;

  switch (graspType) {
    case POWER:
    case POWERSMALL:
    case POWERTOOL:
    case RELAX:
      matrix = POWER_MATRIX;
      frameCount = POWER_FRAMES;
      break;
    case MONKEY:
      matrix = MONKEY_MATRIX;
      frameCount = MONKEY_FRAMES;
      break;
    case PINCH:
      matrix = PINCH_MATRIX;
      frameCount = PINCH_FRAMES;
      break;
    default:
      // Error: unknown grasp type
      for (int i = 0; i < ANY_MATRIX_COLS; i++) data[i] = -1;
      return;
  }

  // Check frame bounds
  if (frame < 0 || frame >= frameCount) {
    // Error: frame out of range
    for (int i = 0; i < ANY_MATRIX_COLS; i++) data[i] = -1;
    return;
  }

  // Copy the data (all 6 finger values for this frame)
  for (int col = 0; col < ANY_MATRIX_COLS; col++) {
    data[col] = matrix[frame][col];
  }
}

void FingersController::prepareGrasp(GraspType graspType) {

  SerialBT->println("Preparing Grasp");
  u8 IDN = 5;
  u8 IDs[IDN] = { INDEX_ID, MIDDLE_ID, RING_ID, THUMB_ID, THUMB_R_ID };
  s16 Pos[IDN];
  u16 Speed[IDN] = { 2000, 2000, 2000, 2000, 2000 };
  u8 Acc[IDN] = { 200, 200, 200, 200, 200 };

  int data[ANY_MATRIX_COLS];
  getDataFromTrajectory(graspType, 0, data);
  for (int idx = 0; idx <= VectorIdx::ThumbRot; idx++) {
    Pos[idx] = getPosFromFactor(idx, data[idx]);
  }

  moveUntilLoadLimitHit(IDN, IDs, Pos, Speed, Acc);
  delay(10);

  SerialBT->println("Grasp Prepared");
}

void FingersController::grasp(const GraspType graspType, const int graspFactor) {
  SerialBT->printf("[FingersController] Grasp request type=%d factor=%d\n", graspType, graspFactor);

  u8 IDN = 5;
  u8 IDs[IDN] = { INDEX_ID, MIDDLE_ID, RING_ID, THUMB_ID, THUMB_R_ID };
  s16 Pos[IDN];
  u16 Speed[IDN] = { 2000, 2000, 2000, 2000, 2000 };
  u8 Acc[IDN] = { 200, 200, 200, 200, 200 };

  bool positionSet = false;
  for (size_t frame = 0; frame < getTrajectorySize(graspType) - 1; frame++) {
    int currFrame[ANY_MATRIX_COLS];
    int nextFrame[ANY_MATRIX_COLS];
    getDataFromTrajectory(graspType, frame, currFrame);
    getDataFromTrajectory(graspType, frame + 1, nextFrame);
    int currGraspFactor = currFrame[VectorIdx::Factor];
    int nextGraspFactor = nextFrame[VectorIdx::Factor];
    if (graspFactor >= currGraspFactor && graspFactor <= nextGraspFactor) {
      for (int col = VectorIdx::Index; col <= VectorIdx::ThumbRot; col++) {  // iterate fingers
        int currPos = getPosFromFactor(col, currFrame[col]);
        int nextPos = getPosFromFactor(col, nextFrame[col]);
        Pos[col] = map(graspFactor, currGraspFactor, nextGraspFactor, currPos, nextPos);
        Pos[col] = limit(Pos[col], currPos, nextPos);
      }
      positionSet = true;
      break;
    }
  }

  if (positionSet) {
    // moveUntilLoadLimitHit(IDN, IDs, Pos, Speed, Acc);
    move(IDN, IDs, Pos, Speed, Acc);
  }
}

void FingersController::buildPositionsFromFactors(const uint8_t factors[5], s16 positions[5]) {
  for (int idx = VectorIdx::Index; idx <= VectorIdx::ThumbRot; ++idx) {
    positions[idx] = static_cast<s16>(getPosFromFactor(idx, factors[idx]));
  }
}

bool FingersController::moveFingerPercent(VectorIdx idx, int percent, u16 speed, u8 acc) {
  if (idx < VectorIdx::Index || idx > VectorIdx::ThumbRot) {
    SerialBT->printf("[FingersController] Percent move rejected: invalid finger index=%d\n", idx);
    return false;
  }

  const int clamped_percent = constrain(percent, 0, 100);
  const int target = getPosFromFactor(idx, clamped_percent);
  const int servo_id = getMotorIdByVectorIndex(idx);
  SerialBT->printf("[FingersController] Percent move %s percent=%d servo=%d target=%d speed=%u accel=%u\n",
                   getVectorName(idx), clamped_percent, servo_id, target, speed, acc);
  moveUntilLoadLimitHit(static_cast<u8>(servo_id), static_cast<s16>(target), speed, acc);
  return true;
}

bool FingersController::movePosePercent(const uint8_t factors[5], u16 speed, u8 acc) {
  const u8 IDN = 5;
  u8 IDs[IDN] = { INDEX_ID, MIDDLE_ID, RING_ID, THUMB_ID, THUMB_R_ID };
  s16 positions[IDN];
  u16 speed_values[IDN] = { speed, speed, speed, speed, speed };
  u8 acc_values[IDN] = { acc, acc, acc, acc, acc };

  buildPositionsFromFactors(factors, positions);
  SerialBT->printf("[FingersController] Pose percent move factors=[%u,%u,%u,%u,%u] speed=%u accel=%u\n",
                   factors[0], factors[1], factors[2], factors[3], factors[4], speed, acc);
  move(IDN, IDs, positions, speed_values, acc_values);
  return true;
}

bool FingersController::executeTimedStep(const uint8_t factors[5], uint16_t speed, uint8_t accel,
                                         unsigned long move_timeout_ms, unsigned long hold_time_ms) {
  const u8 IDN = 5;
  u8 IDs[IDN] = { INDEX_ID, MIDDLE_ID, RING_ID, THUMB_ID, THUMB_R_ID };
  s16 positions[IDN];
  u16 speed_values[IDN] = { speed, speed, speed, speed, speed };
  u8 accel_values[IDN] = { accel, accel, accel, accel, accel };

  buildPositionsFromFactors(factors, positions);

  SerialBT->printf("[FingersController] Timed step -> [%u,%u,%u,%u,%u], speed=%u, accel=%u, timeout=%lu, hold=%lu\n",
                   factors[0], factors[1], factors[2], factors[3], factors[4],
                   speed, accel, move_timeout_ms, hold_time_ms);

  move(IDN, IDs, positions, speed_values, accel_values);
  const bool reached = posMatchWait(IDN, IDs, positions, move_timeout_ms);

  if (!reached) {
    SerialBT->println("[FingersController] Timed step timeout while waiting for target");
  }

  if (hold_time_ms > 0) {
    delay(hold_time_ms);
  }

  return reached;
}

int FingersController::getMotorIdByVectorIndex(const VectorIdx idx) {
  return idx + 1;
}

int FingersController::getRangeMin(VectorIdx idx) const {
  return MOTORS_POS_RANGE[idx][RANGE_MIN];
}

int FingersController::getRangeMax(VectorIdx idx) const {
  return MOTORS_POS_RANGE[idx][RANGE_MAX];
}

void FingersController::setCalibratedRange(VectorIdx idx, int open_position, int closed_position) {
  MOTORS_POS_RANGE[idx][RANGE_MIN] = constrain(open_position, MOTOR_POS_ABSOLUTE_MIN, MOTOR_POS_ABSOLUTE_MAX);
  MOTORS_POS_RANGE[idx][RANGE_MAX] = constrain(closed_position, MOTOR_POS_ABSOLUTE_MIN, MOTOR_POS_ABSOLUTE_MAX);
  SerialBT->printf("[FingersController] Safe range set %s servo=%d open=%d closed=%d travel=%d\n",
                   getVectorName(idx),
                   getMotorIdByVectorIndex(idx),
                   MOTORS_POS_RANGE[idx][RANGE_MIN],
                   MOTORS_POS_RANGE[idx][RANGE_MAX],
                   abs(MOTORS_POS_RANGE[idx][RANGE_MAX] - MOTORS_POS_RANGE[idx][RANGE_MIN]));
}

bool FingersController::pingServo(u8 ID) {
  const int result = st.Ping(ID);
  if (result != -1) {
    SerialBT->printf("[FingersController] Ping OK servo=%u response=%d\n", ID, result);
    Serial.printf("[FingersController] Ping OK servo=%u response=%d\n", ID, result);
    return true;
  }

  SerialBT->printf("[FingersController] Ping FAILED servo=%u\n", ID);
  Serial.printf("[FingersController] Ping FAILED servo=%u\n", ID);
  return false;
}

bool FingersController::calibrationMoveRaw(u8 ID, s16 position, u16 speed, u8 acc, u16 torque) {
  if (ID < 1 || ID > MAX_SERVOS) {
    SerialBT->printf("[FingersController] Calibration raw move rejected: servo %u is outside 1-%d\n", ID, MAX_SERVOS);
    return false;
  }

  const s16 target = constrain(position, MOTOR_POS_ABSOLUTE_MIN, MOTOR_POS_ABSOLUTE_MAX);
  const int previousTorque = readMaxTorque(ID);

  SerialBT->printf("[FingersController] Calibration raw move servo=%u target=%d speed=%u accel=%u torque=%u\n",
                   ID, target, speed, acc, torque);

  setMaxTorque(ID, torque);
  moveUntilLoadLimitHit(ID, target, speed, acc);

  if (previousTorque >= 0 && previousTorque <= 1000) {
    setMaxTorque(ID, previousTorque);
    SerialBT->printf("[FingersController] Calibration raw move restored servo=%u torque=%d\n", ID, previousTorque);
  }

  const int finalPos = readPos(ID);
  const int finalLoad = readLoad(ID);
  SerialBT->printf("[FingersController] Calibration raw move complete servo=%u final=%d load=%d\n",
                   ID, finalPos, finalLoad);
  return true;
}

bool FingersController::calibrationJog(u8 ID, int delta, u16 speed, u8 acc, u16 torque) {
  const int current = readPos(ID);
  if (current < MOTOR_POS_ABSOLUTE_MIN || current > MOTOR_POS_ABSOLUTE_MAX) {
    SerialBT->printf("[FingersController] Calibration jog rejected: servo=%u unreadable current=%d\n", ID, current);
    return false;
  }

  const int target = constrain(current + delta, MOTOR_POS_ABSOLUTE_MIN, MOTOR_POS_ABSOLUTE_MAX);
  SerialBT->printf("[FingersController] Calibration jog servo=%u current=%d delta=%d target=%d\n",
                   ID, current, delta, target);
  return calibrationMoveRaw(ID, target, speed, acc, torque);
}

bool FingersController::calibrationMoveFactor(VectorIdx idx, int factor, u16 speed, u8 acc, u16 torque) {
  const int constrainedFactor = constrain(factor, 0, 100);
  const int target = getPosFromFactor(idx, constrainedFactor);
  SerialBT->printf("[FingersController] Calibration factor move %s servo=%d factor=%d target=%d\n",
                   getVectorName(idx),
                   getMotorIdByVectorIndex(idx),
                   constrainedFactor,
                   target);
  return calibrationMoveRaw(getMotorIdByVectorIndex(idx), target, speed, acc, torque);
}

bool FingersController::calibrationWiggle(u8 ID, int delta, u16 speed, u8 acc, u16 torque) {
  const int start = readPos(ID);
  if (start < MOTOR_POS_ABSOLUTE_MIN || start > MOTOR_POS_ABSOLUTE_MAX) {
    SerialBT->printf("[FingersController] Calibration identify rejected: servo=%u unreadable current=%d\n", ID, start);
    return false;
  }

  const int first = constrain(start + delta, MOTOR_POS_ABSOLUTE_MIN, MOTOR_POS_ABSOLUTE_MAX);
  SerialBT->printf("[FingersController] Calibration identify servo=%u start=%d wiggle=%d first=%d\n",
                   ID, start, delta, first);
  if (!calibrationMoveRaw(ID, first, speed, acc, torque)) {
    return false;
  }
  delay(300);
  return calibrationMoveRaw(ID, start, speed, acc, torque);
}

// void FingersController::moveFingerAsync(const int factor, VectorIdx idx, u16 speed, u8 acc) {
//   s16 pos = map(factor, 0, 100, MOTORS_POS_RANGE[idx][RANGE_MIN], MOTORS_POS_RANGE[idx][RANGE_MAX]);
//   pos = limit(pos, MOTORS_POS_RANGE[idx][RANGE_MIN], MOTORS_POS_RANGE[idx][RANGE_MAX]);
//   moveFingerAsync(pos, getMotorIdByVectorIndex(idx), speed, acc);
// }

// void FingersController::moveFingerAsync(const s16 pos, const u8 ID, const u16 speed, const u8 acc) {
//   u8 IDN = 1;
//   u8 IDs[IDN] = { ID };
//   u16 Speed[IDN] = { speed };
//   u8 Acc[IDN] = { acc };
//   s16 Pos[IDN] = { pos };
//   st.SyncWritePosEx(IDs, IDN, Pos, Speed, Acc);
// }

int FingersController::moveFingerSync(const int factor, VectorIdx idx) {
  const s16 pos = getPosFromFactor(idx, factor);
  return moveFingerSync(getMotorIdByVectorIndex(idx), pos, 2000, 200);
}

int FingersController::moveFingerSync(const u8 ID, const s16 pos, const u16 speed, const u8 accel) {
  moveUntilLoadLimitHit(ID, pos, speed, accel);
  return readPos(ID);
}

void FingersController::moveFingerAsync(const int factor, VectorIdx idx, u16 speed, u8 acc) {
  const s16 pos = getPosFromFactor(idx, factor);
  SerialBT->printf("[FingersController] Async finger move %s factor=%d target=%d\n",
                   getVectorName(idx), factor, pos);
  moveFingerAsync(pos, getMotorIdByVectorIndex(idx), speed, acc);
}

void FingersController::moveFingerAsync(const s16 pos, const u8 ID, const u16 speed, const u8 acc) {
  SerialBT->printf("[FingersController] Async raw move servo=%u target=%d speed=%u accel=%u\n", ID, pos, speed, acc);
  u8 IDs[1] = { ID };
  s16 Pos[1] = { pos };
  u16 Speed[1] = { speed };
  u8 Acc[1] = { acc };
  st.SyncWritePosEx(IDs, 1, Pos, Speed, Acc);
}

void FingersController::moveAllFingersToMiddlePosition() {
  for (int id = 1; id <= 5; id++) {
    SerialBT->printf("Moving to middle position id: %d..", id);
    moveFingerAsync(2048, id, 100, 200);
    SerialBT->println(".. moved!");
  }
}

int FingersController::readPos(const u8 ID) {
  const int pos = st.ReadPos(ID);
  if (pos < 0) {
    SerialBT->printf("[FingersController] Read position failed servo=%u result=%d\n", ID, pos);
  }
  return pos;
}

bool FingersController::isMoving(const u8 ID) {
  bool moving = st.ReadMove(ID) == 1 ? true : false;
  return moving;
}

void FingersController::printLoad() {
  SerialBT->print("LOAD : TR: ");
  SerialBT->print(st.ReadLoad(THUMB_R_ID));
  SerialBT->print("  TF: ");
  SerialBT->print(st.ReadLoad(THUMB_ID));
  SerialBT->print("  IF: ");
  SerialBT->print(st.ReadLoad(INDEX_ID));
  SerialBT->print("  MF: ");
  SerialBT->print(st.ReadLoad(MIDDLE_ID));
  SerialBT->print("  RLF: ");
  SerialBT->println(st.ReadLoad(RING_ID));
}

int FingersController::readLoad(const u8 ID) {
  return st.ReadLoad(ID);
}

void FingersController::action(const FingersController::GraspType grasp_type, const int factor) {
  SerialBT->printf("[FingersController] Action request type=%d factor=%d\n", grasp_type, factor);
  switch (grasp_type) {
    case FingersController::GraspType::POWER:
    case FingersController::GraspType::POWERSMALL:
    case FingersController::GraspType::POWERTOOL:
    case FingersController::GraspType::MONKEY:
    case FingersController::GraspType::PINCH:
    case FingersController::GraspType::RELAX:
      grasp(grasp_type, factor);
      break;
    case FingersController::GraspType::INDEX:
      moveUntilLoadLimitHit(VectorIdx::Index, factor, 2000, 200);
      break;
    case FingersController::GraspType::MIDDLE:
      moveUntilLoadLimitHit(VectorIdx::Middle, factor, 2000, 200);
      break;
    case FingersController::GraspType::RING:
      moveUntilLoadLimitHit(VectorIdx::Ring, factor, 2000, 200);
      break;
    case FingersController::GraspType::THUMB:
      moveUntilLoadLimitHit(VectorIdx::Thumb, factor, 2000, 200);
      break;
    case FingersController::GraspType::THUMB_ROT:
      moveUntilLoadLimitHit(VectorIdx::ThumbRot, factor, 2000, 200);
      break;
  }
}

void FingersController::pingTest(const u8 ID) {
  int res = st.Ping(ID);
  if (res != -1) {
    Serial.print("Servo ID:");
    Serial.println(ID, DEC);
    SerialBT->printf("Servo ID: %d \n", ID);
    delay(100);
  } else {
    Serial.println("Ping servo ID error!");
    SerialBT->printf("Ping servo ID error!\n");
    delay(100);
  }
}

int FingersController::readTemper(const u8 ID) {
  return st.ReadTemper(ID);
}

int FingersController::readMaxTorque(u8 ID) {
  int torqueLimit = st.readWord(ID, SMS_STS_TORQUE_LIMIT_L);
  if (torqueLimit == -1) {
    SerialBT->print("Error reading torque limit for servo ");
    SerialBT->println(ID);
    return -1;  // Error code
  }
  return torqueLimit;  // Returns 0-1000 (1000 = 100% max torque)
}

void FingersController::printFeedback(const int id) {
  int Pos;
  int Speed;
  int Load;
  int Voltage;
  int Temper;
  int Move;
  int Current;
  int MaxTorque;
  if (st.FeedBack(id) != -1) {
    Pos = st.ReadPos(id);
    Speed = st.ReadSpeed(id);
    Load = st.ReadLoad(id);
    Voltage = st.ReadVoltage(id);
    Temper = st.ReadTemper(id);
    Move = st.ReadMove(id);
    Current = st.ReadCurrent(id);
    MaxTorque = readMaxTorque(id);
    SerialBT->printf("ID:%d, P:%d, L:%d, V:%d, T:%d, M:%d, C:%d, MaxT:%d\n", id, Pos, Load, Voltage, Temper, Move, Current, MaxTorque);
    delay(10);
  } else {
    SerialBT->println("FeedBack ERROR!");
    delay(500);
  }
}

void FingersController::changeID(const int currentId, const int newId) {
  st.unLockEprom(currentId);                   // Unlock EPROM-SAFE
  st.writeByte(currentId, SMS_STS_ID, newId);  // Change ID
  st.LockEprom(newId);                         // EPROM-SAFE is locked

  int ID = st.Ping(newId);
  if (ID == newId) {
    SerialBT->println("New ID successfully set!");
  } else {
    SerialBT->println("FAILED to set new ID!");
  }
}

void FingersController::readPositions(u8 IDN, u8 IDs[], s16 positions[]) {
  SerialBT->printf("[FingersController] Sync read positions count=%u\n", IDN);

  st.syncReadPacketTx(IDs, IDN, SMS_STS_PRESENT_POSITION_L, 2);

  for (int i = 0; i < IDN; i++) {
    u8 data[2];
    if (st.syncReadPacketRx(IDs[i], data) == 2) {
      // Use the built-in decoder!
      positions[i] = (s16)st.syncReadRxPacketToWrod(15);  // 15 = negative bit for position
    }
  }
}

bool FingersController::posMatch(u8 IDN, u8 IDs[], s16 target_pos[]) {

  s16 cur_pos[MAX_SERVOS];
  readPositions(IDN, IDs, cur_pos);

  bool allClose = true;
  int tolerance = 100;
  for (int i = 0; i < IDN; i++) {
    allClose = allClose && isClose(cur_pos[i], target_pos[i], tolerance);
  }
  return allClose;
}

// // Returns false if timed out
bool FingersController::posMatchWait(u8 IDN, u8 IDs[], s16 target_pos[], const unsigned long timeout) {
  auto t1 = millis();
  while (!posMatch(IDN, IDs, target_pos)) {
    delay(10);
    if (millis() - t1 > timeout) {
      return false;
    }
  }
  return true;
}

bool FingersController::posMatch(u8 IDN, u8 pos1[], u8 pos2[], u8 thr) {
  for (int i = 0; i < IDN; i++) {
    if (abs(pos1[i] - pos2[i]) > thr) {
      return false;
    }
  }
  return true;
}

bool FingersController::stalled(u8 IDN, u8 IDs[]) {
  for (u8 id = 0; id < IDN; id++) {
    if (st.ReadMove(IDs[id])) {
      return false;
    }
  }
  return true;
}

void FingersController::setCurPosAsTarget(u8 IDN, u8 IDs[]) {
  s16 curPos[MAX_SERVOS];
  u16 Speed[MAX_SERVOS];
  u8 Acc[MAX_SERVOS];
  readPositions(IDN, IDs, curPos);
  for (int i = 0; i < IDN; i++) {
    Speed[i] = 2000;
    Acc[i] = 200;
  }
  st.SyncWritePosEx(IDs, IDN, curPos, Speed, Acc);
}

void FingersController::moveUntilLoadLimitHit(u8 IDN, VectorIdx IDXs[], const u8 Factor[], u16 Speed[], u8 Acc[]) {

  u8 IDs[IDN];
  s16 Pos[IDN];
  for (u8 i = 0; i < IDN; i++) {
    IDs[i] = getMotorIdByVectorIndex(IDXs[i]);
    Pos[i] = getPosFromFactor(IDXs[i], Factor[i]);
  }
  moveUntilLoadLimitHit(IDN, IDs, Pos, Speed, Acc);
}

void FingersController::moveUntilLoadLimitHit(VectorIdx idx, const int factor, u16 speed, u8 acc) {
  SerialBT->printf("[FingersController] Finger move idx=%d factor=%d speed=%u accel=%u\n",
                   idx, factor, speed, acc);
  s16 pos = map(factor, 0, 100, MOTORS_POS_RANGE[idx][RANGE_MIN], MOTORS_POS_RANGE[idx][RANGE_MAX]);
  pos = limit(pos, MOTORS_POS_RANGE[idx][RANGE_MIN], MOTORS_POS_RANGE[idx][RANGE_MAX]);
  moveUntilLoadLimitHit(getMotorIdByVectorIndex(idx), pos, speed, acc);
}

void FingersController::move(u8 IDN, u8 IDs[], s16 Pos[], u16 Speed[], u8 Acc[]){
  SerialBT->printf("[FingersController] Sync move start count=%u\n", IDN);
  for (u8 i = 0; i < IDN; ++i) {
    SerialBT->printf("[FingersController]   servo=%u target=%d speed=%u accel=%u\n",
                     IDs[i], Pos[i], Speed[i], Acc[i]);
  }
  st.SyncWritePosEx(IDs, IDN, Pos, Speed, Acc);
}

void FingersController::moveUntilLoadLimitHit(u8 IDN, u8 IDs[], s16 Pos[], u16 Speed[], u8 Acc[]) {
  SerialBT->printf("[FingersController] Load-limited move start count=%u\n", IDN);
  for (u8 i = 0; i < IDN; ++i) {
    SerialBT->printf("[FingersController]   servo=%u target=%d speed=%u accel=%u\n",
                     IDs[i], Pos[i], Speed[i], Acc[i]);
  }
  st.SyncWritePosEx(IDs, IDN, Pos, Speed, Acc);
  delay(100);
  unsigned long t1 = millis();
  const unsigned long TIMEOUT_MS = 3000;  // 3 second timeout
  while (!stalled(IDN, IDs)) {
    delay(10);
    yield();
    if (millis() - t1 > TIMEOUT_MS) {
      SerialBT->println("ERROR: moveUntilLoadLimitHit timeout!");
      break;
    }
  }
  setCurPosAsTarget(IDN, IDs);
  SerialBT->printf("[FingersController] Load-limited move complete count=%u elapsed=%lu\n",
                   IDN, millis() - t1);
}

void FingersController::moveUntilLoadLimitHit(u8 ID, s16 pos, u16 speed, u8 acc) {
  u8 IDN = 1;
  u8 IDs[IDN] = { ID };
  s16 Pos[IDN] = { pos };
  u16 Speed[IDN] = { speed };
  u8 Acc[IDN] = { acc };
  moveUntilLoadLimitHit(IDN, IDs, Pos, Speed, Acc);
}

void FingersController::setMaxTorque(const u8 ID, const u16 maxTorque) {
  SerialBT->printf("[FingersController] Set max torque servo=%u value=%u\n", ID, maxTorque);
  st.EnableTorque(ID, 0);  // Disable torque
  st.writeWord(ID, SMS_STS_TORQUE_LIMIT_L, maxTorque);
  st.EnableTorque(ID, 1);  // Enable with limit
}

void FingersController::setMaxTorque(const u8 IDN, u8 IDs[], const u16 MaxTorque[]) {
  if (IDN > MAX_SERVOS) {
    SerialBT->printf("[FingersController] Set max torque rejected count=%u max=%d\n", IDN, MAX_SERVOS);
    return;
  }
  SerialBT->printf("[FingersController] Set max torque batch count=%u\n", IDN);
  // Prepare 2-byte torque values
  u8 bytes[2 * IDN];  // 5 servos × 2 bytes each

  // Convert to bytes
  const u8 torqueCount = IDN;
  for (int i = 0; i < 5; i++) {
    if (i >= torqueCount) {
      break;
    }
    bytes[i * 2] = MaxTorque[i] & 0xFF;             // Low byte
    bytes[i * 2 + 1] = (MaxTorque[i] >> 8) & 0xFF;  // High byte
  }
  // ONE SYNC WRITE - minimal delay!
  st.syncWrite(IDs, IDN, SMS_STS_TORQUE_LIMIT_L, bytes, 2);
}

void FingersController::setRangeByCurrentPos(u8 IDN, VectorIdx IDXs[], u8 rangeIndex) {
  for (u8 i = 0; i < IDN; i++) {
    MOTORS_POS_RANGE[IDXs[i]][rangeIndex] = readPos(getMotorIdByVectorIndex(IDXs[i]));
    SerialBT->printf("Set IDX %d POS: %d\n", IDXs[i], MOTORS_POS_RANGE[IDXs[i]][rangeIndex]);
  }
}

void FingersController::setRangeByCurrentPos(VectorIdx IDX, u8 rangeIndex) {
  MOTORS_POS_RANGE[IDX][rangeIndex] = readPos(getMotorIdByVectorIndex(IDX));
  SerialBT->printf("Set IDX %d POS: %d\n", IDX, MOTORS_POS_RANGE[IDX][rangeIndex]);
}

void FingersController::applyCalibrationFromStorage(CalibrationStorage* storage) {
    if (!storage || !storage->isValid()) {
        SerialBT->println("Warning: Cannot apply invalid calibration");
        return;
    }

    // Apply motor ranges
    for (int i = 0; i < 5; i++) {
        MOTORS_POS_RANGE[i][RANGE_MIN] = storage->getMotorMin(i);
        MOTORS_POS_RANGE[i][RANGE_MAX] = storage->getMotorMax(i);
    }

    // Apply pinch calibration
    setPinchCalibration(
        storage->getPinchThumbFactor(),
        storage->getPinchThumbRotFactor()
    );

    SerialBT->println("Calibration applied from storage");
}

void FingersController::saveCalibrationToStorage(CalibrationStorage* storage) {
    if (!storage) {
        SerialBT->println("Error: No storage instance");
        return;
    }

    // Save motor ranges
    for (int i = 0; i < 5; i++) {
        storage->setMotorMin(i, MOTORS_POS_RANGE[i][RANGE_MIN]);
        storage->setMotorMax(i, MOTORS_POS_RANGE[i][RANGE_MAX]);
    }

    // Save pinch calibration
    storage->setPinchThumbFactor(getPinchThumbFactor());
    storage->setPinchThumbRotFactor(getPinchThumbRotFactor());

    // Mark all stages complete
    storage->markStageComplete(CALIB_COMPLETE);

    SerialBT->println("Calibration prepared for storage");
}

