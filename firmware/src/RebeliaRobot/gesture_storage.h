#ifndef REBELIA_ROBOT_GESTURE_STORAGE_H
#define REBELIA_ROBOT_GESTURE_STORAGE_H

#include <Arduino.h>
#include <Preferences.h>
#include "BluetoothSerial.h"

static const uint8_t GESTURE_FACTOR_COUNT = 5;
static const uint8_t MAX_CUSTOM_GESTURES = 8;
static const uint8_t MAX_GESTURE_STEPS = 12;
static const uint8_t GESTURE_NAME_CAPACITY = 16;
static const uint16_t GESTURE_MAGIC = 0x4753;
static const uint8_t GESTURE_SCHEMA_VERSION = 1;
static const char* const GESTURE_NAMESPACE = "gestures";

static const uint16_t MIN_GESTURE_MOVE_TIME_MS = 20;
static const uint16_t MAX_GESTURE_MOVE_TIME_MS = 5000;
static const uint16_t MAX_GESTURE_HOLD_TIME_MS = 5000;
static const uint16_t MIN_GESTURE_SPEED = 100;
static const uint16_t MAX_GESTURE_SPEED = 4000;
static const uint8_t MAX_GESTURE_ACCEL = 255;

enum class GestureRecordState : uint8_t {
  Deleted = 0,
  Active = 1
};

#pragma pack(push, 1)
struct GestureStep {
  uint8_t factor[GESTURE_FACTOR_COUNT];
  uint16_t move_time_ms;
  uint16_t hold_time_ms;
  uint16_t speed;
  uint8_t accel;
};

struct GestureRecord {
  uint16_t magic;
  uint8_t schema_version;
  uint8_t slot;
  uint8_t record_state;
  uint8_t step_count;
  uint16_t crc16;
  char name[GESTURE_NAME_CAPACITY];
  uint32_t revision;
  uint32_t saved_at_ms;
  GestureStep steps[MAX_GESTURE_STEPS];
};
#pragma pack(pop)

struct GestureEditor {
  bool has_data;
  bool dirty;
  char name[GESTURE_NAME_CAPACITY];
  uint8_t step_count;
  GestureStep steps[MAX_GESTURE_STEPS];
};

class GestureStorage {
public:
  GestureStorage();

  void initialize(BluetoothSerial* bt);

  void clearEditor();
  bool beginEditor(const String& name, String* error = nullptr);
  bool appendEditorStep(const GestureStep& step, String* error = nullptr);
  bool loadGesture(uint8_t slot, GestureRecord* out_record);
  bool loadGestureIntoEditor(uint8_t slot, String* error = nullptr);
  bool saveEditorToSlot(uint8_t slot, String* error = nullptr);
  bool deleteGesture(uint8_t slot, String* error = nullptr);

  void listGestures(BluetoothSerial* bt);
  void dumpGesture(uint8_t slot, BluetoothSerial* bt);
  void dumpEditor(BluetoothSerial* bt);

  bool hasEditorGesture() const { return m_editor.has_data; }
  bool isEditorDirty() const { return m_editor.dirty; }
  const GestureEditor& getEditor() const { return m_editor; }

  static bool parseGestureStepCsv(const String& csv, GestureStep* out_step, String* error = nullptr);
  static bool validateStep(const GestureStep& step, String* error = nullptr);
  static bool validateName(const String& name, String* error = nullptr);

private:
  enum class LoadResult : uint8_t {
    Missing = 0,
    Active,
    Deleted,
    Invalid
  };

  struct SlotLoad {
    LoadResult result;
    uint8_t active_bank;
    GestureRecord record;
  };

  void log(const String& message);
  bool isValidSlot(uint8_t slot) const;
  void setEditorFromRecord(const GestureRecord& record);
  void buildRecordFromEditor(uint8_t slot, uint32_t revision, GestureRecord* out_record);
  bool recordsEquivalent(const GestureRecord& a, const GestureRecord& b) const;
  String makeBankKey(uint8_t slot, uint8_t bank) const;
  String makeSelectorKey(uint8_t slot) const;
  bool readRecordFromBank(uint8_t slot, uint8_t bank, GestureRecord* out_record);
  bool writeRecordToBank(uint8_t slot, uint8_t bank, const GestureRecord& record);
  bool writeSelector(uint8_t slot, uint8_t bank);
  uint8_t readSelector(uint8_t slot);
  bool validateRecord(const GestureRecord& record, String* error = nullptr) const;
  SlotLoad loadSlotInternal(uint8_t slot);
  uint32_t nextRevisionForSlot(uint8_t slot);

  static uint16_t calculateCRC(const GestureRecord& record);
  static void updateCRC(GestureRecord* record);

  Preferences m_prefs;
  BluetoothSerial* m_bt;
  GestureEditor m_editor;
};

extern GestureStorage g_gestureStorage;

#endif  // REBELIA_ROBOT_GESTURE_STORAGE_H
