#include "gesture_storage.h"

#include <stddef.h>
#include <string.h>

GestureStorage g_gestureStorage;

static bool parseStrictIntToken(const String& token, int* out_value) {
  if (!out_value) {
    return false;
  }

  String trimmed = token;
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

GestureStorage::GestureStorage()
  : m_bt(nullptr) {
  clearEditor();
}

void GestureStorage::initialize(BluetoothSerial* bt) {
  m_bt = bt;
  log("[GestureStorage] Initializing gesture storage namespace");
  uint8_t active_count = 0;
  for (uint8_t slot = 0; slot < MAX_CUSTOM_GESTURES; ++slot) {
    SlotLoad load = loadSlotInternal(slot);
    if (load.result == LoadResult::Active) {
      ++active_count;
    }
  }
  log("[GestureStorage] Initialized. Active gestures: " + String(active_count));
}

void GestureStorage::clearEditor() {
  memset(&m_editor, 0, sizeof(m_editor));
}

bool GestureStorage::beginEditor(const String& name, String* error) {
  if (!validateName(name, error)) {
    return false;
  }

  clearEditor();
  m_editor.has_data = true;
  m_editor.dirty = true;
  strncpy(m_editor.name, name.c_str(), GESTURE_NAME_CAPACITY - 1);
  m_editor.name[GESTURE_NAME_CAPACITY - 1] = '\0';

  log("[GestureStorage] Editor started for gesture '" + String(m_editor.name) + "'");
  return true;
}

bool GestureStorage::appendEditorStep(const GestureStep& step, String* error) {
  if (!m_editor.has_data) {
    if (error) {
      *error = "Start the editor first with GEST_BEGIN <name>";
    }
    return false;
  }

  if (m_editor.step_count >= MAX_GESTURE_STEPS) {
    if (error) {
      *error = "Gesture already has the maximum number of steps";
    }
    return false;
  }

  if (!validateStep(step, error)) {
    return false;
  }

  m_editor.steps[m_editor.step_count] = step;
  ++m_editor.step_count;
  m_editor.dirty = true;

  log("[GestureStorage] Added editor step " + String(m_editor.step_count) +
      " move=" + String(step.move_time_ms) +
      " hold=" + String(step.hold_time_ms));
  return true;
}

bool GestureStorage::loadGesture(uint8_t slot, GestureRecord* out_record) {
  log("[GestureStorage] Load request slot " + String(slot));
  SlotLoad load = loadSlotInternal(slot);
  if (load.result != LoadResult::Active) {
    return false;
  }

  if (out_record) {
    *out_record = load.record;
  }
  return true;
}

bool GestureStorage::loadGestureIntoEditor(uint8_t slot, String* error) {
  GestureRecord record;
  if (!loadGesture(slot, &record)) {
    if (error) {
      *error = "No saved gesture in slot " + String(slot);
    }
    return false;
  }

  setEditorFromRecord(record);
  log("[GestureStorage] Loaded slot " + String(slot) + " into editor");
  return true;
}

bool GestureStorage::saveEditorToSlot(uint8_t slot, String* error) {
  log("[GestureStorage] Save editor request slot " + String(slot));
  if (!isValidSlot(slot)) {
    if (error) {
      *error = "Slot must be between 0 and " + String(MAX_CUSTOM_GESTURES - 1);
    }
    return false;
  }

  if (!m_editor.has_data || m_editor.step_count == 0) {
    if (error) {
      *error = "Editor is empty. Use GEST_BEGIN and GEST_STEP first";
    }
    return false;
  }

  SlotLoad current = loadSlotInternal(slot);
  const uint32_t revision = nextRevisionForSlot(slot);

  GestureRecord new_record;
  buildRecordFromEditor(slot, revision, &new_record);

  if (current.result == LoadResult::Active && recordsEquivalent(current.record, new_record)) {
    m_editor.dirty = false;
    log("[GestureStorage] Slot " + String(slot) + " unchanged. Flash write skipped");
    return true;
  }

  uint8_t target_bank = 0;
  if (current.result == LoadResult::Active || current.result == LoadResult::Deleted) {
    target_bank = current.active_bank == 0 ? 1 : 0;
  }

  if (!writeRecordToBank(slot, target_bank, new_record)) {
    if (error) {
      *error = "Failed to write gesture bank";
    }
    return false;
  }

  GestureRecord readback;
  if (!readRecordFromBank(slot, target_bank, &readback) || !validateRecord(readback, error)) {
    if (error && error->isEmpty()) {
      *error = "Gesture verification failed after write";
    }
    return false;
  }

  if (!writeSelector(slot, target_bank)) {
    if (error) {
      *error = "Failed to commit gesture selector";
    }
    return false;
  }

  m_editor.dirty = false;
  log("[GestureStorage] Saved gesture '" + String(new_record.name) + "' to slot " + String(slot));
  return true;
}

bool GestureStorage::deleteGesture(uint8_t slot, String* error) {
  log("[GestureStorage] Delete request slot " + String(slot));
  if (!isValidSlot(slot)) {
    if (error) {
      *error = "Slot must be between 0 and " + String(MAX_CUSTOM_GESTURES - 1);
    }
    return false;
  }

  SlotLoad current = loadSlotInternal(slot);
  if (current.result == LoadResult::Missing) {
    log("[GestureStorage] Delete skipped. Slot " + String(slot) + " is already empty");
    return true;
  }

  GestureRecord tombstone;
  memset(&tombstone, 0, sizeof(tombstone));
  tombstone.magic = GESTURE_MAGIC;
  tombstone.schema_version = GESTURE_SCHEMA_VERSION;
  tombstone.slot = slot;
  tombstone.record_state = static_cast<uint8_t>(GestureRecordState::Deleted);
  tombstone.step_count = 0;
  tombstone.revision = nextRevisionForSlot(slot);
  tombstone.saved_at_ms = millis();
  updateCRC(&tombstone);

  uint8_t target_bank = 0;
  if (current.result == LoadResult::Active || current.result == LoadResult::Deleted) {
    target_bank = current.active_bank == 0 ? 1 : 0;
  }

  if (!writeRecordToBank(slot, target_bank, tombstone)) {
    if (error) {
      *error = "Failed to write gesture tombstone";
    }
    return false;
  }

  if (!writeSelector(slot, target_bank)) {
    if (error) {
      *error = "Failed to commit gesture delete";
    }
    return false;
  }

  log("[GestureStorage] Deleted gesture in slot " + String(slot));
  return true;
}

void GestureStorage::listGestures(BluetoothSerial* bt) {
  if (!bt) {
    return;
  }

  bt->println("=== SAVED GESTURES ===");
  uint8_t found = 0;
  for (uint8_t slot = 0; slot < MAX_CUSTOM_GESTURES; ++slot) {
    GestureRecord record;
    if (loadGesture(slot, &record)) {
      bt->printf("Slot %u: %s | steps=%u | rev=%lu\n",
                 slot,
                 record.name,
                 record.step_count,
                 static_cast<unsigned long>(record.revision));
      ++found;
    }
  }
  if (found == 0) {
    bt->println("No saved gestures");
  }
  bt->println("======================");
}

void GestureStorage::dumpGesture(uint8_t slot, BluetoothSerial* bt) {
  if (!bt) {
    return;
  }

  SlotLoad load = loadSlotInternal(slot);
  if (load.result == LoadResult::Missing) {
    bt->printf("Slot %u is empty\n", slot);
    return;
  }
  if (load.result == LoadResult::Invalid) {
    bt->printf("Slot %u is corrupt\n", slot);
    return;
  }
  if (load.result == LoadResult::Deleted) {
    bt->printf("Slot %u is deleted\n", slot);
    return;
  }

  const GestureRecord& record = load.record;
  bt->println("=== GESTURE DUMP ===");
  bt->printf("Slot: %u\n", record.slot);
  bt->printf("Name: %s\n", record.name);
  bt->printf("Steps: %u\n", record.step_count);
  bt->printf("Revision: %lu\n", static_cast<unsigned long>(record.revision));
  for (uint8_t i = 0; i < record.step_count; ++i) {
    const GestureStep& step = record.steps[i];
    bt->printf("Step %u: [%u,%u,%u,%u,%u] move=%u hold=%u speed=%u accel=%u\n",
               i,
               step.factor[0],
               step.factor[1],
               step.factor[2],
               step.factor[3],
               step.factor[4],
               step.move_time_ms,
               step.hold_time_ms,
               step.speed,
               step.accel);
  }
  bt->println("====================");
}

void GestureStorage::dumpEditor(BluetoothSerial* bt) {
  if (!bt) {
    return;
  }

  if (!m_editor.has_data) {
    bt->println("Gesture editor is empty");
    return;
  }

  bt->println("=== EDITOR GESTURE ===");
  bt->printf("Name: %s\n", m_editor.name);
  bt->printf("Dirty: %s\n", m_editor.dirty ? "YES" : "NO");
  bt->printf("Steps: %u\n", m_editor.step_count);
  for (uint8_t i = 0; i < m_editor.step_count; ++i) {
    const GestureStep& step = m_editor.steps[i];
    bt->printf("Step %u: [%u,%u,%u,%u,%u] move=%u hold=%u speed=%u accel=%u\n",
               i,
               step.factor[0],
               step.factor[1],
               step.factor[2],
               step.factor[3],
               step.factor[4],
               step.move_time_ms,
               step.hold_time_ms,
               step.speed,
               step.accel);
  }
  bt->println("======================");
}

bool GestureStorage::parseGestureStepCsv(const String& csv, GestureStep* out_step, String* error) {
  if (!out_step) {
    if (error) {
      *error = "Output step pointer is null";
    }
    return false;
  }

  int values[9] = { 0 };
  int count = 0;
  int start = 0;

  while (true) {
    const int comma = csv.indexOf(',', start);
    String token = comma < 0 ? csv.substring(start) : csv.substring(start, comma);
    token.trim();

    if (token.isEmpty()) {
      if (error) {
        *error = "GEST_STEP contains an empty CSV field";
      }
      return false;
    }

    if (count >= 9) {
      if (error) {
        *error = "GEST_STEP has too many CSV values";
      }
      return false;
    }

    if (!parseStrictIntToken(token, &values[count])) {
      if (error) {
        *error = "GEST_STEP fields must be integers only";
      }
      return false;
    }
    ++count;

    if (comma < 0) {
      break;
    }
    start = comma + 1;
  }

  if (count != 9) {
    if (error) {
      *error = "GEST_STEP needs 9 CSV values: idx,mid,ring,thumb,thumbRot,moveMs,holdMs,speed,accel";
    }
    return false;
  }

  memset(out_step, 0, sizeof(GestureStep));
  for (uint8_t i = 0; i < GESTURE_FACTOR_COUNT; ++i) {
    out_step->factor[i] = static_cast<uint8_t>(values[i]);
  }
  out_step->move_time_ms = static_cast<uint16_t>(values[5]);
  out_step->hold_time_ms = static_cast<uint16_t>(values[6]);
  out_step->speed = static_cast<uint16_t>(values[7]);
  out_step->accel = static_cast<uint8_t>(values[8]);

  return validateStep(*out_step, error);
}

bool GestureStorage::validateStep(const GestureStep& step, String* error) {
  for (uint8_t i = 0; i < GESTURE_FACTOR_COUNT; ++i) {
    if (step.factor[i] > 100) {
      if (error) {
        *error = "Finger factors must be between 0 and 100";
      }
      return false;
    }
  }

  if (step.move_time_ms < MIN_GESTURE_MOVE_TIME_MS || step.move_time_ms > MAX_GESTURE_MOVE_TIME_MS) {
    if (error) {
      *error = "moveMs must be between " + String(MIN_GESTURE_MOVE_TIME_MS) + " and " + String(MAX_GESTURE_MOVE_TIME_MS);
    }
    return false;
  }

  if (step.hold_time_ms > MAX_GESTURE_HOLD_TIME_MS) {
    if (error) {
      *error = "holdMs must be between 0 and " + String(MAX_GESTURE_HOLD_TIME_MS);
    }
    return false;
  }

  if (step.speed < MIN_GESTURE_SPEED || step.speed > MAX_GESTURE_SPEED) {
    if (error) {
      *error = "speed must be between " + String(MIN_GESTURE_SPEED) + " and " + String(MAX_GESTURE_SPEED);
    }
    return false;
  }

  if (step.accel > MAX_GESTURE_ACCEL) {
    if (error) {
      *error = "accel must be between 0 and " + String(MAX_GESTURE_ACCEL);
    }
    return false;
  }

  return true;
}

bool GestureStorage::validateName(const String& name, String* error) {
  String trimmed = name;
  trimmed.trim();

  if (trimmed.isEmpty()) {
    if (error) {
      *error = "Gesture name cannot be empty";
    }
    return false;
  }

  if (trimmed.length() >= GESTURE_NAME_CAPACITY) {
    if (error) {
      *error = "Gesture name must be shorter than " + String(GESTURE_NAME_CAPACITY) + " characters";
    }
    return false;
  }

  for (size_t i = 0; i < trimmed.length(); ++i) {
    const char ch = trimmed.charAt(i);
    if (ch < 32 || ch > 126) {
      if (error) {
        *error = "Gesture name must use printable ASCII characters";
      }
      return false;
    }
  }

  return true;
}

void GestureStorage::log(const String& message) {
  Serial.println(message);
  if (m_bt) {
    m_bt->println(message);
  }
}

bool GestureStorage::isValidSlot(uint8_t slot) const {
  return slot < MAX_CUSTOM_GESTURES;
}

void GestureStorage::setEditorFromRecord(const GestureRecord& record) {
  clearEditor();
  m_editor.has_data = true;
  m_editor.dirty = false;
  strncpy(m_editor.name, record.name, GESTURE_NAME_CAPACITY - 1);
  m_editor.name[GESTURE_NAME_CAPACITY - 1] = '\0';
  m_editor.step_count = record.step_count;
  memcpy(m_editor.steps, record.steps, sizeof(record.steps));
}

void GestureStorage::buildRecordFromEditor(uint8_t slot, uint32_t revision, GestureRecord* out_record) {
  memset(out_record, 0, sizeof(GestureRecord));
  out_record->magic = GESTURE_MAGIC;
  out_record->schema_version = GESTURE_SCHEMA_VERSION;
  out_record->slot = slot;
  out_record->record_state = static_cast<uint8_t>(GestureRecordState::Active);
  out_record->step_count = m_editor.step_count;
  strncpy(out_record->name, m_editor.name, GESTURE_NAME_CAPACITY - 1);
  out_record->name[GESTURE_NAME_CAPACITY - 1] = '\0';
  out_record->revision = revision;
  out_record->saved_at_ms = millis();
  memcpy(out_record->steps, m_editor.steps, sizeof(m_editor.steps));
  updateCRC(out_record);
}

bool GestureStorage::recordsEquivalent(const GestureRecord& a, const GestureRecord& b) const {
  if (a.record_state != b.record_state || a.step_count != b.step_count) {
    return false;
  }
  if (strncmp(a.name, b.name, GESTURE_NAME_CAPACITY) != 0) {
    return false;
  }
  return memcmp(a.steps, b.steps, sizeof(a.steps)) == 0;
}

String GestureStorage::makeBankKey(uint8_t slot, uint8_t bank) const {
  char buffer[8];
  snprintf(buffer, sizeof(buffer), "g%ua%u", slot, bank);
  return String(buffer);
}

String GestureStorage::makeSelectorKey(uint8_t slot) const {
  char buffer[8];
  snprintf(buffer, sizeof(buffer), "g%usel", slot);
  return String(buffer);
}

bool GestureStorage::readRecordFromBank(uint8_t slot, uint8_t bank, GestureRecord* out_record) {
  if (!out_record) {
    return false;
  }

  if (!m_prefs.begin(GESTURE_NAMESPACE, true)) {
    log("[GestureStorage] Failed to open NVS for bank read");
    return false;
  }

  String key = makeBankKey(slot, bank);
  size_t read = m_prefs.getBytes(key.c_str(), out_record, sizeof(GestureRecord));
  m_prefs.end();

  return read == sizeof(GestureRecord);
}

bool GestureStorage::writeRecordToBank(uint8_t slot, uint8_t bank, const GestureRecord& record) {
  if (!m_prefs.begin(GESTURE_NAMESPACE, false)) {
    log("[GestureStorage] Failed to open NVS for bank write");
    return false;
  }

  log("[GestureStorage] Writing slot " + String(slot) + " bank " + String(bank));
  String key = makeBankKey(slot, bank);
  size_t written = m_prefs.putBytes(key.c_str(), &record, sizeof(GestureRecord));
  m_prefs.end();

  return written == sizeof(GestureRecord);
}

bool GestureStorage::writeSelector(uint8_t slot, uint8_t bank) {
  if (!m_prefs.begin(GESTURE_NAMESPACE, false)) {
    log("[GestureStorage] Failed to open NVS for selector write");
    return false;
  }

  log("[GestureStorage] Committing selector slot " + String(slot) + " -> bank " + String(bank));
  String key = makeSelectorKey(slot);
  size_t written = m_prefs.putUChar(key.c_str(), bank);
  m_prefs.end();

  return written == sizeof(uint8_t);
}

uint8_t GestureStorage::readSelector(uint8_t slot) {
  if (!m_prefs.begin(GESTURE_NAMESPACE, true)) {
    return 0;
  }

  String key = makeSelectorKey(slot);
  uint8_t bank = m_prefs.getUChar(key.c_str(), 0);
  m_prefs.end();
  return bank > 1 ? 0 : bank;
}

bool GestureStorage::validateRecord(const GestureRecord& record, String* error) const {
  if (record.magic != GESTURE_MAGIC) {
    if (error) {
      *error = "Gesture magic mismatch";
    }
    return false;
  }

  if (record.schema_version != GESTURE_SCHEMA_VERSION) {
    if (error) {
      *error = "Gesture schema mismatch";
    }
    return false;
  }

  if (!isValidSlot(record.slot)) {
    if (error) {
      *error = "Gesture slot out of range";
    }
    return false;
  }

  if (record.step_count > MAX_GESTURE_STEPS) {
    if (error) {
      *error = "Gesture step count out of range";
    }
    return false;
  }

  if (calculateCRC(record) != record.crc16) {
    if (error) {
      *error = "Gesture CRC mismatch";
    }
    return false;
  }

  const GestureRecordState state = static_cast<GestureRecordState>(record.record_state);
  if (state == GestureRecordState::Deleted) {
    return true;
  }

  if (state != GestureRecordState::Active) {
    if (error) {
      *error = "Gesture record state is invalid";
    }
    return false;
  }

  String name_error;
  if (!validateName(String(record.name), &name_error)) {
    if (error) {
      *error = name_error;
    }
    return false;
  }

  if (record.step_count == 0) {
    if (error) {
      *error = "Active gesture must contain at least one step";
    }
    return false;
  }

  for (uint8_t i = 0; i < record.step_count; ++i) {
    String step_error;
    if (!validateStep(record.steps[i], &step_error)) {
      if (error) {
        *error = "Invalid step " + String(i) + ": " + step_error;
      }
      return false;
    }
  }

  return true;
}

GestureStorage::SlotLoad GestureStorage::loadSlotInternal(uint8_t slot) {
  SlotLoad load;
  memset(&load, 0, sizeof(load));
  load.result = LoadResult::Missing;
  load.active_bank = readSelector(slot);

  GestureRecord primary;
  bool primary_present = readRecordFromBank(slot, load.active_bank, &primary);
  String error;
  if (primary_present && validateRecord(primary, &error)) {
    load.record = primary;
    load.result = static_cast<GestureRecordState>(primary.record_state) == GestureRecordState::Active
      ? LoadResult::Active
      : LoadResult::Deleted;
    return load;
  }

  const uint8_t fallback_bank = load.active_bank == 0 ? 1 : 0;
  GestureRecord fallback;
  bool fallback_present = readRecordFromBank(slot, fallback_bank, &fallback);
  if (fallback_present && validateRecord(fallback, &error)) {
    load.record = fallback;
    load.active_bank = fallback_bank;
    load.result = static_cast<GestureRecordState>(fallback.record_state) == GestureRecordState::Active
      ? LoadResult::Active
      : LoadResult::Deleted;
    return load;
  }

  if (primary_present || fallback_present) {
    load.result = LoadResult::Invalid;
  }

  return load;
}

uint32_t GestureStorage::nextRevisionForSlot(uint8_t slot) {
  SlotLoad load = loadSlotInternal(slot);
  if (load.result == LoadResult::Active || load.result == LoadResult::Deleted) {
    return load.record.revision + 1;
  }
  return 1;
}

uint16_t GestureStorage::calculateCRC(const GestureRecord& record) {
  const uint8_t* start = reinterpret_cast<const uint8_t*>(&record.slot);
  const size_t length = sizeof(GestureRecord) - offsetof(GestureRecord, slot);

  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(start[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void GestureStorage::updateCRC(GestureRecord* record) {
  if (!record) {
    return;
  }
  record->crc16 = calculateCRC(*record);
}
