#include "calibration_storage.h"


CalibrationStorage g_calibStorage;

static const char* MOTOR_NAMES[NUM_MOTORS] = {
    "INDEX", "MIDDLE", "RING", "THUMB", "THUMB_ROT"
};

static bool parseStrictInt(const String& text, int* out_value) {
    if (!out_value) {
        return false;
    }

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


CalibrationStorage::CalibrationStorage()
    : m_state(CalibrationState::UNCHECKED)
    , m_dirty(false)
{
    // Initialize data to defaults
    memset(&m_data, 0, sizeof(m_data));
    m_data.magic = CALIB_MAGIC;
    m_data.schema_version = CALIB_SCHEMA_VERSION;

    // Default motor positions (center of servo range)
    for (int i = 0; i < NUM_MOTORS; i++) {
        m_data.motor_min[i] = 2048;
        m_data.motor_max[i] = 2048;
    }

    // Default pinch values
    m_data.pinch_thumb_factor = 65;
    m_data.pinch_thumb_rot_factor = 70;
}


CalibrationState CalibrationStorage::initialize() {
    Serial.println("[CalibStorage] Initializing...");

    // Try to load existing calibration
    if (load()) {
        Serial.println("[CalibStorage] Calibration loaded successfully");
        m_state = CalibrationState::VALID;
        return m_state;
    }

    // Load failed - determine why
    if (!m_prefs.begin(CALIB_NAMESPACE, true)) {
        Serial.println("[CalibStorage] Cannot open NVS namespace");
        m_state = CalibrationState::MISSING;
        return m_state;
    }

    // Check if schema key exists
    if (!m_prefs.isKey("schema")) {
        m_prefs.end();
        Serial.println("[CalibStorage] No calibration found (first boot?)");
        m_state = CalibrationState::MISSING;
        return m_state;
    }

    // Get stored schema version
    uint8_t storedVersion = m_prefs.getUChar("schema", 0);
    m_prefs.end();

    if (storedVersion != CALIB_SCHEMA_VERSION) {
        Serial.printf("[CalibStorage] Version mismatch: stored=%d current=%d\n",
                      storedVersion, CALIB_SCHEMA_VERSION);
        m_state = CalibrationState::VERSION_MISMATCH;
        return m_state;
    }

    // Version matched but validation failed - check completion
    if (m_data.completion_flags != CALIB_COMPLETE) {
        Serial.printf("[CalibStorage] Incomplete calibration: flags=0x%02X\n",
                      m_data.completion_flags);
        m_state = CalibrationState::INCOMPLETE;
        return m_state;
    }

    // Try backup
    Serial.println("[CalibStorage] Primary corrupt, trying backup...");
    if (tryLoadBackup()) {
        Serial.println("[CalibStorage] Backup loaded successfully");
        m_state = CalibrationState::BACKUP_USED;
        return m_state;
    }

    Serial.println("[CalibStorage] All recovery attempts failed");
    m_state = CalibrationState::CORRUPT;
    return m_state;
}

bool CalibrationStorage::load() {
    Serial.println("[CalibStorage] Loading calibration blob from NVS");
    if (!m_prefs.begin(CALIB_NAMESPACE, true)) {  // true = read-only
        Serial.println("[CalibStorage] Failed to open NVS for reading");
        return false;
    }

    // Read the data blob
    size_t len = m_prefs.getBytes("data", &m_data, sizeof(m_data));
    m_prefs.end();

    if (len != sizeof(m_data)) {
        Serial.printf("[CalibStorage] Size mismatch: read=%d expected=%d\n",
                      len, sizeof(m_data));
        return false;
    }

    // Validate loaded data
    const bool valid = validate();
    Serial.printf("[CalibStorage] Load validation result: %s\n", valid ? "OK" : "FAILED");
    if (valid) {
        m_state = CalibrationState::VALID;
        m_dirty = false;
    }
    return valid;
}

bool CalibrationStorage::save() {
    Serial.println("[CalibStorage] Saving calibration...");

    if (m_data.completion_flags != CALIB_COMPLETE) {
        Serial.printf("[CalibStorage] ERROR: Refusing save because completion flags are 0x%02X, expected 0x%02X\n",
                      m_data.completion_flags,
                      CALIB_COMPLETE);
        return false;
    }

    if (!validateRanges()) {
        Serial.println("[CalibStorage] ERROR: Refusing save because motor ranges are unsafe");
        return false;
    }

    // Update metadata
    m_data.magic = CALIB_MAGIC;
    m_data.schema_version = CALIB_SCHEMA_VERSION;
    m_data.calibration_timestamp = millis();
    m_data.calibration_count++;

    // Calculate CRC
    updateCRC();

    // Open NVS in read-write mode
    if (!m_prefs.begin(CALIB_NAMESPACE, false)) {
        Serial.println("[CalibStorage] ERROR: Cannot open NVS for writing");
        return false;
    }

    // Save backup first (so we have recovery if write fails)
    m_prefs.putBytes("backup", &m_data, sizeof(m_data));

    // Save primary data
    size_t written = m_prefs.putBytes("data", &m_data, sizeof(m_data));

    // Save schema version separately (for quick version check)
    m_prefs.putUChar("schema", CALIB_SCHEMA_VERSION);

    m_prefs.end();

    if (written != sizeof(m_data)) {
        Serial.printf("[CalibStorage] ERROR: Write size mismatch: %d\n", written);
        return false;
    }

    m_dirty = false;
    m_state = CalibrationState::VALID;
    Serial.printf("[CalibStorage] Saved successfully (count=%d)\n",
                  m_data.calibration_count);
    return true;
}

void CalibrationStorage::factoryReset() {
    Serial.println("[CalibStorage] Factory reset...");

    if (!m_prefs.begin(CALIB_NAMESPACE, false)) {
        Serial.println("[CalibStorage] Warning: Cannot open NVS");
        return;
    }

    m_prefs.clear();  // Remove all keys in namespace
    m_prefs.end();

    // Reset to defaults
    memset(&m_data, 0, sizeof(m_data));
    m_data.magic = CALIB_MAGIC;
    m_data.schema_version = CALIB_SCHEMA_VERSION;

    for (int i = 0; i < NUM_MOTORS; i++) {
        m_data.motor_min[i] = 2048;
        m_data.motor_max[i] = 2048;
    }

    m_data.pinch_thumb_factor = 65;
    m_data.pinch_thumb_rot_factor = 70;

    m_state = CalibrationState::MISSING;
    m_dirty = false;

    Serial.println("[CalibStorage] Factory reset complete");
}

void CalibrationStorage::setMotorMin(int idx, int16_t value) {
    if (idx >= 0 && idx < NUM_MOTORS) {
        Serial.printf("[CalibStorage] %s_MIN <- %d\n", MOTOR_NAMES[idx], value);
        m_data.motor_min[idx] = value;
        m_dirty = true;
    }
}

void CalibrationStorage::setMotorMax(int idx, int16_t value) {
    if (idx >= 0 && idx < NUM_MOTORS) {
        Serial.printf("[CalibStorage] %s_MAX <- %d\n", MOTOR_NAMES[idx], value);
        m_data.motor_max[idx] = value;
        m_dirty = true;
    }
}


bool CalibrationStorage::validate() {
    // Check magic number
    if (m_data.magic != CALIB_MAGIC) {
        Serial.printf("[CalibStorage] Invalid magic: 0x%04X\n", m_data.magic);
        return false;
    }

    // Check schema version
    if (m_data.schema_version != CALIB_SCHEMA_VERSION) {
        Serial.printf("[CalibStorage] Version mismatch: %d\n", m_data.schema_version);
        return false;
    }

    // Verify CRC
    uint16_t computed = calculateCRC(&m_data);
    if (computed != m_data.crc16) {
        Serial.printf("[CalibStorage] CRC mismatch: stored=0x%04X computed=0x%04X\n",
                      m_data.crc16, computed);
        return false;
    }

    // Validate motor ranges
    if (!validateRanges()) {
        return false;
    }

    return true;
}

bool CalibrationStorage::validateRanges() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        int16_t minPos = m_data.motor_min[i];
        int16_t maxPos = m_data.motor_max[i];

        // Check absolute bounds
        if (minPos < MOTOR_POS_ABSOLUTE_MIN || minPos > MOTOR_POS_ABSOLUTE_MAX) {
            Serial.printf("[CalibStorage] Motor %d min out of bounds: %d\n", i, minPos);
            return false;
        }
        if (maxPos < MOTOR_POS_ABSOLUTE_MIN || maxPos > MOTOR_POS_ABSOLUTE_MAX) {
            Serial.printf("[CalibStorage] Motor %d max out of bounds: %d\n", i, maxPos);
            return false;
        }

        // Check minimum travel range
        int range = abs(maxPos - minPos);
        if (range < MOTOR_RANGE_MIN_TRAVEL) {
            Serial.printf("[CalibStorage] Motor %d range too small: %d\n", i, range);
            return false;
        }
        if (range > MOTOR_RANGE_MAX_TRAVEL) {
            Serial.printf("[CalibStorage] Motor %d range too large: %d\n", i, range);
            return false;
        }
    }
    return true;
}

uint16_t CalibrationStorage::calculateCRC(const CalibrationData* data) {
    // CRC covers data portion only (everything after crc16 field)
    const uint8_t* start = (const uint8_t*)&data->motor_min[0];
    size_t len = sizeof(CalibrationData) - offsetof(CalibrationData, motor_min);

    // CRC-16-CCITT
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)start[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void CalibrationStorage::updateCRC() {
    m_data.crc16 = calculateCRC(&m_data);
}

bool CalibrationStorage::tryLoadBackup() {
    Serial.println("[CalibStorage] Attempting to load backup blob");
    if (!m_prefs.begin(CALIB_NAMESPACE, true)) {
        return false;
    }

    CalibrationData backup;
    size_t len = m_prefs.getBytes("backup", &backup, sizeof(backup));
    m_prefs.end();

    if (len != sizeof(backup)) {
        return false;
    }

    // Temporarily copy to m_data to validate
    CalibrationData original = m_data;
    m_data = backup;

    if (validate()) {
        // Backup is valid - restore as primary
        Serial.println("[CalibStorage] Backup blob validated, restoring as primary");
        save();
        return true;
    }

    // Restore original
    Serial.println("[CalibStorage] Backup blob validation failed");
    m_data = original;
    return false;
}

const char* CalibrationStorage::getStateString() const {
    switch (m_state) {
        case CalibrationState::UNCHECKED:       return "UNCHECKED";
        case CalibrationState::VALID:           return "VALID";
        case CalibrationState::MISSING:         return "MISSING";
        case CalibrationState::CORRUPT:         return "CORRUPT";
        case CalibrationState::VERSION_MISMATCH: return "VERSION_MISMATCH";
        case CalibrationState::INVALID_VALUES:  return "INVALID_VALUES";
        case CalibrationState::INCOMPLETE:      return "INCOMPLETE";
        case CalibrationState::BACKUP_USED:     return "BACKUP_USED";
        default: return "UNKNOWN";
    }
}


void CalibrationStorage::dumpToSerial(BluetoothSerial* bt) {
    bt->println("========== CALIBRATION DATA ==========");
    bt->printf("State: %s\n", getStateString());
    bt->printf("Schema Version: %d\n", m_data.schema_version);
    bt->printf("Magic: 0x%04X\n", m_data.magic);
    bt->printf("CRC16: 0x%04X\n", m_data.crc16);
    bt->printf("Completion Flags: 0x%02X (%s)\n",
               m_data.completion_flags,
               isCalibrationComplete() ? "COMPLETE" : "INCOMPLETE");
    bt->printf("Calibration Count: %d\n", m_data.calibration_count);
    bt->println("");
    bt->println("Motor Ranges:");
    for (int i = 0; i < NUM_MOTORS; i++) {
        int range = abs(m_data.motor_max[i] - m_data.motor_min[i]);
        int degrees = (int)((double)range / 4096.0 * 360.0);
        bt->printf("  %s: MIN=%d MAX=%d [%d deg]\n",
                   MOTOR_NAMES[i],
                   m_data.motor_min[i],
                   m_data.motor_max[i],
                   degrees);
    }
    bt->println("");
    bt->printf("Pinch Thumb Factor: %d\n", m_data.pinch_thumb_factor);
    bt->printf("Pinch Thumb Rot Factor: %d\n", m_data.pinch_thumb_rot_factor);
    bt->println("=======================================");
}

void CalibrationStorage::dumpAsJSON(BluetoothSerial* bt) {
    bt->println("{");
    bt->printf("  \"state\": \"%s\",\n", getStateString());
    bt->printf("  \"schema_version\": %d,\n", m_data.schema_version);
    bt->printf("  \"crc16\": %d,\n", m_data.crc16);
    bt->printf("  \"calibration_count\": %d,\n", m_data.calibration_count);
    bt->printf("  \"completion_flags\": %d,\n", m_data.completion_flags);
    bt->println("  \"motor_ranges\": [");
    for (int i = 0; i < NUM_MOTORS; i++) {
        bt->printf("    {\"name\": \"%s\", \"min\": %d, \"max\": %d}%s\n",
                   MOTOR_NAMES[i],
                   m_data.motor_min[i],
                   m_data.motor_max[i],
                   (i < NUM_MOTORS - 1) ? "," : "");
    }
    bt->println("  ],");
    bt->printf("  \"pinch_thumb_factor\": %d,\n", m_data.pinch_thumb_factor);
    bt->printf("  \"pinch_thumb_rot_factor\": %d\n", m_data.pinch_thumb_rot_factor);
    bt->println("}");
}

void CalibrationStorage::exportAsCommands(BluetoothSerial* bt) {
    bt->println("# Calibration Export - paste these commands to restore:");
    for (int i = 0; i < NUM_MOTORS; i++) {
        bt->printf("CALIB_SET %s_MIN %d\n", MOTOR_NAMES[i], m_data.motor_min[i]);
        bt->printf("CALIB_SET %s_MAX %d\n", MOTOR_NAMES[i], m_data.motor_max[i]);
    }
    bt->printf("CALIB_SET PINCH_THUMB %d\n", m_data.pinch_thumb_factor);
    bt->printf("CALIB_SET PINCH_THUMB_ROT %d\n", m_data.pinch_thumb_rot_factor);
    bt->println("CALIB_SAVE");
}

bool CalibrationStorage::parseSetCommand(const String& args, BluetoothSerial* bt) {
    // Expected format: "KEY VALUE" e.g. "INDEX_MIN 850"
    int spaceIdx = args.indexOf(' ');
    if (spaceIdx < 0) {
        bt->println("ERROR: Format is CALIB_SET KEY VALUE");
        return false;
    }

    String key = args.substring(0, spaceIdx);
    key.trim();
    key.toUpperCase();

    int value = 0;
    if (!parseStrictInt(args.substring(spaceIdx + 1), &value)) {
        bt->println("ERROR: CALIB_SET value must be an integer");
        return false;
    }
    Serial.printf("[CalibStorage] CALIB_SET %s %d\n", key.c_str(), value);

    if (value < MOTOR_POS_ABSOLUTE_MIN || value > MOTOR_POS_ABSOLUTE_MAX) {
        bt->printf("ERROR: Value must be between %d and %d\n",
                   MOTOR_POS_ABSOLUTE_MIN,
                   MOTOR_POS_ABSOLUTE_MAX);
        return false;
    }

    // Motor MIN values
    if (key == "INDEX_MIN") { setMotorMin(0, value); }
    else if (key == "MIDDLE_MIN") { setMotorMin(1, value); }
    else if (key == "RING_MIN") { setMotorMin(2, value); }
    else if (key == "THUMB_MIN") { setMotorMin(3, value); }
    else if (key == "THUMB_ROT_MIN") { setMotorMin(4, value); }
    // Motor MAX values
    else if (key == "INDEX_MAX") { setMotorMax(0, value); }
    else if (key == "MIDDLE_MAX") { setMotorMax(1, value); }
    else if (key == "RING_MAX") { setMotorMax(2, value); }
    else if (key == "THUMB_MAX") { setMotorMax(3, value); }
    else if (key == "THUMB_ROT_MAX") { setMotorMax(4, value); }
    // Pinch values
    else if (key == "PINCH_THUMB") { setPinchThumbFactor(value); }
    else if (key == "PINCH_THUMB_ROT") { setPinchThumbRotFactor(value); }
    else {
        bt->printf("ERROR: Unknown key '%s'\n", key.c_str());
        bt->println("Valid keys: INDEX_MIN, INDEX_MAX, MIDDLE_MIN, MIDDLE_MAX,");
        bt->println("            RING_MIN, RING_MAX, THUMB_MIN, THUMB_MAX,");
        bt->println("            THUMB_ROT_MIN, THUMB_ROT_MAX,");
        bt->println("            PINCH_THUMB, PINCH_THUMB_ROT");
        return false;
    }

    bt->printf("OK: %s = %d\n", key.c_str(), value);
    return true;
}
