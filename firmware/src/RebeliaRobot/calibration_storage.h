#ifndef CALIBRATION_STORAGE_H
#define CALIBRATION_STORAGE_H

#include <Arduino.h>
#include <Preferences.h>
#include "BluetoothSerial.h"

#define CALIB_SCHEMA_VERSION    1

#define CALIB_MAGIC             0xCAFE

#define CALIB_NAMESPACE         "calib"

#define MOTOR_POS_ABSOLUTE_MIN  0
#define MOTOR_POS_ABSOLUTE_MAX  4095
#define MOTOR_RANGE_MIN_TRAVEL  100
#define MOTOR_RANGE_MAX_TRAVEL  3500

#define NUM_MOTORS              5


#define CALIB_STAGE_OPEN        (1 << 0)  // Open limits found
#define CALIB_STAGE_CLOSE       (1 << 1)  // Close limits found
#define CALIB_STAGE_THUMB_ROT   (1 << 2)  // Thumb rotation calibrated
#define CALIB_STAGE_PINCH       (1 << 3)  // Pinch grasp calibrated
#define CALIB_COMPLETE          0x0F      // All stages complete

#pragma pack(push, 1)


struct CalibrationData {
    // Header (6 bytes)
    uint16_t magic;               // Must be CALIB_MAGIC (0xCAFE)
    uint8_t  schema_version;      // Schema version for compatibility
    uint8_t  completion_flags;    // Bit flags for calibration stages
    uint16_t crc16;               // CRC-16 checksum of data portion

    // Motor ranges (20 bytes) - indexes match VectorIdx enum
    // [0]=Index, [1]=Middle, [2]=Ring, [3]=Thumb, [4]=ThumbRot
    int16_t  motor_min[NUM_MOTORS];  // Open position for each motor
    int16_t  motor_max[NUM_MOTORS];  // Closed position for each motor

    // Pinch calibration (4 bytes)
    int16_t  pinch_thumb_factor;     // Calibrated thumb factor for pinch
    int16_t  pinch_thumb_rot_factor; // Calibrated thumb rotation for pinch

    // Metadata (6 bytes)
    uint32_t calibration_timestamp;  // millis() when calibrated
    uint16_t calibration_count;      // Number of times calibrated
};


#pragma pack(pop)

enum class CalibrationState {
    UNCHECKED,          // Not yet checked
    VALID,              // All checks passed, calibration loaded
    MISSING,            // No calibration data found (first boot)
    CORRUPT,            // CRC check failed
    VERSION_MISMATCH,   // Schema version incompatible
    INVALID_VALUES,     // Values outside acceptable range
    INCOMPLETE,         // Calibration was interrupted
    BACKUP_USED         // Primary failed, using backup
};


class CalibrationStorage {
public:
    // Constructor
    CalibrationStorage();

    // ========== LIFECYCLE METHODS ==========

    /**
     * Initialize storage and attempt to load calibration
     * Call this in setup() BEFORE any motor operations
     * @return CalibrationState indicating result
     */
    CalibrationState initialize();

    /**
     * Save current calibration data to NVS
     * @return true if save successful
     */
    bool save();

    /**
     * Load calibration data from NVS
     * @return true if load successful and data valid
     */
    bool load();

    /**
     * Reset to factory defaults (clear all stored data)
     */
    void factoryReset();

    // ========== DATA ACCESS METHODS ==========

    /**
     * Get the current calibration data blob
     */
    const CalibrationData& getData() const { return m_data; }

    /**
     * Get current calibration state
     */
    CalibrationState getState() const { return m_state; }

    /**
     * Get state as human-readable string
     */
    const char* getStateString() const;

    /**
     * Check if calibration is valid and ready for operation
     */
    bool isValid() const {
        return m_state == CalibrationState::VALID ||
               m_state == CalibrationState::BACKUP_USED;
    }

    // ========== MOTOR RANGE ACCESS ==========

    /**
     * Get motor minimum (open) position
     * @param idx Motor index (0-4)
     */
    int16_t getMotorMin(int idx) const {
        return (idx >= 0 && idx < NUM_MOTORS) ? m_data.motor_min[idx] : 2048;
    }

    /**
     * Get motor maximum (closed) position
     * @param idx Motor index (0-4)
     */
    int16_t getMotorMax(int idx) const {
        return (idx >= 0 && idx < NUM_MOTORS) ? m_data.motor_max[idx] : 2048;
    }

    /**
     * Set motor minimum (open) position
     * @param idx Motor index (0-4)
     * @param value Position value (0-4095)
     */
    void setMotorMin(int idx, int16_t value);

    /**
     * Set motor maximum (closed) position
     * @param idx Motor index (0-4)
     * @param value Position value (0-4095)
     */
    void setMotorMax(int idx, int16_t value);

    // ========== PINCH CALIBRATION ACCESS ==========

    int16_t getPinchThumbFactor() const { return m_data.pinch_thumb_factor; }
    int16_t getPinchThumbRotFactor() const { return m_data.pinch_thumb_rot_factor; }

    void setPinchThumbFactor(int16_t value) {
        m_data.pinch_thumb_factor = value;
        m_dirty = true;
    }

    void setPinchThumbRotFactor(int16_t value) {
        m_data.pinch_thumb_rot_factor = value;
        m_dirty = true;
    }

    // ========== CALIBRATION STAGE TRACKING ==========

    /**
     * Mark a calibration stage as complete
     * @param stage One of CALIB_STAGE_* constants
     */
    void markStageComplete(uint8_t stage) {
        m_data.completion_flags |= stage;
    }

    /**
     * Reset all completion flags (start new calibration)
     */
    void resetCompletionFlags() {
        m_data.completion_flags = 0;
    }

    /**
     * Check if all calibration stages are complete
     */
    bool isCalibrationComplete() const {
        return m_data.completion_flags == CALIB_COMPLETE;
    }

    // ========== VALIDATION METHODS ==========

    /**
     * Validate the current calibration data
     * @return true if all validation checks pass
     */
    bool validate();

    /**
     * Calculate CRC-16 for data portion
     * @param data Pointer to CalibrationData
     * @return Calculated CRC-16 value
     */
    static uint16_t calculateCRC(const CalibrationData* data);

    // ========== DEBUG/TOOLING METHODS ==========

    /**
     * Dump calibration data to Bluetooth serial
     * @param bt Pointer to BluetoothSerial instance
     */
    void dumpToSerial(BluetoothSerial* bt);

    /**
     * Output calibration as JSON
     * @param bt Pointer to BluetoothSerial instance
     */
    void dumpAsJSON(BluetoothSerial* bt);

    /**
     * Export as commands that can be copy-pasted
     * @param bt Pointer to BluetoothSerial instance
     */
    void exportAsCommands(BluetoothSerial* bt);

    /**
     * Parse CALIB_SET command and apply value
     * Format: "KEY VALUE" e.g. "INDEX_MIN 850"
     * @param args Command arguments (everything after "CALIB_SET ")
     * @param bt Pointer to BluetoothSerial for response
     * @return true if successfully parsed and applied
     */
    bool parseSetCommand(const String& args, BluetoothSerial* bt);

private:
    // ========== PRIVATE METHODS ==========

    /**
     * Try to load backup calibration data
     */
    bool tryLoadBackup();

    /**
     * Validate motor range values
     */
    bool validateRanges();

    /**
     * Update CRC in data structure
     */
    void updateCRC();

    // ========== MEMBER VARIABLES ==========

    Preferences m_prefs;          // NVS preferences instance
    CalibrationData m_data;       // Current calibration data
    CalibrationState m_state;     // Current state
    bool m_dirty;                 // True if data changed since last save
};


#endif // CALIBRATION_STORAGE_H
