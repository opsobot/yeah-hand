# Yeah Hand firmware {#mainpage}

Firmware of the Yeah Hand (ESP32-WROOM, Arduino core 3.3.x, five Feetech ST3215 servos on a
1 Mbit/s bus). This page gives the overall picture; every function has its own page (see
*Files* and *Classes*), with call graphs showing who calls it and what it calls.

Build these pages: `cd firmware && doxygen Doxyfile`, then open `docs/doxygen/html/index.html`
(needs `doxygen` and `graphviz`).

## Files

| File | Content |
|---|---|
| `RebeliaRobot.ino` | `setup()`, `loop()`, command parsing and dispatch, service and test commands (CYCLE, TLOG, KTEST), telemetry |
| `fingers_controller.h/.cpp` | class FingersController: servo bus, calibration, grasp matrices, movements, torque limits, power presets, holding phase, safety check |
| `security.h/.cpp` | namespace HandSecurity: provisioning, password sessions (AUTH), lockout, Bluetooth pairing window and encryption, FACTORY commands |
| `version.h` | `FW_VERSION` (printed by FACTORY STATUS) |
| `tools/provision_hand.py` | manufacturer: generates the password, provisions a hand over USB, appends to the register |
| `tools/emc_endurance_test.py` | manufacturer: opens/closes the hand over Bluetooth for hours, logs acknowledgements and temperatures |

## Servos and units

| ID | VectorIdx | Joint |
|---|---|---|
| 1 | Index (0) | index flexion |
| 2 | Middle (1) | middle flexion |
| 3 | Ring (2) | ring + little flexion (one servo) |
| 4 | Thumb (3) | thumb flexion |
| 5 | ThumbRot (4) | thumb rotation |

- Position: 0..4095, 0.088 deg per step.
- **Factor**: 0..100, 0 = open, 100 = closed, mapped on the calibrated range
  `MOTORS_POS_RANGE[idx][RANGE_MIN..RANGE_MAX]` (getPosFromFactor(), getFactorFromPos()).
- Torque limit (register 48, in RAM): 0..1000, about 0.02 kg·cm per unit.
- Current: 6.5 mA per unit. Temperature: deg C.

## Start-up

1. `setup()` loads the provisioning from flash (HandSecurity::loadProvisioning()).
2. **Not provisioned**: Bluetooth stays off; only `FACTORY STATUS` / `FACTORY PROVISION` on USB
   are accepted. Nothing moves.
3. **Provisioned**: Bluetooth starts with authentication + encryption required
   (HandSecurity::preBeginBluetooth()); the pairing window is open for 120 s. Hand side (L/R)
   comes from the model stored at provisioning, the power preset from NVS.
4. The hand does **not move** until an authenticated client sends `CALIBRATE`
   (`g_calibrated` is false after every power-on).

## From a command line to a movement

```
loop()
 ├─ readChannel(USB)  ─┐
 ├─ readChannel(BT)   ─┴─ readLine() → [FACTORY on USB] → processStringCmd()
 │                                         ├─ HandSecurity::handleCommand()  AUTH, LOGOUT, SETPASS, PAIRING, UNPAIR ALL
 │                                         ├─ not authenticated → ERR AUTH REQUIRED
 │                                         ├─ direct, blocking:  CALIBRATE, ROS, FI..RT, INFO, POS
 │                                         ├─ deferred to loop:  grasp name → g_new_grasp_type
 │                                         │                     0..100     → g_newClosurePercent
 │                                         └─ SERVICE ON → processServiceCommand()
 ├─ cycleUpdate(), tlogUpdate()
 ├─ FSM::Control and calibrated:
 │     grasp changed   → prepareGrasp()   (start pose, blocking)
 │     closure changed → fc.action() → grasp() → move()   (non-blocking)
 ├─ FSM::TendonInstall → tendonInstallation(), once, then FSM::DoNothing
 ├─ telemetry every 100 ms (sendRosFeedback)
 ├─ fc.holdUpdate()      holding phase
 └─ fc.safetyFeature()   every 1 s
```

Each channel (USB, Bluetooth) has its own session; replies go to the channel of the command
being executed (`g_out`, FingersController::setOutput()). If several `ROS` lines arrive at
once, only the last is executed.

## States

| State | Where | Meaning |
|---|---|---|
| provisioned | HandSecurity | password stored; otherwise only FACTORY on USB |
| session.authenticated | HandSecurity::Session | per channel, after `AUTH`; reset by `LOGOUT` or link loss |
| `g_calibrated` | .ino | false after power-on; every motion command is refused until `CALIBRATE` |
| `g_serviceMode` | .ino | `SERVICE ON` / `SERVICE OFF`; reset by a Bluetooth link loss |
| `gFsmState` | .ino | `Control`: grasps and closures run. `TendonInstall`: INSTALL runs once, then `DoNothing`. `Testing`: TEST, does nothing. `CONTROL`, `SERVICE OFF` and `CALIBRATE` return to `Control` |

## Two kinds of movement

| | FingersController::moveUntilLoadLimitHit() | FingersController::move() |
|---|---|---|
| Blocking | yes (until stopped, max 6 s) | no |
| At the end | goal = present position, hold torque at once | holding phase armed, applied by holdUpdate() when all fingers stop |
| Used by | ROS, FI..RT, MOVE, CALIBRATE, INSTALL, prepareGrasp(), CYCLE on one servo | grasp() (closure 0..100 of a grasp), KTEST |

"Stopped" = no servo reports the *moving* flag: the target was reached or the torque limit
was hit on an object.

## Grasps

A grasp is a matrix of frames (`POWER_MATRIX`, `BAR_MATRIX`, `MONKEY_MATRIX`, `PINCH1_MATRIX`,
`PINCH2_MATRIX` in fingers_controller.h). Each row: factors of the 5 joints + the grasp
factor of that frame. grasp() finds the two frames around the requested closure and
interpolates every joint between them. Frame 0 is the start pose (prepareGrasp()).
`SETPINCH` shifts the thumb flexion of the pinch grasps.

## Calibration (FingersController::calibrateHand())

Blocking, a few seconds, each step with low torque limits so the fingers stop on their
mechanical end stops:

1. Pre-position for opening (torque 250).
2. Open to the end stops (75/100/75/150/200), step slightly back, store **RANGE_MIN**.
3. Pre-position for closing (200), close to the end stops (300/300/300/500), store
   **RANGE_MAX** of the 4 flexions.
4. Thumb rotation: open the thumb flexion, rotate to both end stops (400), step back, store
   both ends.
5. Open all (200), then working torque = preset caps (MAX_LOAD, TF_MAX_LOAD for the thumb,
   200 for the thumb rotation).

Every value is capped by the active preset, so on preset 1A (MAX_LOAD 200) a stiff new hand
may close ring+little to about 90 %: the calibrated "100" is where the finger stopped.
Directions and positions differ for left and right hands (`LS_` / `RS_` arrays).

## Torque: presets, working torque, holding phase

- **Power presets** (`POWER_PRESETS`, `SUPPLY <name>`, stored in NVS, applied at restart):
  `{MAX_LOAD, TF_MAX_LOAD, OVER_CURRENT, OVER_LOAD, OVER_TEMP, HOLD_LOAD}`

  | Preset | Values | Use |
  |---|---|---|
  | 5A | 330, 300, 150, 600, 60, 80 | default |
  | 1A | 200, 200, 150, 250, 60, 80 | supplies limited to about 1 A |
  | TEST | 600, 600, 250, 900, 60, 80 | manufacturer tests only, never for delivery |

- **Working torque**: setMaxTorque() writes the limit capped by the preset and remembers it
  in `work_torque[]`. Written live, with the torque on (switching the torque off and on would
  move the goal to the present position and the finger would stop pushing).
- **Holding phase**: when a movement has finished, every servo drops to `HOLD_LOAD` (80) to
  limit heat; restoreWorkTorque() puts the working torque back before the next movement.
  `HOLD <n>` changes it until restart (tests).

## Safety (FingersController::safetyFeature(), every second)

- Reads current, load and temperature of each servo.
- Temperature: out-of-range readings dropped; decisions on the **median of the last 5**
  readings (one wrong reading cannot stop the hand).
- `WARN HOT` at OVER_TEMP − 5 (55 °C): new closing commands are refused (`ERR HOT`,
  refuseIfHot()); opening is always allowed. `INFO COOL` at 52 °C.
- **Torque cut** of a servo on OVER_CURRENT, OVER_LOAD or OVER_TEMP (60 °C), with a
  `SAFETY:` message; torque back on when the fault is gone.
- Bluetooth link loss (enterSafeStateAfterLinkLoss()): service mode off, pending motion
  cancelled; the hand keeps its position, an object is not released.

## Security (HandSecurity)

- `FACTORY PROVISION <serial> <L|R> <password>`: USB only, once (until the flash is erased);
  the password can never be read back.
- `FACTORY STATUS`: serial, model, Bluetooth name, eFuse MAC, `fw=` (FW_VERSION), `build=`
  (compile date of the .ino), `elf=` (first hex digits of the ELF SHA-256, identifies the
  exact build).
- `AUTH <password>` per channel, compared in constant time; after 5 failures increasing
  lockout (30 s, doubled, max 15 min).
- Bluetooth: SPP server restarted with authentication + encryption, encryption keys of
  128 bit only; new pairings only inside the pairing window (120 s at power-on or after
  `PAIRING`); devices bonded outside the window are refused.

## Test and service commands (after SERVICE ON)

| Command | Function | Notes |
|---|---|---|
| `INSTALL` | tendonInstallation() | flexion servos to the tendon position, torque kept |
| `CALIBRATE` | doCalibrate() | also allowed outside service mode |
| `SETCENTER <id>` | setCenterOfRange() | servo EEPROM offset |
| `SUPPLY [<preset>]` | printPowerPresets() / savePowerPreset() | restarts to apply |
| `SETPINCH <-40..40>` | setPinchOffset() | until restart |
| `LIM <id> <0..1000>` | setMaxTorque() | capped by the preset |
| `HOLD <0..1000>` | setHoldLoad() | until restart |
| `MOVE <id> <0..4095>` | moveUntilLoadLimitHit() | raw position |
| `CYCLE <s> <n> [servo]` | cycleUpdate() | s closed + s open, n cycles, grasp or one servo; log in NVS, `CYCLE LOG`; stopped by any command except INFO, TELEMETRY, AUTH, SERVICE ON, TLOG, CYCLE…; not stopped by a link loss |
| `TLOG <min>` / `TLOG SHOW` / `TLOG OFF` | tlogStart() / tlogShow() | temperatures and positions every n minutes, 60 records |
| `KTEST <id> <start> <step> <steps> [settle ms]` | runKTest() | stiffness K = dLIM / \|dP\| |
| `TEST` | — | sets FSM::Testing, does nothing |
| `SERVICE OFF` | — | back to Control |

Normal commands: `CALIBRATE`, `TELEMETRY ON|OFF`, `POWER|BAR|MONKEY|PINCH1|PINCH2`, `0..100`,
`ROS a b c d e`, `FI|FM|FR|FT|RT <0..100>`, `INFO <id>`, `POS`, `CONTROL`, `HELP`, `LOGOUT`,
`SETPASS <old> <new>`, `PAIRING`, `UNPAIR ALL`, `SERVICE ON`.

## Data kept in flash (NVS)

| Namespace | Content |
|---|---|
| `yhsec` | password, serial, model (L/R) |
| `yhcfg` | power preset |
| `yhcycle` | last CYCLE log |
| `yhtlog` | last TLOG record |

## Known limits and legacy code

- `moveFingerSync`, `moveFingerAsync`, `moveAllFingersToMiddlePosition` are declared in
  fingers_controller.h but not implemented or used.
- `TEST` / FSM::Testing does nothing (its test function is commented out).
- `POS` prints the 4 flexion servos only.
- `build=` in FACTORY STATUS is the compile date of the .ino: the Arduino IDE recompiles only
  changed files, so it can be older than the build. Use `elf=` to identify a build.
- safetyFeature() writes "torque on" to every healthy servo every second.
- `MONKEY_FRAMES` was 7 while the matrix has 4 rows (fixed: no effect in practice, the
  search always stopped before row 4).
