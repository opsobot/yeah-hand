# Yeah Hand — Alternative embodiments, variants and extensions

This document is part of the public technical disclosure of the Yeah Hand,
an open source tendon-driven anthropomorphic hand usable as a
robotic end effector and as an active prosthetic hand. It extends the
disclosure of Rebelia V1 (https://github.com/opsobot/rebelia, OSHWA IT000021,
Zenodo DOI 10.5281/zenodo.23394100; technical disclosure on Technical
Disclosure Commons), whose content remains valid for Yeah Hand unless stated below.

Its purpose is to describe, as broadly as possible, the ways in which the
concepts of this design can be implemented, modified and extended, so that all
of them are publicly available prior art and remain free for anyone to make,
use, improve and sell. Each entry is meant to be understandable and
reproducible by a person skilled in robotics or prosthetics.

First public disclosure on GitHub:

| Part | Where | Date |
|---|---|---|
| Rebelia V1 mechanical design, firmware, electronics | opsobot/rebelia | 2025-03-03 to 2025-04-20 |
| Rebelia V2 = Yeah Hand v1.0: Blender sources (hierarchic model, two-part spools) | opsobot/rebelia, branch V2 | 2025-11-26 |
| Rebelia V2 = Yeah Hand v1.0: robotic firmware (feedback, calibration, ROS mode) | opsobot/rebelia, branch V2 | 2026-02-01 to 2026-02-25 |
| Yeah Hand repository and release v1.2 (STL, Blender source) | opsobot/yeah-hand | 2026-09-14 |
| Yeah Hand assembly instructions Rev 1.8 (IT, EN) | opsobot/yeah-hand | 2026-10-05 |
| This document | opsobot/yeah-hand | 2026-10-15 |

---

## 1. Core concepts of the Yeah Hand reference design

All core concepts of Rebelia V1 apply (actuators housed in the hand skeleton,
antagonistic flexion/extension tendons driven by one actuator per finger,
self-locking distal anchoring, thumb base on the thumb rotation actuator
flange, load-based calibration and force-limited grasping, grasp presets,
EMG control in the prosthetic version). Yeah Hand adds or changes:

1. **Bowden-tube tendon routing.** Each flexion tendon and each extension
   tendon runs inside its own low-friction tube (PTFE) from the finger to the
   spool. The tubes are inserted in holes, pass-through tunnels and seating
   slots formed in the printed skeleton, which fix their two ends and their
   path. The tubes keep tendons separated and guided, so the Rebelia V1
   tendons-barrier is no longer needed, and they keep the tendon path length
   constant regardless of how the tendons are routed inside the hand.
2. **Two-part spool.** Each flexion actuator carries a spool assembly made of
   two stacked parts: a flexion spool and an extension spool, fixed together
   to the actuator flange by the same screws. The flexion spool has two
   diameters (stepped profile). The flexion end of the tendon is threaded
   through holes of both parts and locked under a bolt; the extension end is
   wound about one turn in the opposite direction on the extension spool and
   locked under a second bolt. Each part can have its own diameter, so the
   take-up of the flexion end and the release of the extension end can be
   matched to the finger kinematics. The two parts are held together only by
   interlocking and loose screws during tendon installation, then fixed.
3. **Direct ring–little coupling on the spool.** The flexion ends and the
   extension ends of the ring and little finger tendons are fixed directly to
   the same two-part spool of the shared actuator, without clamps or sliding
   loops.
4. **Thumb rotation servo platform (TRS platform).** The thumb rotation
   actuator, with the thumb base bolted to its flange, is mounted on a separate
   platform that is inserted into the skeleton as a sub-assembly.
5. **Modular, identical upper fingers with pre-installed tendons.** The same
   finger design (the "index" STL files) is used for index, middle, ring and
   little fingers: identical, interchangeable modules, which simplify
   production, moulds, assembly and spare parts. Each finger is supplied
   with its tendon already threaded and anchored, and is inserted in its slot
   on the skeleton; the free tendon ends are then pulled through the Bowden
   tubes with a nylon-loop needle and fixed to the spool.
6. **Extended feedback and control.** The controller reads from each actuator
   position, load, voltage, temperature and current, and streams them
   periodically (ROS mode, about every 100 ms); commands set the closure of
   each degree of freedom directly (ROS commands) or through grasp presets.
   Whole-hand calibration routine; left and right hand versions selected in
   firmware; protection thresholds for over-current and over-temperature;
   per-actuator ID registration programs; Bluetooth and Wi-Fi links.
7. **Standard robot interface.** Mechanical interface according to
   ISO 9409-1-50-4-M6 with 12 V power through the robot interface and wireless
   signals.
8. **Grip surfaces.** Rigid and flexible printed polymers with replaceable,
   interchangeable skin-safe silicone pads.

### Yeah Hand-specific variants

- Bowden tubes: PTFE, nylon, polyethylene, coil-wound steel housings, lined
  tubes; one tube per tendon end or shared multi-lumen tubes; tubes moulded or
  printed into the skeleton as channels; tubes extending outside the hand to
  actuators placed in the forearm, on the robot or remotely.
- Two-part spool: parts of equal or different diameter; stepped or conical
  sections; non-circular (cam) profiles; parts rotationally adjustable with
  respect to each other to pre-tension the tendon, then locked; tendon locking
  by bolt, set screw, clamp plate or wedge.
- Upper fingers: one identical module for all positions (Yeah Hand reference), or
  fingers with individual lengths and proportions for each position
  (anthropomorphic), scaled sizes (child, adult, oversize), or a common
  module with position-specific phalanx lengths, covers or pads.
- Direct coupling of two or more fingers on one spool (in Yeah Hand the ring and
  little fingers open and close together), with equal or
  different spool diameters per finger to set the relative closing speed.
- TRS platform: platform carrying one or more actuators (thumb rotation, thumb
  flexion, wrist), removable as a module; quick-release fixing.
- Feedback streamed over serial, Bluetooth, Wi-Fi, USB, CAN or Ethernet, to
  ROS / ROS 2 or other middleware, used for slip detection, thermal
  protection, health monitoring and learning-based control.

## 2. Mechanical variants

| Element | Reference implementation | Alternatives |
|---|---|---|
| Skeleton | Single FDM-printed internal skeleton housing all actuators, with separate palm part and covers | Any shape of the skeleton that houses the actuators and supports fingers and covers: different outlines, sections, lengths and widths, open frame or closed shell, lattice or topology-optimised structure, different positions and orientations of the actuator slots; multi-part printed assembly; SLS/MJF nylon; injection-moulded; CNC-machined aluminium or magnesium; carbon-fibre composite; cast resin |
| Finger structure | Printed phalanges with rigid and flexible parts | Fully rigid phalanges with pin joints; compliant (flexure) joints printed in TPU/TPE; monolithic finger with living hinges; overmoulded rigid core and soft skin; elastic return elements (springs, elastic cords) instead of extension tendons |
| Number of phalanges | 3 per finger, 2 for the thumb | 2 or 4 phalanges; fused distal-medial phalanges with fixed coupling |
| Number of fingers | 5 | 2, 3 or 4 fingers using the same modules; 6-finger or mirrored-thumb grippers; left and right hand versions |
| Scale | Human-size hand | Child-size, partial hand (individual fingers for partial amputations), oversize industrial gripper, miniature versions |
| Covers | Snap-fit printed covers | Covers of any shape, size, style and colour (anthropomorphic, stylised, industrial, partial or none), fixed by snap-fit, screws, magnets or straps, in rigid or soft materials; silicone glove; textured high-friction pads; sensorised skin; cosmetic covers; interchangeable fingertips (soft, hard, tool tips) |
| Wrist | Joint with soft bearings | Movable wrist rotating about the X, Y and/or Z axis, i.e. 1, 2 or 3 rotational axes: pronation/supination, flexion/extension and radial/ulnar deviation, alone or in any combination; each axis passive (manually positioned, with friction or locking), compliant, or actuated (motors in the wrist, in the forearm or in the hand skeleton, direct or through tendons, gears or linkages); spherical or parallel-mechanism wrist; rigid quick-disconnect; standard prosthetic wrist interfaces (e.g. threaded or bayonet); ISO 9409 robot flange |
| Mounting | Robot flange with alignment pin; prosthetic socket cover | Quick-change tool changer; magnetic coupling; direct socket lamination; adapter plates for different robot brands |

## 3. Transmission and tendon variants

| Element | Reference implementation | Alternatives |
|---|---|---|
| Tendon material | Yeah Hand: braided polyethylene line (Rebelia V1: nylon line) | UHMWPE (Dyneema/Spectra), aramid, steel cable, PTFE-coated cable, braided fishing line |
| Spool | Yeah Hand: two-part spool per actuator, a flexion spool and an extension spool stacked on the same flange, tendons wound in opposite directions (Rebelia V1: one-piece spool) | **Stepped spool: one spool made of two coaxial sections of different diameter, one section for each end of the tendon (one for flexion, one for extension), wound in opposite directions on the same actuator. The ratio between the two diameters matches the different length that the flexion path and the extension path take up or release during finger motion, so both ends stay tensioned without springs or tensioners; the ratio can be chosen per finger.** Separate spools of different diameters for flexion and extension on the same shaft; more than two sections (e.g. for coupled fingers); non-circular (cam-shaped) sections for variable transmission ratio along the motion; spool integrated in the servo horn |
| Tendon locking on spool | Bolt head clamping | Set screw, crimp sleeve, knot in a slot, wedge clamp, quick-release clip |
| Tendon locking on phalanx | Double self-wrap | Knot, crimp sleeve, glue, printed hook, threaded insert with set screw |
| Tendon guiding | Yeah Hand: PTFE Bowden tubes from each finger to its spool, routed through holes, tunnels and slots in the skeleton (Rebelia V1: holes and channels in printed parts and a tendons-barrier) | PTFE tubes; Bowden sheaths; small pulleys or bearings at joints; low-friction inserts |
| Ring–little coupling | Yeah Hand: both ends of the ring and little tendons fixed directly to the same two-part spool (Rebelia V1: crimped clamp and sliding loop) | Whiffletree / rocker lever; pulley differential; elastic link; coupling of more fingers (e.g. middle+ring+little on one actuator); full single-actuator hand with differential tree |
| Tendon tension | Set at installation and calibrated by firmware | Spring-loaded tensioners; elastic segment in series (series elastic actuation); adjustable screw tensioners at the spool |
| Alternative transmissions | Tendons | Linkages; worm gear at each joint; lead screw and nut driving a linkage; hydraulic or pneumatic soft actuators |

## 4. Actuators and electronics variants

| Element | Reference implementation | Alternatives |
|---|---|---|
| Actuators | 5 serial bus servomotors (WaveShare ST3215) with position and load feedback | Other smart servos (Dynamixel, Feetech, etc.); DC gear motors with encoders and current sensing; BLDC with gearbox; linear actuators pulling tendons; non-backdrivable worm-drive motors for zero-power holding |
| Degrees of freedom | 5 actuated (index, middle, ring+little, thumb flexion, thumb rotation) | 1 to 6 actuators; independent little finger; actuated thumb abduction and adduction as separate axes; manual (passive) thumb positioning with locking |
| Controller | ESP32 (custom board for prosthesis; WaveShare servo driver for robot) | RP2040, STM32, nRF52, any microcontroller with UART; FPGA; control from the robot controller directly |
| Servo bus interface | Half-duplex UART with buffer ICs | RS-485, CAN bus, EtherCAT, PWM servos |
| Power supply | 12 V DC, switching regulator to 5 V | Prosthetic Li-ion battery pack (e.g. 2S/3S); robot tool-flange power; USB-C PD; supercapacitor buffer for peak current |
| Feedback / sensing | Servo position and load | Fingertip force sensors (FSR, capacitive, barometric), tendon tension sensors, joint angle encoders, tactile skins, IMU, proximity sensors, camera in the palm |

## 5. Control and interface variants

- Grasp selection and closure commanded by: Bluetooth serial (reference),
  Wi-Fi, USB, wired UART, robot I/O, CAN, ROS / ROS 2 nodes.
- For prosthetic use: surface EMG with a generic number N of channels
  (1, 2 or more sensors, single electrodes or arrays/sleeves), with signal
  processing by FFT (reference), by machine learning (e.g. neural networks,
  support vector machines, LDA, random forests, trained on the user's
  signals to classify gestures or estimate continuous closure/force), or by a
  combination of both (FFT or other spectral/time-domain features fed to a
  machine-learning classifier or regressor), with threshold, proportional or
  pattern-recognition control, and optional online adaptation or
  recalibration of the model; force myography, ultrasound sensing,
  implanted sensors, foot switches, voice commands, smartphone app,
  co-contraction to switch grasp, IMU gestures, automatic grasp selection by a
  camera and object recognition.
- Closure strategies: position control, force limit per finger (reference),
  impedance/compliance control, slip detection with automatic re-gripping,
  closing speed proportional to the control signal.
- Calibration: load-threshold calibration at start-up (reference), calibration
  with end-stop switches, manual calibration via app, periodic re-calibration
  to compensate for tendon creep.
- Additional grasp presets: lateral (key) grip, tripod, pointing, hook,
  spherical, cylindrical, precision pinch, user-defined grasps recorded by
  demonstration.

## 6. Applications

- End effector for collaborative and industrial robot arms.
- Upper-limb prosthesis (transradial, wrist disarticulation; partial hand with
  individual finger modules).
- Research and teaching platform for robotics, grasping and machine learning.
- Humanoid robots and teleoperation (with a data glove or exoskeleton as
  master).
- Rehabilitation devices and hand exoskeletons using the same tendon and spool
  actuation on a glove.
- Remote handling in hazardous environments; animatronics; assistive
  feeding devices.

## 7. Solutions considered and discarded

*To be completed by the author: list here the approaches tried during
development and why they were changed (e.g. threading tools for tendons, other
tendon materials, other anchoring methods, other licences or architectures).
Discarded solutions are prior art too.*

- Tendon threading: several tools were tried; a loop of 0.35 mm nylon thread
  used as a needle proved the fastest and most reliable method.
- Rebelia V1 tendons-barrier on the rear of the skeleton: replaced in Yeah Hand by Bowden
  tubes, which separate and guide every tendon end individually.
- Rebelia V1 ring–little coupling with crimped clamp and sliding loop: replaced in Yeah Hand
  by fixing the tendon ends of both fingers directly to the shared spool.
- Rebelia V1 one-piece spool: replaced in Yeah Hand by a two-part spool (flexion and
  extension parts), which simplifies tendon installation and allows different
  diameters for the two tendon ends.

---

## Change log

| Date | Change |
|---|---|
| 2026-10-15 | First version of this document |
