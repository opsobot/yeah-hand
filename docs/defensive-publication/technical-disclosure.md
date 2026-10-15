# Tendon-driven anthropomorphic hand with Bowden-tube tendon routing inside the hand skeleton, two-part antagonistic spools, direct finger coupling on the spool and a removable thumb-rotation actuator platform, for robotic and prosthetic use

- **Author:** Vittorio Lumare (vittorio.lumare@robotgarage.org)
- **Project name:** Yeah Hand (current version v1.3; first published as Rebelia V2 = Yeah Hand v1.0)
- **First public disclosure:** 26 November 2025 as Rebelia V2 (Yeah Hand v1.0), https://github.com/opsobot/rebelia/tree/V2 ; repository https://github.com/opsobot/yeah-hand and release v1.2 on 14 September 2026; assembly instructions Rev 1.8 on 5 October 2026
- **Archived on Zenodo:** release v1.3 https://doi.org/10.5281/zenodo.23397456 ; CAD sources and STL files https://doi.org/10.5281/zenodo.23397779
- **Previous version:** Rebelia V1, OSHWA IT000021, https://doi.org/10.5281/zenodo.23394100 , technical disclosure published on Technical Disclosure Commons
- **License of this document:** CC BY 4.0

---

## Abstract

An anthropomorphic hand, manufacturable by 3D printing, usable as a robotic
end effector and as an active prosthetic hand. Five rotary actuators are housed
inside an internal skeleton of the hand. Each finger is driven by one actuator
through an antagonistic pair of tendon ends, one for flexion and one for
extension. Each tendon end runs inside its own low-friction Bowden tube (PTFE)
from the finger to the actuator; the tubes are seated in holes, pass-through
tunnels and slots formed in the printed skeleton, which separate and guide the
tendons without a separate tendon barrier. On each actuator flange a two-part
spool, made of a flexion spool and an extension spool stacked and fixed by the
same screws, takes up one tendon end while releasing the other. The ring and
little fingers share one actuator, with the tendon ends of both fingers fixed
directly to the same spool. The thumb-rotation actuator, carrying the thumb base
on its flange, is mounted on a separate platform inserted into the skeleton.
Fingers are modular, with tendons pre-installed. The controller reads position,
load, voltage, temperature and current from each actuator, performs whole-hand
calibration and force-limited grasps, streams feedback for ROS, and supports
two-channel EMG control in the prosthetic version.

## Keywords

robotic hand; prosthetic hand; artificial hand; gripper; end effector;
tendon-driven; cable-driven; Bowden cable; Bowden tube; PTFE tube; sheath;
antagonistic tendons; flexor tendon; extensor tendon; spool; two-part spool;
stepped spool; capstan; underactuated; finger coupling; thumb opposition;
actuator platform; modular finger; identical fingers; interchangeable finger; servo load feedback; tendon calibration;
force-limited grasp; ROS; EMG; myoelectric prosthesis; 3D-printed hand

## Background

In tendon-driven hands, tendons that run freely inside the hand rub against
each other and against the structure, change length when other parts move,
and need barriers or pulleys to stay separated. Spools that carry both tendon
ends in one piece are difficult to thread and tension during assembly.
Coupling two fingers to one actuator usually needs extra parts (clamps,
whiffletrees, sliding loops). Thumb actuators fixed directly in the main
structure are hard to assemble and replace.

## Description

### 1. Overall architecture (Figs. 1, 2)

The hand comprises an internal skeleton, a palm part, upper and lower covers,
four upper fingers, a thumb and a robot or prosthetic interface. Five serial
bus servomotors are housed in slots of the skeleton: index flexion, middle
flexion, ring-and-little flexion, thumb flexion and thumb rotation. Left and
right hand versions are mirrored. The robotic version is mounted on a robot
through a flange according to ISO 9409-1-50-4-M6 (other flanges: ISO 9409-1-40,
PCD-40); 12 V power comes from the robot interface and commands are wireless.

### 2. Thumb rotation servo platform (Fig. 3)

The thumb base is bolted to the output flange of the thumb rotation servo. The
servo is screwed onto a separate platform (TRS platform), and the platform with
servo and thumb base is inserted into the skeleton as a sub-assembly and fixed
with screws. The thumb flexion tendon is driven by the thumb flexion servo in
the skeleton.

### 3. Bowden-tube tendon routing (Fig. 4)

Each upper finger has a flexion end and an extension end of its tendon. Each
end runs inside its own PTFE tube. For the flexion end, the tube enters an
outlet hole on the rear of the skeleton, crosses to an inlet hole on the front
near the finger slot, and is seated in an initial slot; its other end is seated
in a final slot near the spool, passing where needed through pass-through
tunnels formed in the skeleton. For the extension end, the tube runs from an
initial slot on the rear of the skeleton, through tunnels, to a final slot near
the spool. Tubes are cut to length. The skeleton geometry (holes, tunnels,
slots) thus defines and fixes the path of every tendon end; tendons are
separated from each other and from moving parts, and the path length is
independent of the motion of other parts. A separate tendon barrier, used in
Rebelia V1, is no longer needed. Tendons are threaded through the tubes with a loop of
nylon line used as a needle.

### 4. Two-part antagonistic spool (Figs. 5, 6)

Each flexion servo carries a spool assembly made of two parts mounted one above
the other: a flexion spool (lower, with a stepped profile having its smaller
diameter at the bottom) and an extension spool (upper). During installation the
two parts are held together only by interlocking and by screws inserted but not
tightened. The flexion end of the tendon is threaded through a hole of the
flexion spool and out through the corresponding hole of the extension spool;
the extension end is threaded through holes of the extension spool. With the
servo in the tendon-installation position, the assembly is screwed to the servo
flange; the extension end is wound one turn clockwise around the extension
spool, pulled until the finger is straight, wrapped around a bolt and locked by
tightening the bolt; the flexion end is then pulled taut without moving the
finger, wrapped around a second bolt and locked. Rotation of the servo in one
direction winds the flexion end and unwinds the extension end (closing); the
opposite rotation opens the finger. Both directions are actively driven, without
return springs. The two parts can have different diameters so that take-up and
release match the finger kinematics. The thumb uses a single spool carrying both
ends.

### 5. Direct ring–little coupling on the spool

The ring and little fingers share one servo. The flexion ends of both fingers
are fixed together to the flexion part of the same two-part spool, and the
extension ends of both fingers to the extension part, without clamps or sliding
loops (used in Rebelia V1). The ring and little fingers therefore close and open
together, driven by the same spool rotation.

### 6. Modular, identical upper fingers

The four upper fingers (index, middle, ring, little) use one and the same finger
design: identical phalanges, joints, inserts, covers and pads, interchangeable
between positions. One finger module is produced and stocked for all four
positions, which reduces part count, tooling (including silicone pad moulds),
assembly steps and spare parts. Each finger is supplied with its tendon already threaded through the phalanges
and anchored at the distal phalanx. The finger is inserted into its slot on the
skeleton and fixed with a screw and a nut; the free flexion and extension ends
are then pulled into their Bowden tubes and fixed to the spool. Fingers can be
replaced individually. Grip surfaces use rigid and flexible printed polymers
with replaceable silicone pads.

### 7. Control and feedback (Figs. 7, 8)

The controller (ESP32) commands the servos over a serial bus and reads from each
servo position, load, voltage, temperature and current. Functions of the
robotic firmware (RebeliaRobot):

- whole-hand calibration: each servo, including thumb rotation, is driven
  towards each end of its travel until its load limit is reached, moved back
  by a small increment, and that position is stored as the range limit
  (compensating tendon length and stretch); per-servo ID registration
  programs;
- grasp presets (power, small power, hook/"monkey", pinch, relax) driven by a
  single 0–100% closure value, and RAW control of each degree of freedom;
- force-limited closing, each finger stopping when its load reaches a limit;
- protection thresholds for over-current and over-temperature;
- ROS mode: commands set the five degrees of freedom directly
  ("ROS a b c d e"), and the controller streams position, load, voltage,
  temperature and current of all servos about every 100 ms;
- tendon installation mode, moving servos to predefined positions;
- left/right hand selected in firmware; Bluetooth and Wi-Fi links;

Prosthetic firmware (RebeliaHuman, same as in Rebelia V1): two-channel EMG
control, where the FFT intensity of each channel is compared with its own
threshold to open, close or hold the hand, with grasp presets, force-limited
closing, automatic relax and grasp selection by manual thumb positioning. The
automatic calibration routine belongs to the robotic firmware only.

## Figures

The figures are included in the PDF version of this disclosure
(`technical-disclosure.pdf` in this folder). Fig. 2 is rendered from the Yeah Hand STL
files of release v1.2; Figs. 1 and 3–6 are taken from the Yeah Hand assembly
instructions (Rev 1.8); Figs. 7–8 are drawn from the published firmware.

- Fig. 1 — Overall view: upper fingers, thumb, thumb base, palm, covers and attachment flange.
- Fig. 2 — Yeah Hand skeleton, front and rear (holes, tunnels and slots for the Bowden tubes); skeleton with TRS platform, thumb base, index finger and ISO 9409 flange.
- Fig. 3 — Thumb base on the thumb rotation servo; servo and thumb base on the TRS platform; TRS platform installed in the skeleton.
- Fig. 4 — Bowden tubes installed in the skeleton, front and rear views.
- Fig. 5 — Two-part spool: extension spool (top) and flexion spool (bottom), before and after assembly with the locking bolts.
- Fig. 6 — Tendon installation on the two-part spool: pre-winding and locking of the extension end and of the flexion end.
- Fig. 7 — Flow chart of calibration and force-limited closing (robotic firmware).
- Fig. 8 — Flow chart of the two-channel EMG decision rule (prosthetic version).

## Alternative embodiments

Tubes may be PTFE, nylon, polyethylene or coil-wound steel housings, one per
tendon end or multi-lumen, separate or formed as channels inside the printed
skeleton, and may extend outside the hand to actuators in the forearm, on the
robot or remote. The two-part spool may have equal or different diameters,
stepped, conical or cam-shaped sections, parts rotatable with respect to each
other for pre-tensioning and then locked, more than two parts for coupled
fingers, and tendon locking by bolt, set screw, clamp plate or wedge. Upper fingers may
use one identical module, or modules of individual lengths and proportions for
each position (index, middle, ring, little), scaled or anthropometric sizes,
or a common module with position-specific phalanx lengths or covers. Two or
more fingers may be coupled directly on one spool, with equal or different
diameters per finger to set their relative speed. The actuator platform may
carry one or more actuators (thumb rotation, thumb flexion, wrist) and be
removable as a module. Feedback may be sent over serial, Bluetooth, Wi-Fi, USB,
CAN or Ethernet to ROS, ROS 2 or other middleware, and used for slip detection,
thermal protection, health monitoring and learning-based control. EMG control
may use N channels and FFT, machine-learning or combined processing. The wrist
may rotate about the X, Y and/or Z axis. Further variants are listed in
`VARIANTS.md` of the repository, which is part of this disclosure, and in the
Rebelia V1 disclosure.

## Advantages

- Every tendon end has a fixed, protected path; no tendon barrier or pulleys.
- Tendon path length independent of other moving parts; less friction and wear.
- Two-part spool simplifies threading and tensioning, and allows different
  take-up and release diameters.
- Two fingers coupled on one actuator with no extra parts.
- Thumb rotation actuator assembled and replaced as a module.
- One identical, interchangeable module for all four upper fingers, supplied
  with tendons pre-installed and replaceable individually.
- Rich feedback for safe operation and integration with ROS.

## Applications

Robot end effectors, collaborative and service robots, humanoid robots,
teleoperation, research and education, upper-limb prostheses, rehabilitation
and exoskeleton devices.
