#!/usr/bin/env python3
# =============================================================================
# Laika Mission Daemon
# Joshua Kraus — Lehigh University Rocketry / Laika Project 2026
# ORCID: 0009-0006-3163-3312
# APACHE 2.0 LICENSE — Please read & cite
#
# "The Earth is the cradle of humanity, but mankind cannot stay in the
#  cradle forever." — Konstantin Tsiolkovsky
#
# =============================================================================
# MISSION OVERVIEW
# =============================================================================
#
# Laika is a DOGZILLA S1 quadruped robot dog integrated as a rocket payload.
# On pin-pull (power-on), this daemon runs automatically via systemd and
# executes the full autonomous mission sequence without human intervention.
#
# FULL MISSION FLOW:
#
#   1. BOOT & PRONE
#      The Yahboom baseboard firmware commands stand-up on power-on.
#      The Pi boots and immediately fights this by holding the legs in a
#      compact prone position for ~10 seconds so the dog fits inside the
#      rocket airframe during flight.
#
#   2. STATE MACHINE  (ON_PAD → LAUNCHED → DESCENDING → LANDED)
#      Uses the onboard 9-axis IMU (via STM32 co-processor over UART) to
#      detect flight phase by monitoring roll/pitch Euler angles:
#
#      ON_PAD     — Dog is vertical in rocket. Gravity vector along body X
#                   axis. IMU is static. Waiting for launch.
#
#      LAUNCHED   — Motion detected (high angular rate from launch thrust
#                   and rail exit). Dog is still inside rocket airframe,
#                   constrained, moving fast.
#
#      DESCENDING — Dog has been ejected from rocket. Body is now level
#                   (gravity vector along body Z axis). Still in motion
#                   due to parachute pendulum swing and body rotation.
#                   Irregular rectangular body on single tether point
#                   ensures continuous angular rate throughout descent.
#                   5 optical photos taken every 15s to capture descent
#                   profile at ~17 ft/s from ~1500 ft (~88s fall time).
#
#      LANDED     — Roll and pitch stable within +/-10 deg, mean angular
#                   rate drops to near zero. Only possible on the ground.
#                   Triggers post-landing sequence.
#
#      Error tolerance: +/-10 deg level threshold, 3.5 deg/sample mean
#      motion threshold over a 15-sample rolling window (~1.5s at ~10Hz).
#
#   3. POST-LANDING SEQUENCE
#      a. Stand up             → action(2) with IMU stabilization
#      b. Lean forward 10 deg  → shifts weight off rear to stabilize stance
#      c. Three-axis survey    → action(10), rotates through roll/pitch/yaw
#      d. 5 optical photos     → onboard camera, 5s between shots
#      e. 5 thermal photos     → Seek Thermal CompactPRO via libseek-thermal
#      f. Walk forward 3s      → slow pace
#      g. Three-axis survey    → final survey pass
#      h. IMU stabilization    → final standby state (mission success)
#
# =============================================================================
# HOW TO ACCESS MISSION DATA
# =============================================================================
#
# View live log (run during or after mission):
#   tail -f /home/pi/laika_mission.log
#
# View full log:
#   cat /home/pi/laika_mission.log
#
# Access photos (optical + thermal, timestamped per mission):
#   ls /home/pi/Pictures/laika_mission/
#   Each mission folder: /home/pi/Pictures/laika_mission/YYYYMMDD_HHMMSS/
#   Contents: descent_0-4.jpg, ground_0-4.jpg, thermal_0-4.png
#
# =============================================================================
# DEPENDENCIES
# =============================================================================
#   pyserial       : pip install pyserial
#   opencv         : sudo apt install python3-opencv
#   libseek-thermal: https://github.com/OpenThermal/libseek-thermal
#                    Build: cd libseek-thermal && mkdir build && cd build
#                           cmake .. && make
#
# =============================================================================
# CITATIONS & CONTRIBUTORS
# =============================================================================
#   Lehigh University Rocketry Association
#     Joshua Kraus — Lead Payload Engineer
#   Yahboom Technologies — DOGZILLA S1 platform & SDK
#     https://www.yahboom.net/study/DOGZILLA
#   OpenThermal — libseek-thermal open source driver
#     https://github.com/OpenThermal/libseek-thermal
#   OpenCV — Computer vision library
#     https://opencv.org
# =============================================================================

import sys
import os
import time
import subprocess
import logging
from collections import deque
from datetime import datetime
from enum import Enum, auto
from pathlib import Path

import cv2

# Ensure LAIKA.py is findable when run as a systemd service (in main dir)
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

from LAIKA import Laika

# =============================================================================
# LOGGING — writes to file for headless debugging, mirrors to stdout/journal
# =============================================================================

LOG_PATH = os.path.join(script_dir, "laika_mission.log")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(LOG_PATH)
    ]
)
log = logging.getLogger("laika")

# =============================================================================
# CONFIGURATION
# =============================================================================

# ── Prone motor positions ─────────────────────────────────────────────────────
# NOTE these are hardcoded servo positions and not fed to inverse kinimatics.
PRONE_MOTORS = {
    'FL': ([11, 12, 13], [-100, 100,  -5]),
    'FR': ([21, 22, 23], [-100, 110,  -5]),
    'BR': ([31, 32, 33], [-100, 110,  -5]),
    'BL': ([41, 42, 43], [-100, 110,  -5]),
}

# ── State machine ─────────────────────────────────────────────────────────────
LEVEL_THRESHOLD_DEG   = 10.0  # |roll| AND |pitch| below this → "upright"
MOTION_THRESHOLD_DEG  = 3.5   # max angle change per sample to be "static"
STATIC_WINDOW         = 15    # consecutive static samples needed (~1.5s @ ~10Hz)
# ── Action IDs (confirmed from Jupyter widget code) ───────────────────────────
ACTION_LIE_DOWN   = 1
ACTION_STAND_UP   = 2
ACTION_THREE_AXIS = 10   # built-in 3_Axis preset — roll/pitch/yaw survey

# ── Camera ────────────────────────────────────────────────────────────────────
OPTICAL_CAMERA_INDEX    = 0     # /dev/video0
PHOTO_COUNT             = 5     # photos per capture session
DESCENT_PHOTO_DELAY_S   = 15.0  # seconds between descent photos
GROUND_PHOTO_DELAY_S    = 5.0   # seconds between ground photos

# ── Thermal ───────────────────────────────────────────────────────────────────
SEEK_SNAPSHOT_BIN = "/home/pi/libseek-thermal/build/examples/seek_snapshot"
THERMAL_COLORMAP  = 11          # COLORMAP_JET — standard thermal palette

# ── Output ────────────────────────────────────────────────────────────────────
SAVE_BASE = Path("/home/pi/Pictures/laika_mission")

# =============================================================================
# HELPERS
# =============================================================================

def send_prone(dog: Laika) -> None:
    """Send all four leg motor positions to hold prone — no reset to avoid bounce."""
    for leg, (ids, angles) in PRONE_MOTORS.items():
        dog.motor(ids, angles)


def is_level(roll: float, pitch: float) -> bool:
    return abs(roll) < LEVEL_THRESHOLD_DEG and abs(pitch) < LEVEL_THRESHOLD_DEG


def is_static(window: deque) -> bool:
    """
    True when the average angular change across the window is below threshold.
    Uses mean instead of max so a single noisy IMU sample doesn't reset detection.
    """
    if len(window) < STATIC_WINDOW:
        return False
    return (sum(window) / len(window)) < MOTION_THRESHOLD_DEG

# =============================================================================
# STATES
# =============================================================================

class State(Enum):
    ON_PAD     = auto()   # vertical in rocket, static
    LAUNCHED   = auto()   # motion detected — off the pad
    DESCENDING = auto()   # body level, still moving — ejected and falling
    LANDED     = auto()   # body level, static — on the ground

# =============================================================================
# PHASE 1: PRONE FIGHT LOOP
# =============================================================================

def phase_prone(dog: Laika, duration_s: float = 10.0) -> None:
    """
    Reset once at start to clear firmware state, then repeatedly send prone
    motor positions. No reset inside loop — avoids the bounce/dance caused
    by repeated resets fighting the firmware every 0.5s.
    """
    log.info(f"Prone fight loop starting ({duration_s}s)...")
    dog.reset()        # single reset to clear firmware state
    time.sleep(0.3)    # let it settle before overriding
    start = time.time()
    while time.time() - start < duration_s:
        send_prone(dog)
        time.sleep(0.5)
    log.info("Prone established.")

# =============================================================================
# PHASE 2: STATE MACHINE
# =============================================================================

def phase_state_machine(dog: Laika, save_dir: Path) -> None:
    """
    Blocks until LANDED is confirmed.
    Takes 5 optical photos every 15s during DESCENDING state.
    """
    state        = State.ON_PAD
    prev_roll    = dog.read_roll()
    prev_pitch   = dog.read_pitch()
    delta_window = deque(maxlen=STATIC_WINDOW)
    sample_count = 0
    descent_photos_taken = 0
    last_descent_photo   = 0.0

    log.info("State machine started.")
    log.info(
        f"  Level: +/-{LEVEL_THRESHOLD_DEG} deg  "
        f"Motion: {MOTION_THRESHOLD_DEG} deg/sample  "
        f"Window: {STATIC_WINDOW} samples"
    )

    while True:
        roll  = dog.read_roll()
        pitch = dog.read_pitch()

        delta = max(abs(roll - prev_roll), abs(pitch - prev_pitch))
        delta_window.append(delta)

        static = is_static(delta_window)
        level  = is_level(roll, pitch)

        prev_roll, prev_pitch = roll, pitch
        sample_count += 1
        prev_state = state

        # ── Transitions (one-directional) ─────────────────────────────────────
        if state == State.ON_PAD:
            if not static:
                state = State.LAUNCHED

        elif state == State.LAUNCHED:
            if level:
                state = State.DESCENDING
                last_descent_photo = time.time()   # start descent photo timer, at ~17ft/s for 1500 feet is an estimated 88 sec. fall time.

        elif state == State.DESCENDING:
            # Take optical photos every 15s during descent. Captures muliple phases of decent profile. 
            now = time.time()
            if (descent_photos_taken < PHOTO_COUNT and
                    now - last_descent_photo >= DESCENT_PHOTO_DELAY_S):
                cap = cv2.VideoCapture(OPTICAL_CAMERA_INDEX)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret:
                        path = save_dir / f"descent_{descent_photos_taken}.jpg"
                        cv2.imwrite(str(path), frame)
                        log.info(f"  Descent photo saved: descent_{descent_photos_taken}.jpg")
                    cap.release()
                descent_photos_taken += 1
                last_descent_photo = now

            if level and static:
                state = State.LANDED

        elif state == State.LANDED:
            log.info(f"LANDED — roll={roll:.1f} deg  pitch={pitch:.1f} deg")
            return

        # ── Log on transition ─────────────────────────────────────────────────
        if state != prev_state:
            log.info(
                f"[{prev_state.name} -> {state.name}]  "
                f"roll={roll:.1f}  pitch={pitch:.1f}  "
                f"delta={delta:.2f}  static={static}  level={level}"
            )

        # Prone is held by Pi once phase_prone() wins — no reassertion needed

# =============================================================================
# PHASE 3: POST-LANDING SEQUENCE
# =============================================================================

def stand_up(dog: Laika) -> None:
    """
    Stand up with full push.
    """
    log.info("Enabling IMU stabilization...")
    dog.imu(1)
    time.sleep(0.5)
    log.info(f"Standing up (action {ACTION_STAND_UP})...")
    dog.action(ACTION_STAND_UP)
    time.sleep(2.0)


def lean_forward(dog: Laika) -> None:
    """
    Pitch forward 10 degrees to shift weight off rear, then return to neutral.
    Helps rear-heavy dog stabilize stance before photos.
    """
    log.info("Leaning forward 10 degrees to shift weight...")
    dog.attitude('p', 10)
    time.sleep(1.5)
    log.info("Returning to neutral stance...")
    dog.attitude('p', 0)
    time.sleep(1.0)


def three_axis_survey(dog: Laika) -> None:
    """
    Runs the built-in 3_Axis preset (action 10).
    Dog rotates through roll/pitch/yaw — surveys the surroundings.
    """
    log.info(f"Running three-axis survey (action {ACTION_THREE_AXIS})...")
    dog.action(ACTION_THREE_AXIS)
    time.sleep(3.0)   # wait for preset to complete
    log.info("Three-axis survey done.")


def capture_optical_ground(save_dir: Path) -> None:
    """
    Captures PHOTO_COUNT frames from the onboard camera after landing.
    5 seconds between shots to allow the dog to settle between positions.
    Saves as ground_0.jpg through ground_N.jpg
    """
    log.info(f"Opening optical camera for ground photos...")
    cap = cv2.VideoCapture(OPTICAL_CAMERA_INDEX)

    if not cap.isOpened():
        log.error("Could not open optical camera. Skipping ground optical capture.")
        return

    time.sleep(1.0)   # let sensor auto-expose

    for i in range(PHOTO_COUNT):
        ret, frame = cap.read()
        if ret:
            path = save_dir / f"ground_{i}.jpg"
            cv2.imwrite(str(path), frame)
            log.info(f"  ground_{i}.jpg saved")
        else:
            log.warning(f"  Ground optical frame {i} failed to capture")
        time.sleep(GROUND_PHOTO_DELAY_S)

    cap.release()
    log.info("Ground optical camera released.")


# =============================================================================
# THERMAL CAPTURE
# =============================================================================

def capture_thermal(save_dir: Path) -> None:
    """
    Captures PHOTO_COUNT thermal frames using seek_snapshot CLI.
    Seek Thermal CompactPRO via libseek-thermal.
    -w 0 skips warmup frames which caused LIBUSB_ERROR_PIPE on this device.
    """
    log.info("Capturing thermal images via seek_snapshot...")
    time.sleep(2.0)   # allow USB device to fully initialize before first capture

    for i in range(PHOTO_COUNT):
        path = save_dir / f"thermal_{i}.png"
        cmd = [
            SEEK_SNAPSHOT_BIN,
            "-t", "seekpro",
            "-w", "0",
            "-c", str(THERMAL_COLORMAP),
            "-r", "270",
            "-o", str(path)
        ]
        try:
            result = subprocess.run(cmd, timeout=15, capture_output=True, text=True)
            if result.returncode == 0:
                log.info(f"  thermal_{i}.png saved")
            else:
                log.warning(f"  seek_snapshot error frame {i}: {result.stderr.strip()}")
        except FileNotFoundError:
            log.error("seek_snapshot not found at expected path.")
            break
        except subprocess.TimeoutExpired:
            log.warning(f"  seek_snapshot timed out on frame {i}")

        time.sleep(GROUND_PHOTO_DELAY_S)

    log.info("Thermal capture complete.")


def walk_sequence(dog: Laika) -> None:
    """
    Walk slowly forward for 3 seconds, stop.
    Then three-axis survey, then IMU stabilization — final standby state.
    """
    log.info("Walking forward (slow, 3s)...")
    dog.pace("slow")
    dog.move_x(10)
    time.sleep(3.0)
    dog.stop()
    log.info("Walk complete.")

    log.info(f"Running post-walk three-axis survey (action {ACTION_THREE_AXIS})...")
    dog.action(ACTION_THREE_AXIS)
    time.sleep(3.0)

    log.info("Enabling IMU stabilization — entering final standby state.")
    dog.imu(1)
    log.info("Mission success.")

# =============================================================================
# MAIN
# =============================================================================

def main():
    log.info("=" * 60)
    log.info("Laika Mission Daemon — boot start")
    log.info("=" * 60)

    # ── Output directory (timestamped per mission) ────────────────────────────
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir  = SAVE_BASE / timestamp
    save_dir.mkdir(parents=True, exist_ok=True)
    log.info(f"Media output: {save_dir}")

    try:
        # ── Init serial ───────────────────────────────────────────────────────
        log.info("Initializing Laika serial connection...")
        dog = Laika(port="/dev/ttyAMA0")
        time.sleep(1.0)

        # ── Phase 1: Prone fight loop ─────────────────────────────────────────
        phase_prone(dog, duration_s=10.0)

        # ── Phase 2: State machine ────────────────────────────────────────────
        phase_state_machine(dog, save_dir)

        # ── Phase 3: Post-landing ─────────────────────────────────────────────
        log.info("Beginning post-landing sequence...")

        stand_up(dog)
        lean_forward(dog)
        three_axis_survey(dog)
        capture_optical_ground(save_dir)
        capture_thermal(save_dir)
        walk_sequence(dog)

        log.info("Mission complete.")

    except Exception as e:
        log.exception(f"Mission daemon crashed: {e}")
        with open(os.path.join(script_dir, "boot_debug.log"), "a") as f:
            f.write(f"{time.ctime()}: Daemon crashed: {str(e)}\n")


if __name__ == "__main__":
    main()
