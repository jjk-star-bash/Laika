#!/usr/bin/env python3
# =============================================================================
# Laika Mission Daemon
# Joshua Kraus -- Lehigh University Rocketry / Laika Project 2026
# ORCID: 0009-0006-3163-3312
# APACHE 2.0 LICENSE -- Please read & cite
#
# "The Earth is the cradle of humanity, but mankind cannot stay in the
#  cradle forever." -- Konstantin Tsiolkovsky
#
# =============================================================================
# MISSION OVERVIEW
# =============================================================================
#
# Laika is a quadruped robot dog integrated as a rocket payload.
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
#   2. STATE MACHINE  (ON_PAD -> FLIGHT -> LANDED)
#      Uses the onboard 9-axis IMU (via STM32 co-processor over UART) to
#      detect flight phase by monitoring roll/pitch Euler angles:
#
#      ON_PAD  -- Dog is vertical in rocket. IMU is static. Waiting for launch.
#                 Detection: angular rate (delta-angle/sample) stays near noise
#                 floor. Launch is a jerk -- N consecutive large delta samples.
#
#      FLIGHT  -- Any motion after pad. Covers ascent, coast, apogee, descent
#                 inside rocket, ejection, and freefall under chute. All of
#                 this is flight -- discriminating between phases with a cheap
#                 IMU is unreliable and unnecessary. One optical photo every
#                 15s for the full flight profile.
#
#      LANDED  -- Angular motion drops to near zero and holds. Static is the
#                 only landing condition -- angle is irrelevant.
#                 Physics: you cannot hold any angle statically in the air
#                 under a single tether chute. The moment std dev drops below
#                 threshold and stays there, you are on the ground regardless
#                 of orientation or terrain.
#
#      Launch detection: angular rate proxy (delta-angle between consecutive
#      samples). N consecutive samples exceeding threshold -> FLIGHT.
#      Counter resets on any quiet sample -- must be sustained jerk.
#
#      Landing detection: rolling std dev of roll < 2.0 deg AND pitch < 6.0 deg
#      over a 100-sample window (~10s at 10Hz). Thresholds set from measured
#      noise floor with margin: roll ~0.7 deg still -> 2.0 deg threshold,
#      pitch ~4.7 deg still -> 6.0 deg threshold. Min 60s in FLIGHT before
#      landing evaluation begins.
#
#   3. POST-LANDING SEQUENCE
#      a. Stand up             -> action(2) with IMU stabilization
#      b. Lean forward 10 deg  -> shifts weight off rear to stabilize stance
#      c. 5 optical photos     -> onboard camera, 5s between shots
#      d. 5 thermal photos     -> Seek Thermal CompactPRO via libseek-thermal
#      e. Three-axis motion    -> action(10), rotates through roll/pitch/yaw
#      f. Walk forward 3s      -> slow pace
#      g. Three-axis motion    -> final survey pass
#      h. Stand                -> action(2), final state (mission success)
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
#   Contents: flight_0-N.jpg, ground_0-4.jpg, thermal_0-4.png
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
#     Joshua Kraus -- Lead Payload Engineer
#   Yahboom Technologies -- DOGZILLA S1 platform & SDK
#     https://www.yahboom.net/study/DOGZILLA
#   OpenThermal -- libseek-thermal open source driver
#     https://github.com/OpenThermal/libseek-thermal
#   OpenCV -- Computer vision library
#     https://opencv.org
#   Anthropic -- Claude, who helped me put all the noise
#      togther as one unified program. Thank you.
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
# LOGGING -- writes to file for headless debugging, mirrors to stdout/journal
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

# -- Prone motor positions -----------------------------------------------------
# NOTE: hardcoded servo positions -- not fed to inverse kinematics.
PRONE_MOTORS = {
    'FL': ([11, 12, 13], [-100, 100,  -5]),
    'FR': ([21, 22, 23], [-100, 110,  -5]),
    'BR': ([31, 32, 33], [-100, 110,  -5]),
    'BL': ([41, 42, 43], [-100, 110,  -5]),
}

# -- State machine -------------------------------------------------------------

SAMPLE_INTERVAL_S  = 0.10       # 10 Hz

# ON_PAD -> FLIGHT : angular rate proxy (total delta-angle per sample, degrees)
# At rest on pad, IMU noise gives ~0.5-1.0 deg/sample combined.
# Rocket rail exit + vibration gives 5-15+ deg/sample. Clear separation.
# Counter resets on any quiet sample -- must be N *consecutive* exceedances.
LAUNCH_RATE_THRESH = 20.0       # deg/sample combined |droll| + |dpitch|
                                 # IMU convergence noise tops ~8 deg/sample on cold boot
                                 # Rail exit under thrust is 50-100+ deg/sample -- clear separation
LAUNCH_CONFIRM_N   = 20         # consecutive samples required (~2s) -- rejects wind gusts,
                                 # any real motor burn sustains this easily

# FLIGHT -> LANDED : sustained stillness via rolling std dev
# No seeding loop -- window fills naturally. Evaluation guarded by len check.
STATIC_WINDOW      = 200        # samples (~20s at 10Hz) -- long enough that pendulum sway cannot fake stillness
ROLL_STD_MAX       = 2.0        # deg -- tight, near bench noise floor. only truly static passes
PITCH_STD_MAX      = 5.0        # deg -- same reasoning
YAW_DELTA_MAX      = 0.5        # deg/sample mean absolute yaw delta -- replaces std dev
                                 # raw yaw wraps at +-180 deg, making std dev meaningless.
                                 # mean sample-to-sample delta with wraparound correction is immune.
                                 # on ground yaw is friction-locked, delta ~0. in air it drifts freely.
MIN_FLIGHT_S       = 60.0       # guard: must be in FLIGHT at least 60s before landing evaluation
MISSION_TIMEOUT_S  = 480.0      # 8 min hard fallback -- if landing never detected after launch,
                                 # initiate post-landing sequence regardless. Bypasses static check.

# LANDED confirmation
LANDED_CONFIRM_S   = 5.0        # static condition must hold for 5s before returning

# -- Action IDs ----------------------------------------------------------------
ACTION_STAND_UP   = 2
ACTION_THREE_AXIS = 10          # built-in 3_Axis preset -- roll/pitch/yaw survey

# -- Camera --------------------------------------------------------------------
OPTICAL_CAMERA_INDEX = 0        # /dev/video0
PHOTO_COUNT          = 5        # photos per ground capture session
FLIGHT_PHOTO_DELAY_S   = 15.0   # seconds between in-flight optical photos
GROUND_PHOTO_DELAY_S   = 5.0    # seconds between ground photos

# -- Thermal -------------------------------------------------------------------
SEEK_SNAPSHOT_BIN = "/home/pi/libseek-thermal/build/examples/seek_snapshot"
THERMAL_COLORMAP  = 11          # COLORMAP_JET -- standard thermal palette

# -- Output --------------------------------------------------------------------
SAVE_BASE = Path("/home/pi/Pictures/laika_mission")

# =============================================================================
# STATES
# =============================================================================

class State(Enum):
    ON_PAD = auto()   # vertical in rocket, static
    FLIGHT = auto()   # any motion -- ascent through freefall under chute
    LANDED = auto()   # static on the ground

# =============================================================================
# HELPERS
# =============================================================================

def send_prone(dog: Laika) -> None:
    """Send all four leg motor positions to hold prone -- no reset to avoid bounce."""
    for leg, (ids, angles) in PRONE_MOTORS.items():
        dog.motor(ids, angles)


def _rollingstd(window: deque) -> float:
    """Population std dev of a deque. Returns 0.0 if fewer than 2 samples."""
    n = len(window)
    if n < 2:
        return 0.0
    mean = sum(window) / n
    return (sum((v - mean) ** 2 for v in window) / n) ** 0.5


def _yaw_delta(a: float, b: float) -> float:
    """Wraparound-safe absolute yaw delta. Handles the +-180 deg boundary."""
    d = b - a
    if d > 180:  d -= 360
    if d < -180: d += 360
    return abs(d)


def _take_optical(save_dir: Path, filename: str) -> None:
    """Capture a single frame from the onboard optical camera and save it."""
    cap = cv2.VideoCapture(OPTICAL_CAMERA_INDEX)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            path = save_dir / filename
            cv2.imwrite(str(path), frame)
            log.info(f"  Photo saved: {filename}")
        else:
            log.warning(f"  Frame capture failed: {filename}")
        cap.release()
    else:
        log.warning(f"  Could not open camera for: {filename}")

# =============================================================================
# PHASE 1: PRONE FIGHT LOOP
# =============================================================================

def phase_prone(dog: Laika, duration_s: float = 10.0) -> None:
    """
    Reset once at start to clear firmware state, then repeatedly send prone
    motor positions. No reset inside loop -- avoids the bounce/dance caused
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
    Blocks until LANDED is confirmed, then returns.

    Transitions:
      ON_PAD -> FLIGHT  : N consecutive samples with |droll|+|dpitch| > threshold
                          (angular rate proxy -- catches jerk at launch, resets on quiet)
      FLIGHT -> LANDED  : roll_std < 2 deg AND pitch_std < 6 deg over 100-sample window
                          + 60s minimum in FLIGHT
      LANDED -> (return): static condition holds for 5s

    No seeding loop. Window builds naturally from first valid sample.
    Launch detection uses angular rate, not std dev -- avoids false trigger
    from noise in a freshly filled window.
    One optical photo every 15s throughout FLIGHT.
    """

    roll_window       = deque(maxlen=STATIC_WINDOW)
    pitch_window      = deque(maxlen=STATIC_WINDOW)
    yaw_delta_window  = deque(maxlen=STATIC_WINDOW)  # stores per-sample yaw deltas, not raw yaw

    state         = State.ON_PAD
    state_entered = time.time()

    prev_roll        = None
    prev_pitch       = None
    prev_yaw         = None
    launch_hit_count = 0          # consecutive samples exceeding launch rate threshold

    flight_photos     = 0
    last_flight_photo = 0.0

    log.info("State machine started.")
    log.info(f"  Launch threshold : |droll|+|dpitch| > {LAUNCH_RATE_THRESH} deg/sample x {LAUNCH_CONFIRM_N} consecutive -- launch is 50-100+ deg/sample")
    log.info(f"  Static threshold : roll_std < {ROLL_STD_MAX} deg, pitch_std < {PITCH_STD_MAX} deg, yaw_delta_mean < {YAW_DELTA_MAX} deg/sample over {STATIC_WINDOW} samples (~20s)")
    log.info(f"  Min flight time  : {MIN_FLIGHT_S}s before landing evaluation")
    log.info(f"  Mission timeout  : {MISSION_TIMEOUT_S}s after launch -- hard fallback to post-landing")
    log.info("STATE: ON_PAD -- ready. Waiting for launch.")

    while True:
        roll  = dog.read_roll()
        time.sleep(SAMPLE_INTERVAL_S / 3)
        pitch = dog.read_pitch()
        time.sleep(SAMPLE_INTERVAL_S / 3)
        yaw   = dog.read_yaw()
        time.sleep(SAMPLE_INTERVAL_S / 3)

        # Discard bad reads -- all zero usually means UART dropout
        if roll == 0.0 and pitch == 0.0 and yaw == 0.0:
            log.warning("  Bad IMU read -- skipping")
            continue

        roll_window.append(roll)
        pitch_window.append(pitch)
        if prev_yaw is not None:
            yaw_delta_window.append(_yaw_delta(prev_yaw, yaw))

        now           = time.time()
        time_in_state = now - state_entered
        prev_state    = state

        # -- Angular rate proxy ------------------------------------------------
        # Total absolute angle change from last sample.
        # Large on launch (jerk), small on pad (noise floor ~0.5-1.0 deg).
        if prev_roll is not None:
            delta = abs(roll - prev_roll) + abs(pitch - prev_pitch)
        else:
            delta = 0.0
        prev_roll, prev_pitch, prev_yaw = roll, pitch, yaw

        # -- Static condition --------------------------------------------------
        is_static = (
            len(roll_window)      >= STATIC_WINDOW
            and len(yaw_delta_window) >= STATIC_WINDOW
            and _rollingstd(roll_window)  < ROLL_STD_MAX
            and _rollingstd(pitch_window) < PITCH_STD_MAX
            and (sum(yaw_delta_window) / len(yaw_delta_window)) < YAW_DELTA_MAX
        )

        # -- Transitions -------------------------------------------------------

        if state == State.ON_PAD:
            if delta > LAUNCH_RATE_THRESH:
                launch_hit_count += 1
            else:
                launch_hit_count = 0      # reset -- must be consecutive exceedances

            if launch_hit_count >= LAUNCH_CONFIRM_N:
                state = State.FLIGHT
                last_flight_photo = now

        elif state == State.FLIGHT:
            # Optical photo every 15s throughout entire flight profile
            if now - last_flight_photo >= FLIGHT_PHOTO_DELAY_S:
                _take_optical(save_dir, f"flight_{flight_photos}.jpg")
                flight_photos     += 1
                last_flight_photo  = now

            if is_static and time_in_state >= MIN_FLIGHT_S:
                state = State.LANDED

            if time_in_state >= MISSION_TIMEOUT_S:
                log.warning(
                    f"Mission timeout reached ({MISSION_TIMEOUT_S}s in FLIGHT) -- "
                    f"static check never confirmed. Initiating post-landing sequence."
                )
                return

        elif state == State.LANDED:
            if not is_static:
                state = State.FLIGHT   # motion resumed -- not on ground, back to monitoring
            elif time_in_state >= LANDED_CONFIRM_S:
                log.info(
                    f"LANDED confirmed -- roll={roll:.1f} deg  pitch={pitch:.1f} deg  "
                    f"time_in_state={time_in_state:.1f}s"
                )
                return

        # -- Log transitions ---------------------------------------------------
        if state != prev_state:
            state_entered = now
            log.info(
                f"[{prev_state.name} -> {state.name}]  "
                f"roll={roll:.1f} deg  pitch={pitch:.1f} deg  yaw={yaw:.1f} deg  "
                f"delta={delta:.2f} deg/sample  "
                f"static={is_static}  "
                f"time_in_{prev_state.name.lower()}={time_in_state:.1f}s"
            )

# =============================================================================
# PHASE 3: POST-LANDING SEQUENCE
# =============================================================================

def stand_up(dog: Laika) -> None:
    """Stand up with IMU stabilization enabled."""
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


def three_axis_motion(dog: Laika) -> None:
    """
    Runs the built-in 3_Axis preset (action 10).
    Dog rotates through roll/pitch/yaw -- surveys the surroundings.
    """
    log.info(f"Running three-axis motion (action {ACTION_THREE_AXIS})...")
    dog.action(ACTION_THREE_AXIS)
    time.sleep(3.0)
    log.info("Three-axis motion done.")


def capture_optical_ground(save_dir: Path) -> None:
    """
    Captures PHOTO_COUNT frames from the onboard camera after landing.
    5 seconds between shots to allow the dog to settle between positions.
    Saves as ground_0.jpg through ground_N.jpg.
    """
    log.info("Opening optical camera for ground photos...")
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


def capture_thermal(save_dir: Path) -> None:
    """
    Captures PHOTO_COUNT thermal frames using seek_snapshot CLI.
    Seek Thermal CompactPRO via libseek-thermal.
    -w 0 skips warmup frames which caused LIBUSB_ERROR_PIPE on this device.
    """
    log.info("Capturing thermal images...")
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
    Then three-axis motion, then stand -- final state.
    """
    log.info("Walking forward (slow, 3s)...")
    three_axis_motion(dog)
    dog.pace("slow")
    dog.move_x(10)
    time.sleep(3.0)
    dog.stop()
    log.info("Walk complete.")

    three_axis_motion(dog)

    log.info(f"Standing by (action {ACTION_STAND_UP}) -- mission complete.")
    dog.action(ACTION_STAND_UP)

# =============================================================================
# MAIN
# =============================================================================

def main():
    log.info("=" * 60)
    log.info("Laika Mission Daemon -- boot start")
    log.info("=" * 60)

    # -- Output directory (timestamped per mission) ----------------------------
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir  = SAVE_BASE / timestamp
    save_dir.mkdir(parents=True, exist_ok=True)
    log.info(f"Media output: {save_dir}")

    try:
        # -- Init serial -------------------------------------------------------
        log.info("Initializing Laika serial connection...")
        dog = Laika(port="/dev/ttyAMA0")
        time.sleep(1.0)

        # -- Phase 1: Prone fight loop -----------------------------------------
        phase_prone(dog, duration_s=10.0)

        # -- Phase 2: State machine --------------------------------------------
        log.info("Settling after prone loop (3s)...")
        time.sleep(3.0)   # let motors and IMU settle before state machine starts
        phase_state_machine(dog, save_dir)

        # -- Phase 3: Post-landing ---------------------------------------------
        log.info("Beginning post-landing sequence...")

        stand_up(dog)
        lean_forward(dog)
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
