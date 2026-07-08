#!/usr/bin/env python3
"""
idrac_fan_watchdog.py

Custom software-based thermal ramp control for Dell iDRAC, using raw IPMI
fan override (since racadm/native thermal-profile tuning isn't available
in this environment).

Because we're fully overriding iDRAC's automatic algorithm, this script
IS the thermal control loop -- it must run continuously. If it dies,
fans stay wherever they were last set. Run it under a process supervisor
(systemd, supervisord, etc.) so it restarts automatically, and see the
--revert-on-exit behavior below.

STRATEGY (a staircase ramp):
  - Start at --base-speed (default 30%).
  - Each fan-speed "step" has its own temperature ceiling. The ceiling
    rises slightly with each step, since higher fan speed generally
    buys you more thermal headroom before further action is needed:

        speed:      30%    35%    40%    45%   ...
        ceiling:    70C    72C    74C    76C   ...   (--base-max-temp,
                                                       --step-temp-increment)

  - If the hottest monitored sensor exceeds the ceiling for the CURRENT
    speed, bump fan speed up by --step-percent.
  - If temps reach --hard-cap-temp, keep stepping up by --step-percent
    every poll (not jumping straight to --max-speed) until the max
    reading is held at or below the cap, capping out at --max-speed if
    it's still not enough.
  - If temps drop comfortably below --target-temp for several
    consecutive polls (--ramp-down-debounce), step speed back down,
    never below --base-speed.

All numbers are configurable via CLI flags. Defaults match what was
discussed: 30% base speed, 65C target, 70C ceiling at base speed, 5%
ramp steps, 80C hard cap, 10s poll interval.

Requires: ipmitool, IPMI-over-LAN enabled in iDRAC.
"""

import argparse
import re
import signal
import subprocess
import sys
import time
from datetime import datetime


# ---------------------------------------------------------------------------
# DEFAULT CONFIGURATION
# Edit these directly if you'd rather not pass CLI flags every time.
# Any of these can still be overridden on the command line (CLI wins).
# ---------------------------------------------------------------------------
DEFAULT_HOST = None                # e.g. "192.168.1.100" (still required via --host if left None)
DEFAULT_USER = None                # e.g. "root"
DEFAULT_PASSWORD = None            # e.g. "calvin"

DEFAULT_SENSORS = "0Eh,0Fh"        # Sensor IDs to monitor (see `ipmitool sdr list`)
DEFAULT_POLL_INTERVAL = 10         # Seconds between polls

DEFAULT_BASE_SPEED = 30            # Starting / minimum fan speed (%)
DEFAULT_TARGET_TEMP = 65           # Temp to settle around (C)
DEFAULT_BASE_MAX_TEMP = 70         # Ceiling temp at base speed before ramping up (C)
DEFAULT_STEP_PERCENT = 5           # Fan speed step size (%)
DEFAULT_STEP_TEMP_INCREMENT = 2    # How much the ceiling rises per step (C)
DEFAULT_HARD_CAP_TEMP = 80         # Absolute max temp; jumps to max speed immediately (C)
DEFAULT_MAX_SPEED = 100            # Maximum fan speed (%)
DEFAULT_RAMP_DOWN_DEBOUNCE = 3     # Consecutive below-target polls before ramping down

DEFAULT_REVERT_ON_EXIT = True      # Revert to automatic iDRAC fan control on clean exit
DEFAULT_EXIT_SPEED = DEFAULT_BASE_SPEED  # If REVERT_ON_EXIT is False, set fans to this speed (%) on clean exit instead
# ---------------------------------------------------------------------------


def log(msg, level="INFO"):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] [{level}] {msg}", flush=True)


class IdracIpmi:
    def __init__(self, host, user, password, interface="lanplus"):
        self.host = host
        self.user = user
        self.password = password
        self.interface = interface

    def _run(self, args, timeout=15):
        cmd = [
            "ipmitool", "-I", self.interface,
            "-H", self.host, "-U", self.user, "-P", self.password,
        ] + args
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if result.returncode != 0:
            raise RuntimeError(f"ipmitool failed ({' '.join(args)}): {result.stderr.strip()}")
        return result.stdout

    def set_manual_mode(self):
        self._run(["raw", "0x30", "0x30", "0x01", "0x00"])

    def set_auto_mode(self):
        self._run(["raw", "0x30", "0x30", "0x01", "0x01"])

    def set_fan_speed(self, percent):
        percent = max(0, min(100, int(percent)))
        hex_speed = format(percent, "02x")
        self._run(["raw", "0x30", "0x30", "0x02", "0xff", f"0x{hex_speed}"])

    def read_temps(self, sensor_ids):
        """
        Return {sensor_id_lower: temp_celsius} for the given sensor IDs
        (e.g. ['0eh', '0fh']), by parsing `ipmitool sdr type Temperature`
        output. Scoped to temperature sensors only -- faster and more
        reliable over lanplus than a full `sdr list` walk of every sensor.
        """
        wanted = {sid.lower() for sid in sensor_ids}
        output = self._run(["sdr", "type", "Temperature"])
        readings = {}
        for line in output.splitlines():
            parts = [p.strip() for p in line.split("|")]
            if len(parts) < 5:
                continue
            sensor_id = parts[1].lower()
            if sensor_id not in wanted:
                continue
            m = re.search(r"(-?\d+)\s*degrees", parts[-1], re.IGNORECASE)
            if m:
                readings[sensor_id] = int(m.group(1))
        return readings


class RampController:
    def __init__(self, base_speed, base_max_temp, step_percent, step_temp_increment,
                 target_temp, hard_cap_temp, max_speed, ramp_down_debounce):
        self.base_speed = base_speed
        self.base_max_temp = base_max_temp
        self.step_percent = step_percent
        self.step_temp_increment = step_temp_increment
        self.target_temp = target_temp
        self.hard_cap_temp = hard_cap_temp
        self.max_speed = max_speed
        self.ramp_down_debounce = ramp_down_debounce

        self.current_speed = base_speed
        self._below_target_count = 0

    def ceiling_for(self, speed):
        """Temperature ceiling for a given fan speed (the staircase)."""
        level = max(0, (speed - self.base_speed) // self.step_percent)
        ceiling = self.base_max_temp + level * self.step_temp_increment
        return min(ceiling, self.hard_cap_temp)

    def evaluate(self, max_temp):
        """
        Given the hottest current sensor reading, decide whether to change
        fan speed. Returns the new speed (may be unchanged).
        """
        ceiling = self.ceiling_for(self.current_speed)
        at_hard_cap = max_temp >= self.hard_cap_temp

        # Ramp up: current ceiling exceeded (ceiling is itself capped at
        # hard_cap_temp, so once at max ceiling this keeps stepping by
        # step_percent for as long as temp stays at/above the hard cap,
        # rather than jumping straight to max speed).
        if (max_temp > ceiling or at_hard_cap) and self.current_speed < self.max_speed:
            new_speed = min(self.current_speed + self.step_percent, self.max_speed)
            level = "CRITICAL" if at_hard_cap else "INFO"
            reason = f"at/above hard cap {self.hard_cap_temp}C" if at_hard_cap else f"exceeds ceiling {ceiling}C"
            log(f"Temp {max_temp}C {reason} at {self.current_speed}%. "
                f"Ramping up to {new_speed}%.", level=level)
            self.current_speed = new_speed
            self._below_target_count = 0
            return self.current_speed

        if at_hard_cap and self.current_speed >= self.max_speed:
            log(f"Temp {max_temp}C at/above hard cap {self.hard_cap_temp}C, already at "
                f"max speed {self.max_speed}%. No further headroom available.", level="CRITICAL")
            self._below_target_count = 0
            return self.current_speed

        # Ramp down: comfortably below target for several consecutive polls.
        if max_temp < self.target_temp and self.current_speed > self.base_speed:
            self._below_target_count += 1
            if self._below_target_count >= self.ramp_down_debounce:
                new_speed = max(self.current_speed - self.step_percent, self.base_speed)
                log(f"Temp {max_temp}C below target {self.target_temp}C for "
                    f"{self._below_target_count} polls. Ramping down to {new_speed}%.")
                self.current_speed = new_speed
                self._below_target_count = 0
            return self.current_speed

        # Steady state.
        self._below_target_count = 0
        return self.current_speed


def main():
    parser = argparse.ArgumentParser(description="Custom software fan ramp watchdog for iDRAC.")
    parser.add_argument("--host", default=DEFAULT_HOST, required=DEFAULT_HOST is None)
    parser.add_argument("--user", default=DEFAULT_USER, required=DEFAULT_USER is None)
    parser.add_argument("--password", default=DEFAULT_PASSWORD, required=DEFAULT_PASSWORD is None)

    parser.add_argument("--sensors", default=DEFAULT_SENSORS,
                         help=f"Comma-separated sensor IDs to monitor (default: {DEFAULT_SENSORS})")
    parser.add_argument("--poll-interval", type=int, default=DEFAULT_POLL_INTERVAL,
                         help="Seconds between polls")

    parser.add_argument("--base-speed", type=int, default=DEFAULT_BASE_SPEED,
                         help="Starting/minimum fan speed %%")
    parser.add_argument("--target-temp", type=float, default=DEFAULT_TARGET_TEMP,
                         help="Temp to settle around (C)")
    parser.add_argument("--base-max-temp", type=float, default=DEFAULT_BASE_MAX_TEMP,
                         help="Ceiling temp at base speed before ramping up (C)")
    parser.add_argument("--step-percent", type=int, default=DEFAULT_STEP_PERCENT,
                         help="Fan speed step size %%")
    parser.add_argument("--step-temp-increment", type=float, default=DEFAULT_STEP_TEMP_INCREMENT,
                         help="How much the ceiling rises per step (C)")
    parser.add_argument("--hard-cap-temp", type=float, default=DEFAULT_HARD_CAP_TEMP,
                         help="Absolute max temp; jumps to --max-speed immediately if hit (C)")
    parser.add_argument("--max-speed", type=int, default=DEFAULT_MAX_SPEED,
                         help="Maximum fan speed %%")
    parser.add_argument("--ramp-down-debounce", type=int, default=DEFAULT_RAMP_DOWN_DEBOUNCE,
                         help="Consecutive below-target polls required before ramping down")

    parser.add_argument("--revert-on-exit", dest="revert_on_exit", action="store_true",
                         default=DEFAULT_REVERT_ON_EXIT,
                         help="Revert to automatic iDRAC fan control on clean exit (default: on)")
    parser.add_argument("--no-revert-on-exit", dest="revert_on_exit", action="store_false",
                         help="Do NOT revert to automatic control on exit; hold --exit-speed instead")
    parser.add_argument("--exit-speed", type=int, default=DEFAULT_EXIT_SPEED,
                         help="Fan speed %% to hold on clean exit when --no-revert-on-exit is set "
                              f"(default: {DEFAULT_EXIT_SPEED})")

    args = parser.parse_args()

    sensor_ids = [s.strip() for s in args.sensors.split(",") if s.strip()]
    ipmi = IdracIpmi(args.host, args.user, args.password)
    controller = RampController(
        base_speed=args.base_speed,
        base_max_temp=args.base_max_temp,
        step_percent=args.step_percent,
        step_temp_increment=args.step_temp_increment,
        target_temp=args.target_temp,
        hard_cap_temp=args.hard_cap_temp,
        max_speed=args.max_speed,
        ramp_down_debounce=args.ramp_down_debounce,
    )

    def handle_shutdown(signum, frame):
        log(f"Received signal {signum}, shutting down.")
        if args.revert_on_exit:
            try:
                ipmi.set_auto_mode()
                log("Reverted to automatic iDRAC fan control.")
            except Exception as e:
                log(f"Failed to revert to automatic control: {e}", level="ERROR")
        else:
            try:
                ipmi.set_fan_speed(args.exit_speed)
                log(f"Manual mode retained. Fan speed set to {args.exit_speed}% on exit.")
            except Exception as e:
                log(f"Failed to set exit fan speed: {e}", level="ERROR")
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    log(f"Starting watchdog. Monitoring sensors {sensor_ids}, base_speed={args.base_speed}%, "
        f"target={args.target_temp}C, base_max_temp={args.base_max_temp}C, "
        f"step={args.step_percent}% / {args.step_temp_increment}C, "
        f"hard_cap={args.hard_cap_temp}C, poll={args.poll_interval}s.")

    ipmi.set_manual_mode()
    ipmi.set_fan_speed(controller.current_speed)
    log(f"Manual mode enabled. Initial fan speed set to {controller.current_speed}%.")

    while True:
        time.sleep(args.poll_interval)
        try:
            readings = ipmi.read_temps(sensor_ids)
            if not readings:
                log(f"No readings found for sensors {sensor_ids}. Check sensor IDs with "
                    f"'ipmitool sdr type Temperature'. Skipping this poll.", level="WARNING")
                continue

            max_temp = max(readings.values())
            new_speed = controller.evaluate(max_temp)

            if new_speed != getattr(main, "_last_applied", None):
                ipmi.set_fan_speed(new_speed)
                main._last_applied = new_speed

            log(f"Readings: {readings} | max={max_temp}C | speed={controller.current_speed}% | "
                f"ceiling={controller.ceiling_for(controller.current_speed)}C")

        except subprocess.TimeoutExpired:
            log("ipmitool call timed out. Skipping this poll.", level="WARNING")
        except RuntimeError as e:
            log(f"IPMI error: {e}. Skipping this poll.", level="WARNING")
        except Exception as e:
            log(f"Unexpected error: {e}. Skipping this poll.", level="ERROR")


if __name__ == "__main__":
    main()
