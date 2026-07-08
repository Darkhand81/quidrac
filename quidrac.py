#!/usr/bin/env python3
"""
quidrac.py

Custom software-based thermal control loop for Dell iDRAC, using raw IPMI
fan override (since racadm/native thermal-profile tuning isn't available
in this environment).

Because we're fully overriding iDRAC's automatic algorithm, this script
IS the thermal control loop -- it must run continuously. Run it under a
process supervisor (systemd, supervisord, etc.) so it restarts
automatically. For belt-and-braces safety under systemd, also add an
external backstop that reverts to automatic control if the unit stops
for any reason:

    ExecStopPost=/usr/bin/ipmitool -I lanplus -H <host> -U <user> -E \\
        raw 0x30 0x30 0x01 0x01

STRATEGY (temperature -> speed curve with asymmetric slew):
  - Fan speed is a pure function of the hottest monitored sensor:

        temp <= --target-temp     ->  --base-speed
        temp >= --hard-cap-temp   ->  --max-speed
        in between                ->  linear interpolation

  - Rising temps take effect instantly: if the curve says a higher speed,
    it is applied on that same poll (so hitting the hard cap jumps
    straight to max speed -- no gradual ramp during an emergency).
  - Falling temps decay slowly: speed drops by at most --fall-rate
    percent per poll, and never below the curve. The speed therefore
    settles at the lowest value that holds temperature at the curve --
    keeping fans as quiet as possible without overheating.
  - Hysteresis: the curve is driven by a "control temperature" that
    follows raw readings upward instantly but only follows them down
    after they drop by --temp-hysteresis degrees. IPMI sensors report
    whole degrees, so a reading flickering between e.g. 67C and 68C
    would otherwise bounce the fans every few polls; with hysteresis the
    speed settles at the top of the flicker band and stays there until
    the temperature genuinely falls.

SAFETY / FAILURE HANDLING:
  - Dead-man's switch: after --max-failed-polls consecutive failed polls
    (IPMI errors, timeouts, or no sensor readings) the script assumes it
    is flying blind and reverts iDRAC to automatic fan control, retrying
    that revert on every subsequent failed poll until it succeeds. When
    polling recovers, manual control is re-engaged automatically.
  - Manual mode and fan speed are re-asserted on EVERY poll (both raw
    commands are cheap and idempotent), so an iDRAC reset -- firmware
    update, racreset, watchdog -- can't silently drop the override while
    we keep believing it's active.
  - Cleanup (revert to auto, or hold --exit-speed) runs in a finally
    block, so it happens on SIGINT/SIGTERM *and* on crashes, not just
    clean signals.

CREDENTIALS:
  Prefer setting the IPMI_PASSWORD environment variable over passing
  --password on the command line (argv is visible to every local user
  via ps). Either way, the password is handed to ipmitool through its
  environment (-E), never on the ipmitool command line.

Requires: ipmitool, IPMI-over-LAN enabled in iDRAC.
"""

import argparse
import os
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
DEFAULT_PASSWORD = None            # prefer the IPMI_PASSWORD env var over hardcoding here

DEFAULT_SENSORS = "0Eh,0Fh"        # Sensor IDs to monitor (see `ipmitool sdr list`)
DEFAULT_POLL_INTERVAL = 10         # Seconds between polls

DEFAULT_BASE_SPEED = 30            # Minimum fan speed (%), used at/below target temp
DEFAULT_TARGET_TEMP = 65           # Curve start: temps at/below this run at base speed (C)
DEFAULT_HARD_CAP_TEMP = 80         # Curve end: temps at/above this run at max speed (C)
DEFAULT_MAX_SPEED = 100            # Maximum fan speed (%)
DEFAULT_FALL_RATE = 2.0            # Max fan speed decrease per poll when cooling (%/poll)
DEFAULT_TEMP_HYSTERESIS = 2.0      # Temp must fall this far below its recent peak before fans follow (C)

DEFAULT_MAX_FAILED_POLLS = 5       # Consecutive failed polls before failsafe revert to auto
DEFAULT_REVERT_ON_EXIT = True      # Revert to automatic iDRAC fan control on exit
DEFAULT_EXIT_SPEED = DEFAULT_BASE_SPEED  # If REVERT_ON_EXIT is False, hold this speed (%) on exit instead
# ---------------------------------------------------------------------------


def log(msg, level="INFO"):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] [{level}] {msg}", flush=True)


class IdracIpmi:
    def __init__(self, host, user, password, interface="lanplus"):
        self.host = host
        self.user = user
        self.interface = interface
        # Password goes to ipmitool via its environment (-E), keeping it
        # out of /proc/*/cmdline. Some ipmitool builds check
        # IPMITOOL_PASSWORD for -E, others IPMI_PASSWORD; set both.
        self._env = dict(os.environ)
        self._env["IPMI_PASSWORD"] = password
        self._env["IPMITOOL_PASSWORD"] = password

    def _run(self, args, timeout=15):
        cmd = [
            "ipmitool", "-I", self.interface,
            "-H", self.host, "-U", self.user, "-E",
        ] + args
        result = subprocess.run(cmd, capture_output=True, text=True,
                                timeout=timeout, env=self._env)
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


class CurveController:
    """
    Temperature->speed curve plus asymmetric slew limiting: speed rises
    instantly to the curve, but falls at most fall_rate percent per
    poll. The curve is driven by a hysteresis-filtered control
    temperature (rises instantly, falls only after a temp_hysteresis
    drop) so single-degree sensor flicker doesn't bounce the fans.
    """

    def __init__(self, base_speed, target_temp, hard_cap_temp, max_speed,
                 fall_rate, temp_hysteresis):
        if hard_cap_temp <= target_temp:
            raise ValueError("hard-cap-temp must be greater than target-temp")
        self.base_speed = base_speed
        self.target_temp = target_temp
        self.hard_cap_temp = hard_cap_temp
        self.max_speed = max_speed
        self.fall_rate = fall_rate
        self.temp_hysteresis = temp_hysteresis

        self.current_speed = float(base_speed)
        self.control_temp = None

    def curve(self, temp):
        """Desired fan speed (%) for a given temperature."""
        if temp <= self.target_temp:
            return float(self.base_speed)
        if temp >= self.hard_cap_temp:
            return float(self.max_speed)
        frac = (temp - self.target_temp) / (self.hard_cap_temp - self.target_temp)
        return self.base_speed + frac * (self.max_speed - self.base_speed)

    def evaluate(self, max_temp):
        """
        Given the hottest current sensor reading, return the fan speed to
        apply (integer percent). Rise instantly, fall slowly.
        """
        # Hysteresis: follow the raw temp up immediately, but only follow
        # it down once it has genuinely fallen, not on 1C sensor flicker.
        if self.control_temp is None or max_temp >= self.control_temp:
            self.control_temp = max_temp
        elif max_temp <= self.control_temp - self.temp_hysteresis:
            self.control_temp = max_temp

        desired = self.curve(self.control_temp)
        if desired > self.current_speed:
            self.current_speed = desired
        else:
            self.current_speed = max(desired, self.current_speed - self.fall_rate)
        return int(round(self.current_speed))


def run_loop(ipmi, controller, sensor_ids, poll_interval, max_failed_polls):
    consecutive_failures = 0
    failsafe = False           # too many failures; trying to hand back to iDRAC
    failsafe_engaged = False   # the revert-to-auto command actually succeeded
    last_speed = None

    while True:
        try:
            readings = ipmi.read_temps(sensor_ids)
            if not readings:
                raise RuntimeError(
                    f"no readings for sensors {sensor_ids} -- check IDs with "
                    f"'ipmitool sdr type Temperature'")

            if failsafe:
                log("Polling recovered. Re-engaging manual fan control.", level="WARNING")
            consecutive_failures = 0
            failsafe = False
            failsafe_engaged = False

            max_temp = max(readings.values())
            speed = controller.evaluate(max_temp)

            if max_temp >= controller.hard_cap_temp:
                log(f"Temp {max_temp}C at/above hard cap {controller.hard_cap_temp}C. "
                    f"Fans at {speed}%.", level="CRITICAL")

            # Re-assert manual mode and speed every poll: both commands are
            # cheap and idempotent, and this self-heals after an iDRAC
            # reset silently drops the override.
            ipmi.set_manual_mode()
            ipmi.set_fan_speed(speed)

            if speed != last_speed:
                log(f"Fan speed {'set' if last_speed is None else 'changed'} to {speed}% "
                    f"(curve target {controller.curve(controller.control_temp):.1f}% "
                    f"at control temp {controller.control_temp}C).")
                last_speed = speed

            log(f"Readings: {readings} | max={max_temp}C | "
                f"control={controller.control_temp}C | speed={speed}%")

        except Exception as e:
            consecutive_failures += 1
            log(f"Poll failed ({consecutive_failures}/{max_failed_polls}): {e}",
                level="WARNING")

            if consecutive_failures >= max_failed_polls and not failsafe:
                log(f"{consecutive_failures} consecutive failed polls -- flying blind. "
                    f"Failsafe: reverting to automatic iDRAC fan control.", level="CRITICAL")
                failsafe = True
                last_speed = None  # force re-log once we recover

            if failsafe and not failsafe_engaged:
                try:
                    ipmi.set_auto_mode()
                    failsafe_engaged = True
                    log("Failsafe engaged: iDRAC automatic fan control restored. "
                        "Will re-engage manual control when polling recovers.",
                        level="CRITICAL")
                except Exception as e2:
                    log(f"Failsafe revert failed ({e2}); retrying next poll.",
                        level="CRITICAL")

        time.sleep(poll_interval)


def cleanup(ipmi, revert_on_exit, exit_speed, attempts=3):
    """Hand control back to iDRAC (or pin the exit speed). Retries because
    this is the last line of defense before the process goes away."""
    for attempt in range(1, attempts + 1):
        try:
            if revert_on_exit:
                ipmi.set_auto_mode()
                log("Reverted to automatic iDRAC fan control.")
            else:
                ipmi.set_manual_mode()
                ipmi.set_fan_speed(exit_speed)
                log(f"Manual mode retained. Fan speed set to {exit_speed}% on exit.")
            return
        except Exception as e:
            log(f"Cleanup attempt {attempt}/{attempts} failed: {e}", level="ERROR")
            if attempt < attempts:
                time.sleep(2)
    log("Cleanup failed; iDRAC may still be in manual mode at the last set speed!",
        level="CRITICAL")


def main():
    parser = argparse.ArgumentParser(
        description="Custom software fan control loop for iDRAC "
                    "(temperature->speed curve with asymmetric slew).")
    parser.add_argument("--host", default=DEFAULT_HOST, required=DEFAULT_HOST is None)
    parser.add_argument("--user", default=DEFAULT_USER, required=DEFAULT_USER is None)
    parser.add_argument("--password", default=DEFAULT_PASSWORD,
                        help="iDRAC password. Prefer the IPMI_PASSWORD environment "
                             "variable, which keeps it out of process listings.")

    parser.add_argument("--sensors", default=DEFAULT_SENSORS,
                        help=f"Comma-separated sensor IDs to monitor (default: {DEFAULT_SENSORS})")
    parser.add_argument("--poll-interval", type=int, default=DEFAULT_POLL_INTERVAL,
                        help="Seconds between polls")

    parser.add_argument("--base-speed", type=int, default=DEFAULT_BASE_SPEED,
                        help="Minimum fan speed %%, used at/below --target-temp")
    parser.add_argument("--target-temp", type=float, default=DEFAULT_TARGET_TEMP,
                        help="Curve start (C): at/below this temp fans run at --base-speed")
    parser.add_argument("--hard-cap-temp", type=float, default=DEFAULT_HARD_CAP_TEMP,
                        help="Curve end (C): at/above this temp fans jump to --max-speed")
    parser.add_argument("--max-speed", type=int, default=DEFAULT_MAX_SPEED,
                        help="Maximum fan speed %%")
    parser.add_argument("--fall-rate", type=float, default=DEFAULT_FALL_RATE,
                        help="Max fan speed decrease per poll when cooling (%%/poll). "
                             "Rises are always instant.")
    parser.add_argument("--temp-hysteresis", type=float, default=DEFAULT_TEMP_HYSTERESIS,
                        help="Temp must fall this many degrees below its recent peak "
                             "before fans follow it down (C). Prevents fan bounce from "
                             "1C sensor flicker; 0 disables.")

    parser.add_argument("--max-failed-polls", type=int, default=DEFAULT_MAX_FAILED_POLLS,
                        help="Consecutive failed polls before the failsafe reverts "
                             "iDRAC to automatic fan control")
    parser.add_argument("--revert-on-exit", dest="revert_on_exit", action="store_true",
                        default=DEFAULT_REVERT_ON_EXIT,
                        help="Revert to automatic iDRAC fan control on exit (default: on)")
    parser.add_argument("--no-revert-on-exit", dest="revert_on_exit", action="store_false",
                        help="Do NOT revert to automatic control on exit; hold --exit-speed instead")
    parser.add_argument("--exit-speed", type=int, default=DEFAULT_EXIT_SPEED,
                        help="Fan speed %% to hold on exit when --no-revert-on-exit is set "
                             f"(default: {DEFAULT_EXIT_SPEED})")

    args = parser.parse_args()

    password = args.password or os.environ.get("IPMI_PASSWORD")
    if not password:
        parser.error("no password: pass --password or set the IPMI_PASSWORD "
                     "environment variable (preferred)")

    sensor_ids = [s.strip() for s in args.sensors.split(",") if s.strip()]
    ipmi = IdracIpmi(args.host, args.user, password)
    controller = CurveController(
        base_speed=args.base_speed,
        target_temp=args.target_temp,
        hard_cap_temp=args.hard_cap_temp,
        max_speed=args.max_speed,
        fall_rate=args.fall_rate,
        temp_hysteresis=args.temp_hysteresis,
    )

    def handle_shutdown(signum, frame):
        log(f"Received signal {signum}, shutting down.")
        sys.exit(0)  # unwinds through the finally block below

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    log(f"Starting. Monitoring sensors {sensor_ids}. Curve: {args.base_speed}% at "
        f"<={args.target_temp}C -> {args.max_speed}% at >={args.hard_cap_temp}C, "
        f"fall rate {args.fall_rate}%/poll, hysteresis {args.temp_hysteresis}C, "
        f"poll every {args.poll_interval}s, "
        f"failsafe after {args.max_failed_polls} failed polls.")

    engaged = False
    try:
        ipmi.set_manual_mode()
        engaged = True
        ipmi.set_fan_speed(int(controller.current_speed))
        log(f"Manual mode enabled. Initial fan speed set to {int(controller.current_speed)}%.")
        run_loop(ipmi, controller, sensor_ids, args.poll_interval, args.max_failed_polls)
    finally:
        # Don't let a second Ctrl-C interrupt the revert.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if engaged:
            cleanup(ipmi, args.revert_on_exit, args.exit_speed)


if __name__ == "__main__":
    main()
