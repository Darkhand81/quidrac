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

WEB INTERFACE:
  A built-in dashboard (stdlib http.server, zero dependencies, works
  offline) serves live charts of temperatures, control temperature,
  target/hard-cap thresholds, and fan speed, plus a form to tweak every
  control parameter at runtime. Parameter changes apply on the next
  poll and are saved to a settings file (default: quidrac-settings.json
  next to this script; override with --settings-file) that takes
  precedence over CLI flags/defaults at startup. "Restore script
  settings" in the UI returns to the CLI/default values and removes the
  file. There is no authentication: bind it to a trusted network only
  (--web-bind, default 0.0.0.0; --web-port, default 8080; --no-web to
  disable).

DEMO MODE:
  --demo runs the full control loop and web UI against a simulated
  server (a simple thermal model with periodic load spikes) instead of
  a real iDRAC -- useful for previewing the dashboard and tuning
  parameters without hardware. No credentials required.

CREDENTIALS:
  Prefer setting the IPMI_PASSWORD environment variable over passing
  --password on the command line (argv is visible to every local user
  via ps). Either way, the password is handed to ipmitool through its
  environment (-E), never on the ipmitool command line.

Requires: ipmitool, IPMI-over-LAN enabled in iDRAC.
"""

import argparse
import json
import math
import os
import random
import re
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs


# ---------------------------------------------------------------------------
# DEFAULT CONFIGURATION
# Edit these directly if you'd rather not pass CLI flags every time.
# Any of these can still be overridden on the command line (CLI wins).
# ---------------------------------------------------------------------------
DEFAULT_HOST = None                # e.g. "192.168.1.100" (still required via --host if left None)
DEFAULT_USER = None                # e.g. "root"
DEFAULT_PASSWORD = None            # prefer the IPMI_PASSWORD env var over hardcoding here

DEFAULT_SENSORS = "0Eh,0Fh"        # Sensor IDs to monitor (see `ipmitool sdr list`)
SENSOR_ALIASES = {                 # Friendly names shown in the dashboard and logs.
    "0Eh": "CPU1",                 # Script-only setting: edit here to match your
    "0Fh": "CPU2",                 # system. Sensors without an entry show their raw ID.
}
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

DEFAULT_WEB_BIND = "0.0.0.0"       # Web UI bind address ("127.0.0.1" for local-only)
DEFAULT_WEB_PORT = 8080            # Web UI port
HISTORY_MAX_SAMPLES = 20000        # Rolling history kept for the charts (~55h at 10s polls)
SETTINGS_FILENAME = "quidrac-settings.json"  # Web UI overrides, saved next to this script
# ---------------------------------------------------------------------------


def log(msg, level="INFO"):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] [{level}] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Runtime-tunable parameters
# ---------------------------------------------------------------------------

# name -> (python type, min, max, label, unit, help)
PARAM_SPEC = {
    "base_speed":       (int,   0,   100, "Base speed",      "%",      "Minimum fan speed, used at/below target temp"),
    "max_speed":        (int,   1,   100, "Max speed",       "%",      "Maximum fan speed"),
    "target_temp":      (float, 20,  105, "Target temp",     "°C", "Curve start: at/below this temp fans run at base speed"),
    "hard_cap_temp":    (float, 25,  110, "Hard cap temp",   "°C", "Curve end: at/above this temp fans jump to max speed"),
    "fall_rate":        (float, 0.1, 100, "Fall rate",       "%/poll", "Max fan speed decrease per poll when cooling (rises are instant)"),
    "temp_hysteresis":  (float, 0,   20,  "Temp hysteresis", "°C", "Temp must fall this far below its recent peak before fans follow"),
    "poll_interval":    (int,   1,   300, "Poll interval",   "s",      "Seconds between sensor polls"),
    "max_failed_polls": (int,   1,   100, "Max failed polls", "",      "Consecutive failed polls before failsafe revert to auto"),
}


def load_settings_file(path, cli_params):
    """Merge saved web-UI overrides from `path` over the CLI/default
    params. Returns the effective params; falls back to cli_params (with
    a warning) if the file is missing keys' worth of sanity or unreadable."""
    if not os.path.exists(path):
        return cli_params
    try:
        with open(path) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError("not a JSON object")
        overrides = {k: v for k, v in data.items() if k in PARAM_SPEC}
        merged = validate_params({**cli_params, **overrides})
        changed = [f"{n}={merged[n]}" for n in PARAM_SPEC if merged[n] != cli_params[n]]
        if changed:
            log(f"Loaded settings overrides from {path}: {', '.join(changed)} "
                f"(web-UI 'Restore script settings' or deleting the file reverts).")
        return merged
    except (OSError, ValueError, json.JSONDecodeError) as e:
        log(f"Ignoring settings file {path}: {e}. Using CLI flags/defaults.",
            level="WARNING")
        return cli_params


def validate_params(params):
    """Range- and cross-check a full params dict. Returns a cleaned copy
    with values coerced to their declared types; raises ValueError."""
    clean = {}
    for name, (ptype, lo, hi, label, _unit, _help) in PARAM_SPEC.items():
        if name not in params:
            raise ValueError(f"missing parameter: {name}")
        try:
            value = ptype(params[name])
        except (TypeError, ValueError):
            raise ValueError(f"{label}: not a number")
        if not (lo <= value <= hi):
            raise ValueError(f"{label}: must be between {lo} and {hi}")
        clean[name] = value
    if clean["base_speed"] > clean["max_speed"]:
        raise ValueError("Base speed must not exceed max speed")
    if clean["hard_cap_temp"] <= clean["target_temp"]:
        raise ValueError("Hard cap temp must be greater than target temp")
    return clean


class SharedState:
    """Thread-safe bridge between the control loop and the web UI:
    live-tunable parameters, rolling sample history, and status.

    `baseline` holds the CLI-flag/default values; `settings_path` is the
    override file web-UI changes are persisted to. Reverting restores
    the baseline and removes the file."""

    def __init__(self, params, baseline, settings_path, aliases=None, demo=False):
        self._lock = threading.Lock()
        self._params = validate_params(params)
        self._baseline = validate_params(baseline)
        self.settings_path = settings_path
        self.aliases = dict(aliases or {})  # sensor_id (lowercase) -> display name
        self._history = deque(maxlen=HISTORY_MAX_SAMPLES)
        self._status = {
            "state": "starting",
            "consecutive_failures": 0,
            "last_error": None,
            "demo": demo,
        }

    def get_params(self):
        with self._lock:
            return dict(self._params)

    def _save_settings_locked(self):
        """Atomically write the current params to the settings file.
        Returns an error string, or None on success. Lock must be held."""
        payload = {"_note": "Written by the quidrac web UI. Overrides CLI "
                            "flags/defaults at startup; delete (or use "
                            "'Restore script settings') to revert."}
        payload.update(self._params)
        tmp = self.settings_path + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(payload, f, indent=2)
                f.write("\n")
            os.replace(tmp, self.settings_path)
            return None
        except OSError as e:
            return str(e)

    def update_params(self, updates):
        """Merge, validate, apply, and persist parameter updates. Returns
        (new_params, list of 'name: old -> new' change strings,
        save error string or None)."""
        with self._lock:
            merged = dict(self._params)
            for name in updates:
                if name not in PARAM_SPEC:
                    raise ValueError(f"unknown parameter: {name}")
            merged.update(updates)
            clean = validate_params(merged)
            changes = [f"{n}: {self._params[n]} -> {clean[n]}"
                       for n in PARAM_SPEC if clean[n] != self._params[n]]
            self._params = clean
            save_error = self._save_settings_locked() if changes else None
            return dict(clean), changes, save_error

    def revert_to_baseline(self):
        """Restore the CLI-flag/default params and remove the settings
        file. Returns (params, list of change strings, error or None)."""
        with self._lock:
            changes = [f"{n}: {self._params[n]} -> {self._baseline[n]}"
                       for n in PARAM_SPEC if self._baseline[n] != self._params[n]]
            self._params = dict(self._baseline)
            error = None
            try:
                os.remove(self.settings_path)
            except FileNotFoundError:
                pass
            except OSError as e:
                error = str(e)
            return dict(self._params), changes, error

    def add_sample(self, sample):
        with self._lock:
            self._history.append(sample)

    def set_status(self, **kwargs):
        with self._lock:
            self._status.update(kwargs)

    def snapshot(self, since=0.0):
        with self._lock:
            samples = [s for s in self._history if s["t"] > since]
            return {
                "now": time.time(),
                "params": dict(self._params),
                "status": dict(self._status),
                "samples": samples,
                "settings_file": os.path.basename(self.settings_path),
                "overrides_active": self._params != self._baseline,
                "aliases": dict(self.aliases),
            }


# ---------------------------------------------------------------------------
# IPMI backends
# ---------------------------------------------------------------------------

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


class DemoIpmi:
    """Simulated iDRAC for --demo: a first-order thermal model whose
    temperatures respond to the fan speed we set, with a slow load wave
    and periodic load spikes so the control loop has something to do."""

    def __init__(self):
        self.fan_speed = 30.0
        self._temps = {}
        self._tick = 0

    def set_manual_mode(self):
        pass

    def set_auto_mode(self):
        pass

    def set_fan_speed(self, percent):
        self.fan_speed = float(max(0, min(100, int(percent))))

    def read_temps(self, sensor_ids):
        if not self._temps:
            for i, sid in enumerate(sensor_ids):
                self._temps[sid.lower()] = 58.0 - 4.0 * i
        self._tick += 1
        for i, sid in enumerate(self._temps):
            load = 12 + 4 * math.sin(self._tick / 45 + i * 1.7) + random.uniform(-0.8, 0.8)
            if (self._tick // 180) % 4 == 2:
                load += 14  # periodic sustained load spike
            # Equilibrium temp falls as fan speed rises.
            equilibrium = 40 - 2 * i + load / (0.30 + self.fan_speed / 100.0)
            cur = self._temps[sid]
            self._temps[sid] = cur + (equilibrium - cur) * 0.10
        return {sid: int(round(v)) for sid, v in self._temps.items()}


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class CurveController:
    """
    Temperature->speed curve plus asymmetric slew limiting: speed rises
    instantly to the curve, but falls at most fall_rate percent per
    poll. The curve is driven by a hysteresis-filtered control
    temperature (rises instantly, falls only after a temp_hysteresis
    drop) so single-degree sensor flicker doesn't bounce the fans.

    Parameters are read live from SharedState each poll, so web-UI
    changes take effect immediately.
    """

    def __init__(self, state):
        self.state = state
        self.current_speed = float(state.get_params()["base_speed"])
        self.control_temp = None
        self.last_desired = self.current_speed

    @staticmethod
    def curve(temp, p):
        """Desired fan speed (%) for a given temperature under params p."""
        if temp <= p["target_temp"]:
            return float(p["base_speed"])
        if temp >= p["hard_cap_temp"]:
            return float(p["max_speed"])
        frac = (temp - p["target_temp"]) / (p["hard_cap_temp"] - p["target_temp"])
        return p["base_speed"] + frac * (p["max_speed"] - p["base_speed"])

    def evaluate(self, max_temp):
        """
        Given the hottest current sensor reading, return the fan speed to
        apply (integer percent). Rise instantly, fall slowly.
        """
        p = self.state.get_params()

        # Hysteresis: follow the raw temp up immediately, but only follow
        # it down once it has genuinely fallen, not on 1C sensor flicker.
        if self.control_temp is None or max_temp >= self.control_temp:
            self.control_temp = max_temp
        elif max_temp <= self.control_temp - p["temp_hysteresis"]:
            self.control_temp = max_temp

        # If max_speed was lowered at runtime, come back inside it.
        self.current_speed = min(self.current_speed, float(p["max_speed"]))

        desired = self.curve(self.control_temp, p)
        self.last_desired = desired
        if desired > self.current_speed:
            self.current_speed = desired
        else:
            self.current_speed = max(desired, self.current_speed - p["fall_rate"])
        return int(round(self.current_speed))


# ---------------------------------------------------------------------------
# Control loop
# ---------------------------------------------------------------------------

def fmt_readings(readings, aliases):
    return ", ".join(f"{aliases.get(sid, sid)}={v}C" for sid, v in readings.items())


def run_loop(ipmi, controller, state, sensor_ids):
    consecutive_failures = 0
    failsafe = False           # too many failures; trying to hand back to iDRAC
    failsafe_engaged = False   # the revert-to-auto command actually succeeded
    last_speed = None

    while True:
        p = state.get_params()
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
            p = state.get_params()  # evaluate() may see newer params; stay consistent

            if max_temp >= p["hard_cap_temp"]:
                log(f"Temp {max_temp}C at/above hard cap {p['hard_cap_temp']}C. "
                    f"Fans at {speed}%.", level="CRITICAL")

            # Re-assert manual mode and speed every poll: both commands are
            # cheap and idempotent, and this self-heals after an iDRAC
            # reset silently drops the override.
            ipmi.set_manual_mode()
            ipmi.set_fan_speed(speed)

            state.add_sample({
                "t": time.time(),
                "temps": readings,
                "max": max_temp,
                "ctrl": controller.control_temp,
                "speed": speed,
                "curve": round(controller.last_desired, 1),
                "target": p["target_temp"],
                "cap": p["hard_cap_temp"],
            })
            state.set_status(state="running", consecutive_failures=0, last_error=None)

            if speed != last_speed:
                log(f"Fan speed {'set' if last_speed is None else 'changed'} to {speed}% "
                    f"(curve target {controller.last_desired:.1f}% "
                    f"at control temp {controller.control_temp}C).")
                last_speed = speed

            log(f"Readings: {fmt_readings(readings, state.aliases)} | max={max_temp}C | "
                f"control={controller.control_temp}C | speed={speed}%")

        except Exception as e:
            consecutive_failures += 1
            log(f"Poll failed ({consecutive_failures}/{p['max_failed_polls']}): {e}",
                level="WARNING")
            state.set_status(state="failsafe" if failsafe else "degraded",
                             consecutive_failures=consecutive_failures,
                             last_error=str(e))

            if consecutive_failures >= p["max_failed_polls"] and not failsafe:
                log(f"{consecutive_failures} consecutive failed polls -- flying blind. "
                    f"Failsafe: reverting to automatic iDRAC fan control.", level="CRITICAL")
                failsafe = True
                last_speed = None  # force re-log once we recover

            if failsafe:
                state.set_status(state="failsafe")
                if not failsafe_engaged:
                    try:
                        ipmi.set_auto_mode()
                        failsafe_engaged = True
                        log("Failsafe engaged: iDRAC automatic fan control restored. "
                            "Will re-engage manual control when polling recovers.",
                            level="CRITICAL")
                    except Exception as e2:
                        log(f"Failsafe revert failed ({e2}); retrying next poll.",
                            level="CRITICAL")

        time.sleep(state.get_params()["poll_interval"])


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


# ---------------------------------------------------------------------------
# Web interface
# ---------------------------------------------------------------------------

def start_web_server(state, bind, port):
    class Handler(BaseHTTPRequestHandler):
        # Keep HTTP request noise out of the control log.
        def log_message(self, fmt, *args):
            pass

        def _send(self, code, body, content_type):
            data = body.encode("utf-8") if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _send_json(self, code, obj):
            self._send(code, json.dumps(obj), "application/json")

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == "/":
                self._send(200, HTML_PAGE, "text/html; charset=utf-8")
            elif url.path == "/api/state":
                qs = parse_qs(url.query)
                try:
                    since = float(qs.get("since", ["0"])[0])
                except ValueError:
                    since = 0.0
                snap = state.snapshot(since=since)
                snap["param_spec"] = {
                    name: {"type": ptype.__name__, "min": lo, "max": hi,
                           "label": label, "unit": unit, "help": help_}
                    for name, (ptype, lo, hi, label, unit, help_) in PARAM_SPEC.items()
                }
                self._send_json(200, snap)
            else:
                self._send_json(404, {"error": "not found"})

        def do_POST(self):
            url = urlparse(self.path)
            if url.path == "/api/params":
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    updates = json.loads(self.rfile.read(length) or b"{}")
                    if not isinstance(updates, dict):
                        raise ValueError("expected a JSON object")
                    new_params, changes, save_error = state.update_params(updates)
                except ValueError as e:
                    self._send_json(400, {"error": str(e)})
                    return
                except json.JSONDecodeError:
                    self._send_json(400, {"error": "invalid JSON"})
                    return
                if changes:
                    log("Params updated via web UI: " + "; ".join(changes)
                        + (f" (saved to {state.settings_path})" if not save_error else ""))
                if save_error:
                    log(f"Could not save settings to {state.settings_path}: {save_error}",
                        level="ERROR")
                self._send_json(200, {"ok": True, "params": new_params,
                                      "save_error": save_error})
            elif url.path == "/api/revert":
                new_params, changes, error = state.revert_to_baseline()
                if changes:
                    log("Reverted to script settings via web UI: " + "; ".join(changes))
                if error:
                    log(f"Could not remove settings file {state.settings_path}: {error}",
                        level="ERROR")
                else:
                    log(f"Settings file {state.settings_path} removed; "
                        f"script settings (CLI flags/defaults) in effect.")
                self._send_json(200, {"ok": True, "params": new_params,
                                      "save_error": error})
            else:
                self._send_json(404, {"error": "not found"})

    server = ThreadingHTTPServer((bind, port), Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True,
                              name="quidrac-web")
    thread.start()
    return server


# The dashboard. Self-contained: no external scripts, fonts, or styles,
# so it works on an offline management LAN.
HTML_PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,">
<title>quidrac — iDRAC fan control</title>
<style>
:root {
  --page:      #f9f9f7;
  --surface:   #fcfcfb;
  --ink:       #0b0b0b;
  --ink-2:     #52514e;
  --muted:     #898781;
  --grid:      #e1e0d9;
  --baseline:  #c3c2b7;
  --border:    rgba(11,11,11,0.10);
  --s1:        #2a78d6;   /* sensor 1 (blue) */
  --s2:        #1baf7a;   /* sensor 2 (aqua) */
  --s3:        #eda100;   /* sensor 3 (yellow) */
  --s4:        #008300;   /* sensor 4 (green) */
  --ctrl:      #4a3aa7;   /* control temp (violet) */
  --speed:     #eb6834;   /* fan speed (orange) */
  --good:      #0ca30c;
  --warning:   #fab219;
  --critical:  #d03b3b;
}
@media (prefers-color-scheme: dark) {
  :root {
    --page:      #0d0d0d;
    --surface:   #1a1a19;
    --ink:       #ffffff;
    --ink-2:     #c3c2b7;
    --muted:     #898781;
    --grid:      #2c2c2a;
    --baseline:  #383835;
    --border:    rgba(255,255,255,0.10);
    --s1:        #3987e5;
    --s2:        #199e70;
    --s3:        #c98500;
    --s4:        #008300;
    --ctrl:      #9085e9;
    --speed:     #d95926;
  }
}
* { box-sizing: border-box; margin: 0; }
body {
  background: var(--page); color: var(--ink);
  font: 14px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif;
  padding: 20px; max-width: 1060px; margin: 0 auto;
}
header { display: flex; align-items: baseline; gap: 12px; margin-bottom: 16px; flex-wrap: wrap; }
header h1 { font-size: 20px; font-weight: 650; }
header .sub { color: var(--ink-2); font-size: 13px; }
.chip {
  margin-left: auto; display: inline-flex; align-items: center; gap: 7px;
  font-size: 13px; font-weight: 600; color: var(--ink-2);
  border: 1px solid var(--border); border-radius: 999px; padding: 4px 12px;
  background: var(--surface);
}
.chip .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--muted); }
.tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 12px; margin-bottom: 16px; }
.tile {
  background: var(--surface); border: 1px solid var(--border); border-radius: 10px;
  padding: 12px 16px;
}
.tile .label { font-size: 12px; color: var(--ink-2); }
.tile .value { font-size: 30px; font-weight: 600; margin-top: 2px; }
.tile .value small { font-size: 15px; font-weight: 500; color: var(--ink-2); }
.tile .note { font-size: 12px; color: var(--muted); margin-top: 2px; min-height: 1.2em; }
.filterrow { display: flex; align-items: center; gap: 6px; margin: 4px 0 12px; flex-wrap: wrap; }
.filterrow .spacer { flex: 1; }
button, input[type=number] {
  font: inherit; color: var(--ink); background: var(--surface);
  border: 1px solid var(--border); border-radius: 7px;
}
button { padding: 5px 12px; cursor: pointer; }
button:hover { border-color: var(--baseline); }
button.on { border-color: var(--ink-2); font-weight: 650; }
.card {
  background: var(--surface); border: 1px solid var(--border); border-radius: 10px;
  padding: 16px; margin-bottom: 16px;
}
.card h2 { font-size: 14px; font-weight: 650; margin-bottom: 2px; }
.card .desc { font-size: 12px; color: var(--ink-2); margin-bottom: 10px; }
.legend { display: flex; gap: 16px; flex-wrap: wrap; font-size: 12px; color: var(--ink-2); margin-bottom: 8px; }
.legend .key { display: inline-flex; align-items: center; gap: 6px; }
.legend .line { width: 16px; height: 0; border-top: 2.5px solid; border-radius: 2px; }
.legend .line.dash { border-top-style: dashed; }
.chartwrap { position: relative; }
canvas { width: 100%; height: 220px; display: block; touch-action: none; }
.tooltip {
  position: absolute; pointer-events: none; display: none; z-index: 5;
  background: var(--surface); border: 1px solid var(--border); border-radius: 8px;
  box-shadow: 0 4px 14px rgba(0,0,0,0.18); padding: 8px 11px; font-size: 12px;
  min-width: 130px;
}
.tooltip .tt-time { color: var(--muted); margin-bottom: 5px; }
.tooltip .tt-row { display: flex; align-items: center; gap: 7px; margin-top: 2px; }
.tooltip .tt-key { width: 12px; height: 0; border-top: 2.5px solid; border-radius: 2px; flex: none; }
.tooltip .tt-val { font-weight: 650; }
.tooltip .tt-name { color: var(--ink-2); }
.params-grid {
  display: grid; grid-template-columns: repeat(auto-fill, minmax(215px, 1fr));
  gap: 12px 16px; margin-bottom: 12px;
}
.pfield label { display: block; font-size: 12px; color: var(--ink-2); margin-bottom: 3px; }
.pfield .inrow { display: flex; align-items: center; gap: 6px; }
.pfield input { width: 90px; padding: 5px 8px; }
.pfield .unit { font-size: 12px; color: var(--muted); }
.pfield .help { font-size: 11px; color: var(--muted); margin-top: 3px; }
.formrow { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
.formmsg { font-size: 13px; }
.formmsg.ok { color: var(--good); }
.formmsg.err { color: var(--critical); }
.formnote { font-size: 12px; color: var(--muted); margin-top: 8px; }
table { border-collapse: collapse; width: 100%; font-size: 12px; }
th, td { text-align: right; padding: 4px 10px; border-bottom: 1px solid var(--grid); font-variant-numeric: tabular-nums; }
th { color: var(--ink-2); font-weight: 600; position: sticky; top: 0; background: var(--surface); }
th:first-child, td:first-child { text-align: left; }
.tscroll { max-height: 320px; overflow-y: auto; }
</style>
</head>
<body>
<header>
  <h1>quidrac</h1>
  <span class="sub">iDRAC fan control</span>
  <span class="chip" id="chip"><span class="dot" id="chipdot"></span><span id="chiptext">connecting…</span></span>
</header>

<div class="tiles">
  <div class="tile"><div class="label">Fan speed</div><div class="value" id="tSpeed">–</div><div class="note" id="tSpeedNote"></div></div>
  <div class="tile"><div class="label">Hottest sensor</div><div class="value" id="tMax">–</div><div class="note" id="tMaxNote"></div></div>
  <div class="tile"><div class="label">Control temp</div><div class="value" id="tCtrl">–</div><div class="note">hysteresis-filtered</div></div>
  <div class="tile"><div class="label">Headroom to hard cap</div><div class="value" id="tHead">–</div><div class="note" id="tHeadNote"></div></div>
</div>

<div class="filterrow" id="ranges">
  <button data-win="900">15m</button>
  <button data-win="3600" class="on">1h</button>
  <button data-win="21600">6h</button>
  <button data-win="86400">24h</button>
  <span class="spacer"></span>
  <button id="tableBtn">Table</button>
</div>

<section class="card">
  <h2>Temperatures</h2>
  <div class="desc">&deg;C &middot; per-sensor readings, control temperature, and thresholds</div>
  <div class="legend" id="legendT"></div>
  <div class="chartwrap"><canvas id="chartT"></canvas><div class="tooltip" id="tipT"></div></div>
</section>

<section class="card">
  <h2>Fan speed</h2>
  <div class="desc">% &middot; applied speed vs. what the curve asks for</div>
  <div class="legend" id="legendS"></div>
  <div class="chartwrap"><canvas id="chartS"></canvas><div class="tooltip" id="tipS"></div></div>
</section>

<section class="card">
  <h2>Parameters</h2>
  <div class="desc" id="pdesc">Applied on the next poll.</div>
  <div class="params-grid" id="pgrid"></div>
  <div class="formrow">
    <button id="applyBtn">Apply</button>
    <button id="revertBtn">Restore script settings</button>
    <span class="formmsg" id="formmsg"></span>
  </div>
</section>

<section class="card" id="tableCard" style="display:none">
  <h2>Recent samples</h2>
  <div class="desc">Newest first &middot; the table view of both charts</div>
  <div class="tscroll"><table id="stable"><thead></thead><tbody></tbody></table></div>
</section>

<script>
"use strict";
const S = {
  samples: [], params: null, spec: null, status: null, aliases: {},
  lastT: 0, now: 0, win: 3600, hoverT: null, tableOn: false,
  sensorIds: [], connected: false, formDirty: false, formBuilt: false,
};
const aliasOf = sid => S.aliases[sid] || sid;
const MAXWIN = 86400;
const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const SENSOR_VARS = ["--s1", "--s2", "--s3", "--s4"];
const $ = id => document.getElementById(id);

function sensorColor(i) { return css(SENSOR_VARS[Math.min(i, SENSOR_VARS.length - 1)]); }

async function poll() {
  try {
    const since = S.lastT || (Date.now() / 1000 - MAXWIN);
    const r = await fetch("/api/state?since=" + since);
    if (!r.ok) throw new Error("HTTP " + r.status);
    const j = await r.json();
    S.params = j.params; S.spec = j.param_spec; S.status = j.status; S.now = j.now;
    S.settingsFile = j.settings_file; S.overridesActive = j.overrides_active;
    S.aliases = j.aliases || {};
    if (j.samples.length) {
      S.samples.push(...j.samples);
      S.lastT = j.samples[j.samples.length - 1].t;
      const cut = j.now - MAXWIN;
      let i = 0;
      while (i < S.samples.length && S.samples[i].t < cut) i++;
      if (i) S.samples.splice(0, i);
      S.sensorIds = Object.keys(S.samples[S.samples.length - 1].temps);
    }
    S.connected = true;
    if (!S.formBuilt && S.spec) buildForm();
    render();
  } catch (e) {
    S.connected = false;
    setChip(css("--muted"), "disconnected");
  }
  setTimeout(poll, 2000);
}

function setChip(color, text) {
  $("chipdot").style.background = color;
  $("chiptext").textContent = text;
}

function fmtTemp(v) { return v == null ? "–" : v + "°C"; }

function render() {
  const st = S.status || {};
  if (st.state === "running") setChip(css("--good"), st.demo ? "running (demo)" : "running");
  else if (st.state === "degraded") setChip(css("--warning"), "degraded — " + st.consecutive_failures + " failed poll" + (st.consecutive_failures === 1 ? "" : "s"));
  else if (st.state === "failsafe") setChip(css("--critical"), "FAILSAFE — auto control");
  else setChip(css("--muted"), st.state || "starting");

  const last = S.samples[S.samples.length - 1];
  if (last) {
    $("tSpeed").textContent = last.speed + "%";
    $("tSpeedNote").textContent = "curve asks " + last.curve + "%";
    const hot = S.sensorIds.reduce((a, b) => (last.temps[a] >= last.temps[b] ? a : b));
    $("tMax").textContent = fmtTemp(last.max);
    $("tMaxNote").textContent = aliasOf(hot);
    $("tCtrl").textContent = fmtTemp(last.ctrl);
    const head = last.cap - last.max;
    $("tHead").textContent = head + "°C";
    $("tHeadNote").textContent = "hard cap at " + last.cap + "°C";
  }
  if (S.settingsFile) {
    $("pdesc").textContent = "Applied on the next poll and saved to " + S.settingsFile +
      (S.overridesActive ? " (overriding script settings). " : ". ") +
      "“Restore script settings” returns to CLI flags/defaults and removes the file.";
  }
  drawTemps(); drawSpeed();
  if (S.tableOn) renderTable();
}

/* ---------- charts ---------- */

const PADL = 44, PADR = 14, PADT = 12, PADB = 26;

function setupCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h };
}

function visible() {
  const t1 = S.now || Date.now() / 1000, t0 = t1 - S.win;
  return { rows: S.samples.filter(s => s.t >= t0 && s.t <= t1), t0, t1 };
}

function niceTicks(lo, hi, n) {
  const span = hi - lo || 1;
  const step0 = span / n;
  const mag = Math.pow(10, Math.floor(Math.log10(step0)));
  let step = mag;
  for (const m of [1, 2, 2.5, 5, 10]) if (m * mag >= step0) { step = m * mag; break; }
  const ticks = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) ticks.push(+v.toFixed(6));
  return ticks;
}

function timeLabel(t) {
  const d = new Date(t * 1000);
  const hm = d.getHours().toString().padStart(2, "0") + ":" + d.getMinutes().toString().padStart(2, "0");
  return S.win >= 86400 ? (d.getMonth() + 1) + "/" + d.getDate() + " " + hm : hm;
}

function gapThreshold() {
  const pi = S.params ? S.params.poll_interval : 10;
  return Math.max(3 * pi, 15);
}

// series: {color, width, dash, get(row) -> value|null, area}
function drawChart(canvas, seriesList, yLo, yHi, unit) {
  const { ctx, w, h } = setupCanvas(canvas);
  const { rows, t0, t1 } = visible();
  const X = t => PADL + (t - t0) / (t1 - t0) * (w - PADL - PADR);
  const Y = v => PADT + (yHi - v) / (yHi - yLo) * (h - PADT - PADB);

  // grid + y ticks
  ctx.font = "11px system-ui, sans-serif";
  ctx.fillStyle = css("--muted");
  ctx.strokeStyle = css("--grid");
  ctx.lineWidth = 1;
  for (const v of niceTicks(yLo, yHi, 4)) {
    const y = Math.round(Y(v)) + 0.5;
    ctx.beginPath(); ctx.moveTo(PADL, y); ctx.lineTo(w - PADR, y); ctx.stroke();
    ctx.textAlign = "right"; ctx.textBaseline = "middle";
    ctx.fillText(v + unit, PADL - 7, y);
  }
  // x ticks
  const nx = Math.max(3, Math.floor(w / 170));
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  for (let i = 0; i <= nx; i++) {
    const t = t0 + (t1 - t0) * i / nx;
    ctx.fillText(timeLabel(t), X(t), h - PADB + 8);
  }
  // baseline
  ctx.strokeStyle = css("--baseline");
  ctx.beginPath();
  ctx.moveTo(PADL, Math.round(h - PADB) + 0.5); ctx.lineTo(w - PADR, Math.round(h - PADB) + 0.5);
  ctx.stroke();

  if (!rows.length) {
    ctx.fillStyle = css("--muted"); ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText("waiting for samples…", (PADL + w - PADR) / 2, h / 2);
    return { rows, X, Y };
  }

  const gap = gapThreshold();
  for (const s of seriesList) {
    ctx.strokeStyle = s.color; ctx.lineWidth = s.width || 2;
    ctx.lineJoin = "round"; ctx.lineCap = "round";
    ctx.setLineDash(s.dash || []);
    // area wash first
    if (s.area) {
      ctx.globalAlpha = 0.1;
      let started = false, startX = 0, prevT = null, prevX = 0;
      ctx.beginPath();
      for (const r of rows) {
        const v = s.get(r); if (v == null) continue;
        const x = X(r.t), y = Y(v);
        if (!started || (prevT != null && r.t - prevT > gap)) {
          if (started) { ctx.lineTo(prevX, Y(yLo)); ctx.lineTo(startX, Y(yLo)); ctx.closePath(); ctx.fillStyle = s.color; ctx.fill(); ctx.beginPath(); }
          ctx.moveTo(x, y); started = true; startX = x;
        } else ctx.lineTo(x, y);
        prevT = r.t; prevX = x;
      }
      if (started) { ctx.lineTo(prevX, Y(yLo)); ctx.lineTo(startX, Y(yLo)); ctx.closePath(); ctx.fillStyle = s.color; ctx.fill(); }
      ctx.globalAlpha = 1;
    }
    let prevT = null;
    ctx.beginPath();
    for (const r of rows) {
      const v = s.get(r); if (v == null) { prevT = null; continue; }
      const x = X(r.t), y = Y(v);
      if (prevT == null || r.t - prevT > gap) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      prevT = r.t;
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // end markers + selective direct label (endpoint only) for main series
  for (const s of seriesList) {
    if (!s.marker) continue;
    for (let i = rows.length - 1; i >= 0; i--) {
      const v = s.get(rows[i]);
      if (v == null) continue;
      const x = X(rows[i].t), y = Y(v);
      ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fillStyle = s.color; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = css("--surface"); ctx.stroke();
      break;
    }
  }

  // crosshair
  if (S.hoverT != null && S.hoverT >= t0 && S.hoverT <= t1) {
    const x = Math.round(X(S.hoverT)) + 0.5;
    ctx.strokeStyle = css("--baseline"); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, PADT); ctx.lineTo(x, h - PADB); ctx.stroke();
  }
  return { rows, X, Y };
}

function tempSeries() {
  const list = S.sensorIds.map((sid, i) => ({
    name: aliasOf(sid), color: sensorColor(i), width: 2, marker: true, get: r => r.temps[sid] ?? null,
  }));
  list.push({ name: "control temp", color: css("--ctrl"), width: 2, dash: [2, 4], marker: false, get: r => r.ctrl });
  list.push({ name: "target", color: css("--muted"), width: 1.5, dash: [5, 4], get: r => r.target });
  list.push({ name: "hard cap", color: css("--critical"), width: 1.5, dash: [5, 4], get: r => r.cap });
  return list;
}

function speedSeries() {
  return [
    { name: "curve target", color: css("--muted"), width: 1.5, dash: [5, 4], get: r => r.curve },
    { name: "fan speed", color: css("--speed"), width: 2, marker: true, area: true, get: r => r.speed },
  ];
}

let geomT = null, geomS = null;

function drawTemps() {
  const series = tempSeries();
  const { rows } = visible();
  let lo = Infinity, hi = -Infinity;
  for (const r of rows) {
    for (const sid of S.sensorIds) { const v = r.temps[sid]; if (v != null) { lo = Math.min(lo, v); hi = Math.max(hi, v); } }
    lo = Math.min(lo, r.target); hi = Math.max(hi, r.cap);
  }
  if (!isFinite(lo)) { lo = 30; hi = 90; }
  const pad = Math.max(2, (hi - lo) * 0.08);
  geomT = drawChart($("chartT"), series, Math.floor(lo - pad), Math.ceil(hi + pad), "°");
  renderLegend($("legendT"), series);
}

function drawSpeed() {
  const series = speedSeries();
  geomS = drawChart($("chartS"), series, 0, 100, "%");
  renderLegend($("legendS"), series);
}

function renderLegend(el, series) {
  if (el.childElementCount === series.length) return;
  el.replaceChildren();
  for (const s of series) {
    const key = document.createElement("span"); key.className = "key";
    const line = document.createElement("span");
    line.className = "line" + (s.dash ? " dash" : "");
    line.style.borderTopColor = s.color;
    key.append(line, document.createTextNode(s.name));
    el.append(key);
  }
}

/* ---------- tooltip ---------- */

function nearestRow(t) {
  const { rows } = visible();
  let best = null, bd = Infinity;
  for (const r of rows) { const d = Math.abs(r.t - t); if (d < bd) { bd = d; best = r; } }
  return best;
}

function attachHover(canvas, tip, seriesFn, unit) {
  const wrap = canvas.parentElement;
  canvas.addEventListener("pointermove", ev => {
    const rect = canvas.getBoundingClientRect();
    const { t0, t1 } = visible();
    const frac = (ev.clientX - rect.left - PADL) / (rect.width - PADL - PADR);
    const t = t0 + Math.max(0, Math.min(1, frac)) * (t1 - t0);
    const row = nearestRow(t);
    if (!row) { S.hoverT = null; tip.style.display = "none"; render(); return; }
    S.hoverT = row.t;
    tip.replaceChildren();
    const tt = document.createElement("div"); tt.className = "tt-time";
    tt.textContent = new Date(row.t * 1000).toLocaleTimeString();
    tip.append(tt);
    for (const s of seriesFn()) {
      const v = s.get(row);
      if (v == null) continue;
      const rowEl = document.createElement("div"); rowEl.className = "tt-row";
      const k = document.createElement("span"); k.className = "tt-key";
      k.style.borderTopColor = s.color;
      if (s.dash) k.style.borderTopStyle = "dashed";
      const val = document.createElement("span"); val.className = "tt-val";
      val.textContent = (Math.round(v * 10) / 10) + unit;
      const nm = document.createElement("span"); nm.className = "tt-name";
      nm.textContent = s.name;
      rowEl.append(k, val, nm);
      tip.append(rowEl);
    }
    tip.style.display = "block";
    const geom = canvas === $("chartT") ? geomT : geomS;
    const x = geom ? geom.X(row.t) : ev.clientX - rect.left;
    const flip = x > rect.width - 170;
    tip.style.left = flip ? "" : (x + 12) + "px";
    tip.style.right = flip ? (rect.width - x + 12) + "px" : "";
    tip.style.top = "14px";
    render();
  });
  canvas.addEventListener("pointerleave", () => {
    S.hoverT = null; tip.style.display = "none"; render();
  });
}

/* ---------- table view ---------- */

function renderTable() {
  const thead = $("stable").querySelector("thead");
  const tbody = $("stable").querySelector("tbody");
  const tr = document.createElement("tr");
  for (const htext of ["Time", ...S.sensorIds.map(s => aliasOf(s) + " °C"), "Control °C", "Speed %", "Curve %", "Target °C", "Cap °C"]) {
    const th = document.createElement("th"); th.textContent = htext; tr.append(th);
  }
  thead.replaceChildren(tr);
  tbody.replaceChildren();
  const { rows } = visible();
  for (const r of rows.slice(-200).reverse()) {
    const tre = document.createElement("tr");
    const cells = [new Date(r.t * 1000).toLocaleTimeString(),
                   ...S.sensorIds.map(s => r.temps[s] ?? ""),
                   r.ctrl, r.speed, r.curve, r.target, r.cap];
    for (const c of cells) { const td = document.createElement("td"); td.textContent = c; tre.append(td); }
    tbody.append(tre);
  }
}

/* ---------- params form ---------- */

function buildForm() {
  const grid = $("pgrid");
  grid.replaceChildren();
  for (const [name, spec] of Object.entries(S.spec)) {
    const field = document.createElement("div"); field.className = "pfield";
    const label = document.createElement("label");
    label.textContent = spec.label; label.htmlFor = "p_" + name;
    const inrow = document.createElement("div"); inrow.className = "inrow";
    const input = document.createElement("input");
    input.type = "number"; input.id = "p_" + name; input.name = name;
    input.min = spec.min; input.max = spec.max;
    input.step = spec.type === "int" ? 1 : 0.5;
    input.value = S.params[name];
    input.addEventListener("input", () => { S.formDirty = true; });
    const unit = document.createElement("span"); unit.className = "unit"; unit.textContent = spec.unit;
    inrow.append(input, unit);
    const help = document.createElement("div"); help.className = "help"; help.textContent = spec.help;
    field.append(label, inrow, help);
    grid.append(field);
  }
  S.formBuilt = true;
}

function fillForm() {
  for (const name of Object.keys(S.spec)) $("p_" + name).value = S.params[name];
  S.formDirty = false;
}

async function applyParams() {
  const updates = {};
  for (const [name, spec] of Object.entries(S.spec)) {
    const raw = $("p_" + name).value;
    updates[name] = spec.type === "int" ? parseInt(raw, 10) : parseFloat(raw);
  }
  const msg = $("formmsg");
  try {
    const r = await fetch("/api/params", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(updates),
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || "HTTP " + r.status);
    S.params = j.params; S.formDirty = false;
    if (j.save_error) {
      msg.className = "formmsg err";
      msg.textContent = "Applied, but could not save: " + j.save_error;
    } else {
      msg.className = "formmsg ok"; msg.textContent = "Applied & saved.";
    }
  } catch (e) {
    msg.className = "formmsg err"; msg.textContent = String(e.message || e);
  }
  setTimeout(() => { if (msg.className === "formmsg ok") msg.textContent = ""; }, 4000);
}

async function revertParams() {
  const msg = $("formmsg");
  try {
    const r = await fetch("/api/revert", { method: "POST" });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || "HTTP " + r.status);
    S.params = j.params;
    fillForm();
    if (j.save_error) {
      msg.className = "formmsg err";
      msg.textContent = "Reverted, but could not remove settings file: " + j.save_error;
    } else {
      msg.className = "formmsg ok"; msg.textContent = "Script settings restored.";
    }
  } catch (e) {
    msg.className = "formmsg err"; msg.textContent = String(e.message || e);
  }
  setTimeout(() => { if (msg.className === "formmsg ok") msg.textContent = ""; }, 4000);
}

/* ---------- wiring ---------- */

$("ranges").addEventListener("click", ev => {
  const btn = ev.target.closest("button[data-win]");
  if (!btn) return;
  S.win = +btn.dataset.win;
  for (const b of $("ranges").querySelectorAll("button[data-win]")) b.classList.toggle("on", b === btn);
  render();
});
$("tableBtn").addEventListener("click", () => {
  S.tableOn = !S.tableOn;
  $("tableBtn").classList.toggle("on", S.tableOn);
  $("tableCard").style.display = S.tableOn ? "" : "none";
  if (S.tableOn) renderTable();
});
$("applyBtn").addEventListener("click", applyParams);
$("revertBtn").addEventListener("click", revertParams);
attachHover($("chartT"), $("tipT"), tempSeries, "°C");
attachHover($("chartS"), $("tipS"), speedSeries, "%");
window.addEventListener("resize", render);
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", render);
poll();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Custom software fan control loop for iDRAC "
                    "(temperature->speed curve with asymmetric slew) "
                    "with a built-in web dashboard.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--user", default=DEFAULT_USER)
    parser.add_argument("--password", default=DEFAULT_PASSWORD,
                        help="iDRAC password. Prefer the IPMI_PASSWORD environment "
                             "variable, which keeps it out of process listings.")
    parser.add_argument("--demo", action="store_true",
                        help="Run against a simulated iDRAC (no credentials needed); "
                             "useful for previewing the web UI and tuning parameters")

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

    parser.add_argument("--settings-file", default=None,
                        help=f"Path where web-UI parameter changes are saved and, if "
                             f"present at startup, override CLI flags/defaults "
                             f"(default: {SETTINGS_FILENAME} next to this script)")
    parser.add_argument("--web-bind", default=DEFAULT_WEB_BIND,
                        help=f"Web UI bind address (default: {DEFAULT_WEB_BIND}; the UI "
                             "has no auth, so bind to a trusted network only)")
    parser.add_argument("--web-port", type=int, default=DEFAULT_WEB_PORT,
                        help=f"Web UI port (default: {DEFAULT_WEB_PORT})")
    parser.add_argument("--no-web", dest="web", action="store_false", default=True,
                        help="Disable the web UI entirely")

    args = parser.parse_args()

    if args.demo:
        ipmi = DemoIpmi()
    else:
        if not args.host or not args.user:
            parser.error("--host and --user are required (or use --demo)")
        password = args.password or os.environ.get("IPMI_PASSWORD")
        if not password:
            parser.error("no password: pass --password or set the IPMI_PASSWORD "
                         "environment variable (preferred)")
        ipmi = IdracIpmi(args.host, args.user, password)

    sensor_ids = [s.strip().lower() for s in args.sensors.split(",") if s.strip()]
    settings_path = args.settings_file or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), SETTINGS_FILENAME)
    cli_params = {
        "base_speed": args.base_speed,
        "max_speed": args.max_speed,
        "target_temp": args.target_temp,
        "hard_cap_temp": args.hard_cap_temp,
        "fall_rate": args.fall_rate,
        "temp_hysteresis": args.temp_hysteresis,
        "poll_interval": args.poll_interval,
        "max_failed_polls": args.max_failed_polls,
    }
    try:
        cli_params = validate_params(cli_params)
    except ValueError as e:
        parser.error(str(e))
    params = load_settings_file(settings_path, cli_params)
    aliases = {k.lower(): v for k, v in SENSOR_ALIASES.items()}
    state = SharedState(params, baseline=cli_params, settings_path=settings_path,
                        aliases=aliases, demo=args.demo)
    controller = CurveController(state)

    def handle_shutdown(signum, frame):
        log(f"Received signal {signum}, shutting down.")
        sys.exit(0)  # unwinds through the finally block below

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    p = state.get_params()
    log(f"Starting{' in DEMO mode' if args.demo else ''}. Monitoring sensors {sensor_ids}. "
        f"Curve: {p['base_speed']}% at <={p['target_temp']}C -> {p['max_speed']}% at "
        f">={p['hard_cap_temp']}C, fall rate {p['fall_rate']}%/poll, hysteresis "
        f"{p['temp_hysteresis']}C, poll every {p['poll_interval']}s, failsafe after "
        f"{p['max_failed_polls']} failed polls.")

    if args.web:
        try:
            start_web_server(state, args.web_bind, args.web_port)
            log(f"Web UI listening on http://{args.web_bind}:{args.web_port}/ "
                f"(no auth -- trusted networks only).")
        except OSError as e:
            log(f"Web UI failed to start on {args.web_bind}:{args.web_port}: {e}. "
                f"Continuing without it.", level="ERROR")

    engaged = False
    try:
        ipmi.set_manual_mode()
        engaged = True
        ipmi.set_fan_speed(int(controller.current_speed))
        log(f"Manual mode enabled. Initial fan speed set to {int(controller.current_speed)}%.")
        run_loop(ipmi, controller, state, sensor_ids)
    finally:
        # Don't let a second Ctrl-C interrupt the revert.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if engaged:
            cleanup(ipmi, args.revert_on_exit, args.exit_speed)


if __name__ == "__main__":
    main()
