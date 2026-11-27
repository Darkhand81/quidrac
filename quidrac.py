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

    ExecStopPost=/usr/local/bin/quidrac.py --host <host> --user <user> \\
        --revert-to-auto

STRATEGY (temperature -> speed curve with asymmetric slew):
  - Fan speed is a pure function of the hottest monitored sensor:

        temp <= --target-temp     ->  --base-speed
        temp >= --hard-cap-temp   ->  --max-speed
        in between                ->  linear interpolation

  - Rising temps act fast but not abruptly: a rise is spread evenly
    across --rise-splits polls (default 2), so a decision to raise the
    fans by 10% applies 5% now and 5% next poll. If the curve asks for
    more mid-rise, the step is recomputed from the remaining gap.
    EXCEPTION: at/above the hard cap the rise is applied instantly --
    no smoothing during an emergency.
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
  - Cleanup (revert to auto, hold --exit-speed, or leave the fans at
    their last applied speed with --exit-speed current) runs in a finally
    block, so it happens on SIGINT/SIGTERM *and* on crashes, not just
    clean signals.

WEB INTERFACE:
  A built-in dashboard (stdlib http.server, zero dependencies, works
  offline) serves live charts of temperatures, control temperature,
  target/hard-cap thresholds, and fan speed, plus a form to tweak the
  control parameters at runtime. Parameter changes apply on the next
  poll and are saved to a settings file (default: quidrac-settings.json
  next to this script; override with --settings-file). At startup,
  settings resolve lowest-to-highest: script defaults -> settings file
  -> CLI flags you explicitly pass (an explicit flag is an explicit
  request, so it outranks the saved file). "Restore script settings" in
  the UI returns to the launch configuration (defaults + explicit
  flags) and removes the file. There is no authentication: bind it to a
  trusted network only
  (--web-bind, default 0.0.0.0; --web-port, default 8080; --no-web to
  disable).

DEMO MODE:
  --demo runs the full control loop and web UI against a simulated
  server (a simple thermal model with periodic load spikes) instead of
  a real iDRAC -- useful for previewing the dashboard and tuning
  parameters without hardware. No credentials required.

IPMI TRANSPORT:
  The default --ipmi-backend native is a built-in IPMI 2.0 / RMCP+
  client (the protocol `ipmitool -I lanplus` speaks, AES-encrypted and
  HMAC-authenticated) that logs in ONCE and keeps that session for every
  poll, instead of a full login handshake per command. If the session
  dies (iDRAC reset, network drop) the next command times out, logs in
  again and retries; a session idle longer than 30s is replaced rather
  than trusted. --ipmi-backend ipmitool restores the old behavior of
  running ipmitool for each command. --list-sensors prints the iDRAC's
  temperature sensors, a handy first connection test.

CREDENTIALS:
  Prefer setting the IPMI_PASSWORD environment variable over passing
  --password on the command line (argv is visible to every local user
  via ps). With --ipmi-backend ipmitool, the password is handed to
  ipmitool through its environment (-E), never on its command line.

Requires: IPMI-over-LAN enabled in iDRAC. Python 3 standard library
only; ipmitool is needed only for --ipmi-backend ipmitool.
"""

import argparse
import hashlib
import hmac
import json
import math
import os
import random
import re
import signal
import socket
import struct
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
DEFAULT_PORT = 623                 # IPMI-over-LAN (RMCP+) UDP port
DEFAULT_IPMI_BACKEND = "native"    # "native": built-in client, one persistent session.
                                   # "ipmitool": shell out per command (a fresh login each time).

DEFAULT_SENSORS = "0Eh,0Fh"        # Sensor IDs to monitor (see --list-sensors)
SENSOR_ALIASES = {                 # Friendly names shown in the dashboard and logs.
    "0Eh": "CPU1",                 # Edit here or pass --sensor-aliases "0Eh=CPU1,..."
    "0Fh": "CPU2",                 # (flag entries win). Unlisted sensors show their raw ID.
}
DEFAULT_POLL_INTERVAL = 10         # Seconds between polls

DEFAULT_BASE_SPEED = 30            # Minimum fan speed (%), used at/below target temp
DEFAULT_TARGET_TEMP = 65           # Curve start: temps at/below this run at base speed (C)
DEFAULT_HARD_CAP_TEMP = 80         # Curve end: temps at/above this run at max speed (C)
DEFAULT_MAX_SPEED = 100            # Maximum fan speed (%)
DEFAULT_FALL_RATE = 2.0            # Max fan speed decrease per poll when cooling (%/poll)
DEFAULT_RISE_SPLITS = 2            # Polls to spread each fan speed rise across (1 = instant)
DEFAULT_TEMP_HYSTERESIS = 2.0      # Temp must fall this far below its recent peak before fans follow (C)

DEFAULT_MAX_FAILED_POLLS = 5       # Consecutive failed polls before failsafe revert to auto
DEFAULT_REVERT_ON_EXIT = True      # Revert to automatic iDRAC fan control on exit
CURRENT_SPEED = "current"          # Sentinel: leave fans at their last applied speed
DEFAULT_EXIT_SPEED = DEFAULT_BASE_SPEED  # If REVERT_ON_EXIT is False, hold this speed (%) on
                                         # exit -- or set to CURRENT_SPEED to leave the fans
                                         # exactly where the loop last put them

DEFAULT_WEB_BIND = "0.0.0.0"       # Web UI bind address ("127.0.0.1" for local-only)
DEFAULT_WEB_PORT = 8080            # Web UI port
HISTORY_MAX_SAMPLES = 20000        # Rolling history kept for the charts (~55h at 10s polls)
SETTINGS_FILENAME = "quidrac-settings.json"  # Web UI overrides, saved next to this script
# ---------------------------------------------------------------------------

HOSTNAME = socket.gethostname()    # Shown in the dashboard header


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
    "target_temp":      (float, 20,  105, "Target temp",     "\u00b0C", "Curve start: at/below this temp fans run at base speed"),
    "hard_cap_temp":    (float, 25,  110, "Hard cap temp",   "\u00b0C", "Curve end: at/above this temp fans jump to max speed"),
    "fall_rate":        (float, 0.1, 100, "Fall rate",       "%/poll", "Max fan speed decrease per poll when cooling"),
    "rise_splits":      (int,   1,   20,  "Rise splits",     "polls",  "Spread each fan speed rise across this many polls (1 = instant; hard-cap rises are always instant)"),
    "temp_hysteresis":  (float, 0,   20,  "Temp hysteresis", "\u00b0C", "Temp must fall this far below its recent peak before fans follow"),
    "poll_interval":    (int,   1,   300, "Poll interval",   "s",      "Seconds between sensor polls"),
}
# max_failed_polls is intentionally NOT here: it's a config/flag-only safety
# setting (DEFAULT_MAX_FAILED_POLLS / --max-failed-polls), kept out of the live
# web form and the settings file.


def read_settings_overrides(path):
    """Read saved web-UI parameter overrides from `path`. Returns a dict
    holding PARAM_SPEC keys only ({} if the file is absent); a malformed
    or unreadable file logs a warning and is ignored."""
    if not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError("not a JSON object")
        return {k: v for k, v in data.items() if k in PARAM_SPEC}
    except (OSError, ValueError, json.JSONDecodeError) as e:
        log(f"Ignoring settings file {path}: {e}.", level="WARNING")
        return {}


def exit_speed_type(value):
    """argparse type for --exit-speed: a 0-100 percentage, or 'current'
    to leave the fans at whatever speed the loop last applied."""
    if str(value).strip().lower() == CURRENT_SPEED:
        return CURRENT_SPEED
    try:
        speed = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError(f"expected a percentage or '{CURRENT_SPEED}', got {value!r}")
    if not 0 <= speed <= 100:
        raise argparse.ArgumentTypeError(f"must be 0-100 or '{CURRENT_SPEED}'")
    return speed


def parse_sensor_aliases(text):
    """Parse a --sensor-aliases value like '0Eh=CPU1,0Fh=CPU2' into
    {sensor_id_lowercase: display_name}. Raises ValueError on bad entries."""
    out = {}
    for entry in text.split(","):
        entry = entry.strip()
        if not entry:
            continue
        sid, sep, name = entry.partition("=")
        if not sep or not sid.strip() or not name.strip():
            raise ValueError(f"bad --sensor-aliases entry {entry!r}; expected ID=NAME")
        out[sid.strip().lower()] = name.strip()
    return out


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

# ---------------------------------------------------------------------------
# Native IPMI 2.0 / RMCP+ client (standard library only)
#
# Speaks the same wire protocol as `ipmitool -I lanplus`, but holds ONE
# authenticated session open across polls instead of logging in afresh
# for every command. Implements what this script needs and no more:
# RAKP session setup with cipher suite 3 (HMAC-SHA1 / HMAC-SHA1-96 /
# AES-CBC-128) or 17 (HMAC-SHA256 / HMAC-SHA256-128 / AES-CBC-128),
# arbitrary request/response commands, SDR repository reads, and sensor
# reading conversion. References: IPMI v2.0 rev 1.1 spec, sections 13
# (LAN interface / RMCP+), 33 (SDR repository) and 35-36 (sensors).
# ---------------------------------------------------------------------------

def _aes_build_tables():
    def xtime(a):
        a <<= 1
        return (a ^ 0x11B) if a & 0x100 else a

    def gmul(a, b):
        r = 0
        while b:
            if b & 1:
                r ^= a
            a = xtime(a)
            b >>= 1
        return r

    sbox = [0] * 256
    for x in range(256):
        inv = 0
        if x:
            inv, base, e = 1, x, 254          # x^254 == x^-1 in GF(2^8)
            while e:
                if e & 1:
                    inv = gmul(inv, base)
                base = gmul(base, base)
                e >>= 1
        s = inv
        for _ in range(4):
            inv = ((inv << 1) | (inv >> 7)) & 0xFF
            s ^= inv
        sbox[x] = s ^ 0x63
    inv_sbox = [0] * 256
    for i, v in enumerate(sbox):
        inv_sbox[v] = i
    mul = {k: [gmul(i, k) for i in range(256)] for k in (2, 3, 9, 11, 13, 14)}
    return sbox, inv_sbox, mul


_AES_SBOX, _AES_INV_SBOX, _AES_MUL = _aes_build_tables()


class _Aes128:
    """Minimal pure-Python AES-128 block cipher (FIPS-197). Slow by crypto
    standards, but RMCP+ packets are a few blocks each, a handful of times
    per poll -- microseconds that don't matter. Exists only so the native
    backend needs nothing outside the standard library."""

    def __init__(self, key):
        if len(key) != 16:
            raise ValueError("AES-128 key must be 16 bytes")
        w = list(key)
        rcon = 1
        while len(w) < 176:
            t = w[-4:]
            if len(w) % 16 == 0:
                t = [_AES_SBOX[t[1]] ^ rcon, _AES_SBOX[t[2]], _AES_SBOX[t[3]], _AES_SBOX[t[0]]]
                rcon = ((rcon << 1) ^ 0x11B) if rcon & 0x80 else rcon << 1
            w.extend([w[-16 + i] ^ t[i] for i in range(4)])
        self._rk = [w[16 * r:16 * r + 16] for r in range(11)]

    def encrypt_block(self, block):
        sb, m2, m3 = _AES_SBOX, _AES_MUL[2], _AES_MUL[3]
        s = [b ^ k for b, k in zip(block, self._rk[0])]
        for rnd in range(1, 11):
            s = [sb[b] for b in s]
            # ShiftRows (state is column-major: s[row + 4*col])
            s = [s[(i + 4 * (i % 4)) % 16] for i in range(16)]
            if rnd != 10:
                t = []
                for c in range(0, 16, 4):
                    a0, a1, a2, a3 = s[c:c + 4]
                    t += [m2[a0] ^ m3[a1] ^ a2 ^ a3, a0 ^ m2[a1] ^ m3[a2] ^ a3,
                          a0 ^ a1 ^ m2[a2] ^ m3[a3], m3[a0] ^ a1 ^ a2 ^ m2[a3]]
                s = t
            s = [b ^ k for b, k in zip(s, self._rk[rnd])]
        return bytes(s)

    def decrypt_block(self, block):
        isb = _AES_INV_SBOX
        m9, m11, m13, m14 = _AES_MUL[9], _AES_MUL[11], _AES_MUL[13], _AES_MUL[14]
        s = [b ^ k for b, k in zip(block, self._rk[10])]
        for rnd in range(9, -1, -1):
            s = [s[(i - 4 * (i % 4)) % 16] for i in range(16)]   # InvShiftRows
            s = [isb[b] for b in s]
            s = [b ^ k for b, k in zip(s, self._rk[rnd])]
            if rnd != 0:
                t = []
                for c in range(0, 16, 4):
                    a0, a1, a2, a3 = s[c:c + 4]
                    t += [m14[a0] ^ m11[a1] ^ m13[a2] ^ m9[a3], m9[a0] ^ m14[a1] ^ m11[a2] ^ m13[a3],
                          m13[a0] ^ m9[a1] ^ m14[a2] ^ m11[a3], m11[a0] ^ m13[a1] ^ m9[a2] ^ m14[a3]]
                s = t
        return bytes(s)

    def cbc_encrypt(self, iv, data):
        out, prev = bytearray(), iv
        for i in range(0, len(data), 16):
            prev = self.encrypt_block(bytes(a ^ b for a, b in zip(data[i:i + 16], prev)))
            out += prev
        return bytes(out)

    def cbc_decrypt(self, iv, data):
        out, prev = bytearray(), iv
        for i in range(0, len(data), 16):
            blk = data[i:i + 16]
            out += bytes(a ^ b for a, b in zip(self.decrypt_block(blk), prev))
            prev = blk
        return bytes(out)


IPMI_COMPLETION_CODES = {
    0xC0: "node busy", 0xC1: "invalid command", 0xC2: "invalid for LUN",
    0xC3: "timeout", 0xC4: "out of space", 0xC5: "reservation cancelled",
    0xC6: "request data truncated", 0xC7: "request data length invalid",
    0xC8: "request data field length limit exceeded",
    0xC9: "parameter out of range", 0xCA: "cannot return number of requested bytes",
    0xCB: "requested sensor, data, or record not present",
    0xCC: "invalid data field in request", 0xCD: "command illegal for sensor or record type",
    0xCE: "response could not be provided", 0xCF: "duplicated request",
    0xD0: "SDR repository in update mode", 0xD1: "device in firmware update mode",
    0xD2: "BMC initialization in progress", 0xD3: "destination unavailable",
    0xD4: "insufficient privilege level", 0xD5: "not supported in present state",
    0xD6: "sub-function disabled", 0xFF: "unspecified error",
}

RMCPPLUS_STATUS_CODES = {
    0x01: "insufficient resources to create a session",
    0x02: "invalid session ID", 0x03: "invalid payload type",
    0x04: "invalid authentication algorithm", 0x05: "invalid integrity algorithm",
    0x06: "no matching authentication payload", 0x07: "no matching integrity payload",
    0x08: "inactive session ID", 0x09: "invalid role",
    0x0A: "unauthorized role or privilege level requested",
    0x0B: "insufficient resources to create a session at the requested role",
    0x0C: "invalid name length", 0x0D: "unauthorized name (unknown user?)",
    0x0E: "unauthorized GUID", 0x0F: "invalid integrity check value",
    0x10: "invalid confidentiality algorithm",
    0x11: "no cipher suite match with proposed security algorithms",
    0x12: "illegal or unrecognized parameter",
}


class IpmiError(RuntimeError):
    """An IPMI command completed with a non-zero completion code."""

    def __init__(self, msg, code=None):
        super().__init__(msg)
        self.code = code


class IpmiTimeout(RuntimeError):
    """The BMC didn't answer (after retries)."""


class RmcpPlusSession:
    """One authenticated, integrity-protected, encrypted IPMI 2.0 session
    over UDP/623. Not thread-safe by itself; NativeIpmi serializes use."""

    # cipher suite -> (auth alg, integrity alg, confidentiality alg, hash, ICV/trailer length)
    CIPHER_SUITES = {
        3:  (0x01, 0x01, 0x01, hashlib.sha1, 12),
        17: (0x03, 0x04, 0x01, hashlib.sha256, 16),
    }
    PRIV_ADMIN = 0x04
    NAME_ONLY_LOOKUP = 0x10   # RAKP1 role flag: look the user up by name and privilege

    def __init__(self, host, user, password, port=623, cipher_suite=3,
                 kg=None, timeout=1.0, retries=3):
        if cipher_suite not in self.CIPHER_SUITES:
            raise ValueError(f"unsupported cipher suite {cipher_suite}")
        self.host, self.port = host, port
        self.user = user.encode()
        self.password = password.encode()
        if len(self.user) > 16:
            raise ValueError("IPMI user names are limited to 16 bytes")
        if len(self.password) > 20:
            raise ValueError("IPMI 2.0 passwords are limited to 20 bytes")
        # KG (the "BMC key"/"encryption key" in iDRAC settings) defaults to
        # the user's password when the BMC has none configured (all zeros).
        self.kg = kg if kg else self.password
        self.cipher_suite = cipher_suite
        self.timeout, self.retries = timeout, retries
        self._sock = None
        self.active = False

    # -- framing helpers -------------------------------------------------

    @staticmethod
    def _checksum(data):
        return (-sum(data)) & 0xFF

    def _ipmi_msg(self, netfn, cmd, data, lun, rq_seq):
        hdr = bytes([0x20, (netfn << 2) | (lun & 3)])
        body = bytes([0x81, (rq_seq << 2), cmd]) + bytes(data)
        return hdr + bytes([self._checksum(hdr)]) + body + bytes([self._checksum(body)])

    def _send_raw(self, packet):
        self._sock.send(packet)

    def _exchange(self, build, match, what):
        """Send build() (rebuilt each attempt, so retransmits get fresh
        session sequence numbers) until match(datagram) returns a non-None
        result. Stray/late datagrams that don't match are ignored."""
        for _attempt in range(self.retries):
            self._send_raw(build())
            deadline = time.monotonic() + self.timeout
            while True:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                self._sock.settimeout(left)
                try:
                    pkt = self._sock.recv(4096)
                except socket.timeout:
                    break
                except OSError:
                    break   # e.g. ICMP port unreachable surfacing as ECONNREFUSED
                result = match(pkt)
                if result is not None:
                    return result
        raise IpmiTimeout(f"no response from BMC {self.host}:{self.port} ({what})")

    # -- session setup ---------------------------------------------------

    def _presession_packet(self, payload_type, payload):
        return (b"\x06\x00\xff\x07" + bytes([0x06, payload_type]) + b"\x00" * 8
                + struct.pack("<H", len(payload)) + payload)

    def _presession_exchange(self, req_type, payload, rsp_type, tag, what):
        def match(pkt):
            if len(pkt) < 16 or pkt[:4] != b"\x06\x00\xff\x07" or pkt[4] != 0x06:
                return None
            if pkt[5] & 0x3F != rsp_type:
                return None
            (length,) = struct.unpack_from("<H", pkt, 14)
            body = pkt[16:16 + length]
            if len(body) < 2 or body[0] != tag:
                return None
            return body
        body = self._exchange(lambda: self._presession_packet(req_type, payload), match, what)
        if body[1] != 0:
            raise IpmiError(f"{what} rejected by BMC: "
                            f"{RMCPPLUS_STATUS_CODES.get(body[1], 'status 0x%02x' % body[1])}",
                            body[1])
        return body

    def _get_channel_auth_caps(self):
        """IPMI 1.5-framed, session-less. Confirms the BMC is there and
        answering before we start the RAKP handshake (some BMCs expect it)."""
        def build():
            msg = self._ipmi_msg(0x06, 0x38, [0x8E, self.PRIV_ADMIN], 0, 0)
            return b"\x06\x00\xff\x07" + b"\x00" + b"\x00" * 8 + bytes([len(msg)]) + msg

        def match(pkt):
            if len(pkt) < 21 or pkt[:4] != b"\x06\x00\xff\x07" or pkt[4] != 0x00:
                return None
            msg = pkt[14:14 + pkt[13]]
            if len(msg) < 8 or msg[5] != 0x38:
                return None
            return msg
        msg = self._exchange(build, match, "get channel auth capabilities")
        if msg[6] != 0:
            raise IpmiError("get channel auth capabilities failed: "
                            f"{IPMI_COMPLETION_CODES.get(msg[6], 'code 0x%02x' % msg[6])}", msg[6])
        if len(msg) > 8 and not msg[8] & 0x80:
            raise IpmiError("BMC does not support IPMI v2.0 / RMCP+ (enable IPMI over LAN?)")

    def open(self):
        self.close()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.connect((self.host, self.port))
        try:
            self._handshake()
        except Exception:
            self.close(graceful=False)
            raise

    def _handshake(self):
        auth_alg, integ_alg, conf_alg, hfunc, icv_len = self.CIPHER_SUITES[self.cipher_suite]
        hmac_ = lambda key, data: hmac.new(key, data, hfunc).digest()

        self._get_channel_auth_caps()

        # Open Session
        self.console_sid = struct.unpack("<I", os.urandom(4))[0] | 1
        tag = os.urandom(1)[0]
        payload = (bytes([tag, self.PRIV_ADMIN, 0, 0]) + struct.pack("<I", self.console_sid)
                   + bytes([0x00, 0, 0, 8, auth_alg, 0, 0, 0])
                   + bytes([0x01, 0, 0, 8, integ_alg, 0, 0, 0])
                   + bytes([0x02, 0, 0, 8, conf_alg, 0, 0, 0]))
        rsp = self._presession_exchange(0x10, payload, 0x11, tag, "open session")
        if struct.unpack_from("<I", rsp, 4)[0] != self.console_sid:
            raise IpmiError("open session response for a different console session")
        self.bmc_sid = struct.unpack_from("<I", rsp, 8)[0]
        sid_m, sid_c = struct.pack("<I", self.console_sid), struct.pack("<I", self.bmc_sid)

        # RAKP 1/2
        rm = os.urandom(16)
        role = self.PRIV_ADMIN | self.NAME_ONLY_LOOKUP
        tag = os.urandom(1)[0]
        payload = (bytes([tag, 0, 0, 0]) + sid_c + rm + bytes([role, 0, 0, len(self.user)])
                   + self.user)
        rsp = self._presession_exchange(0x12, payload, 0x13, tag, "RAKP 1")
        rc, guid_c, auth_code = rsp[8:24], rsp[24:40], rsp[40:]
        tail = bytes([role, len(self.user)]) + self.user
        if not hmac.compare_digest(auth_code, hmac_(self.password, sid_m + sid_c + rm + rc
                                                   + guid_c + tail)):
            raise IpmiError("RAKP 2 authentication failed: wrong password for user "
                            f"'{self.user.decode(errors='replace')}'?")

        # Session keys
        self._sik = hmac_(self.kg, rm + rc + tail)
        digest_len = hfunc().digest_size
        self._k1 = hmac_(self._sik, b"\x01" * digest_len)
        self._aes = _Aes128(hmac_(self._sik, b"\x02" * digest_len)[:16])
        self._hfunc, self._icv_len = hfunc, icv_len

        # RAKP 3/4
        tag = os.urandom(1)[0]
        payload = bytes([tag, 0, 0, 0]) + sid_c + hmac_(self.password, rc + sid_m + tail)
        rsp = self._presession_exchange(0x14, payload, 0x15, tag, "RAKP 3")
        if not hmac.compare_digest(rsp[8:8 + icv_len],
                                   hmac_(self._sik, rm + sid_c + guid_c)[:icv_len]):
            raise IpmiError("RAKP 4 integrity check failed (wrong KG / BMC key?)")

        self._out_seq = 0
        self._rq_seq = 0
        self.active = True
        # Sessions start at User privilege; raise to Administrator.
        self.command(0x06, 0x3B, [self.PRIV_ADMIN])

    # -- established session ---------------------------------------------

    def _session_packet(self, msg):
        self._out_seq = (self._out_seq + 1) & 0xFFFFFFFF or 1
        iv = os.urandom(16)
        pad = (16 - (len(msg) + 1) % 16) % 16
        enc = iv + self._aes.cbc_encrypt(iv, msg + bytes(range(1, pad + 1)) + bytes([pad]))
        sess = (bytes([0x06, 0xC0]) + struct.pack("<IIH", self.bmc_sid, self._out_seq, len(enc))
                + enc)
        ipad = (4 - (len(sess) + 2) % 4) % 4
        sess += b"\xff" * ipad + bytes([ipad, 0x07])
        return b"\x06\x00\xff\x07" + sess + hmac.new(self._k1, sess, self._hfunc).digest()[:self._icv_len]

    def _parse_session_packet(self, pkt):
        """Return the decrypted IPMI message, or None if this datagram isn't
        a valid, authentic packet for our session."""
        if len(pkt) < 16 or pkt[:4] != b"\x06\x00\xff\x07" or pkt[4] != 0x06:
            return None
        if pkt[5] & 0x3F != 0x00 or pkt[5] & 0xC0 != 0xC0:
            return None
        sid, _seq, length = struct.unpack_from("<IIH", pkt, 6)
        if sid != self.console_sid:
            return None
        sess, icv = pkt[4:-self._icv_len], pkt[-self._icv_len:]
        if not hmac.compare_digest(icv, hmac.new(self._k1, sess, self._hfunc).digest()[:self._icv_len]):
            return None
        enc = pkt[16:16 + length]
        if len(enc) < 32 or len(enc) % 16:
            return None
        plain = self._aes.cbc_decrypt(enc[:16], enc[16:])
        pad = plain[-1]
        if pad > 15:
            return None
        return plain[:-(pad + 1)]

    def command(self, netfn, cmd, data=(), lun=0):
        """Send one request; return the response data (after the completion
        code). Raises IpmiError on a non-zero completion code, IpmiTimeout
        if the BMC doesn't answer."""
        if not self.active:
            raise IpmiError("IPMI session is not open")
        self._rq_seq = (self._rq_seq + 1) & 0x3F
        rq_seq = self._rq_seq
        msg = self._ipmi_msg(netfn, cmd, data, lun, rq_seq)

        def match(pkt):
            m = self._parse_session_packet(pkt)
            if (m is None or len(m) < 8 or m[1] >> 2 != netfn + 1 or m[4] >> 2 != rq_seq
                    or m[5] != cmd):
                return None
            return m
        rsp = self._exchange(lambda: self._session_packet(msg), match,
                             f"netfn 0x{netfn:02x} cmd 0x{cmd:02x}")
        code = rsp[6]
        if code != 0:
            raise IpmiError(f"IPMI netfn 0x{netfn:02x} cmd 0x{cmd:02x} failed: "
                            f"{IPMI_COMPLETION_CODES.get(code, 'completion code 0x%02x' % code)}",
                            code)
        return rsp[7:-1]

    def close(self, graceful=True):
        """Best-effort Close Session, so we don't leave a slot occupied on
        the BMC (they only have a handful). graceful=False skips that for a
        session we believe is already dead."""
        if self.active and graceful:
            try:
                saved_retries, self.retries = self.retries, 1
                self.command(0x06, 0x3C, struct.pack("<I", self.bmc_sid))
            except Exception:
                pass
            finally:
                self.retries = saved_retries
        self.active = False
        if self._sock is not None:
            self._sock.close()
            self._sock = None


def _sdr_twos(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


_SDR_LINEARIZATION = {
    0: lambda v: v, 1: math.log, 2: math.log10, 3: math.log2, 4: math.exp,
    5: lambda v: 10 ** v, 6: lambda v: 2 ** v, 7: lambda v: 1 / v,
    8: lambda v: v * v, 9: lambda v: v ** 3, 10: math.sqrt,
    11: lambda v: math.copysign(abs(v) ** (1 / 3), v),
}


class SdrSensor:
    """The fields of a Full Sensor Record (SDR type 0x01) needed to read
    and convert one analog sensor."""

    def __init__(self, rec):
        self.owner = rec[5]
        self.lun = rec[6] & 0x03
        self.number = rec[7]
        self.sensor_type = rec[12]
        self.analog_format = rec[20] >> 6
        self.unit = rec[21]
        self.linearization = rec[23] & 0x7F
        self.m = _sdr_twos(rec[24] | ((rec[25] & 0xC0) << 2), 10)
        self.b = _sdr_twos(rec[26] | ((rec[27] & 0xC0) << 2), 10)
        self.r_exp = _sdr_twos(rec[29] >> 4, 4)
        self.b_exp = _sdr_twos(rec[29] & 0x0F, 4)
        id_len = rec[47] & 0x1F if len(rec) > 47 else 0
        self.name = rec[48:48 + id_len].decode("ascii", "replace").strip("\x00 ")

    @property
    def sensor_id(self):
        return f"{self.number:02x}h"

    def convert(self, raw):
        if self.analog_format == 1:
            raw = _sdr_twos(raw, 8) + (1 if raw & 0x80 else 0)  # one's complement
        elif self.analog_format == 2:
            raw = _sdr_twos(raw, 8)
        elif self.analog_format == 3:
            return None                                         # no analog reading
        value = (self.m * raw + self.b * 10 ** self.b_exp) * 10 ** self.r_exp
        lin = _SDR_LINEARIZATION.get(self.linearization)
        return lin(value) if lin else None


class NativeIpmi:
    """iDRAC backend over our own RMCP+ client: one persistent session,
    re-established transparently when it goes stale (iDRAC reset, idle
    timeout, network blip)."""

    BMC_ADDR = 0x20
    SENSOR_TYPE_TEMPERATURE = 0x01
    # iDRAC drops idle IPMI sessions after a while; rather than probe a
    # possibly-dead session (and eat a timeout), re-login if we've been
    # quiet longer than this. Polls are normally far more frequent.
    MAX_IDLE = 30.0

    def __init__(self, host, user, password, port=623, cipher_suite=None, kg=None):
        self._args = (host, user, password, port, kg)
        # None = try 3 (iDRAC 7/8) and fall back to 17 (newer firmware).
        self._suites = [cipher_suite] if cipher_suite else [3, 17]
        self._session = None
        self._last_used = 0.0
        self._lock = threading.Lock()
        self._sensors = None           # {sensor_id: SdrSensor}, read once
        self._warned_missing = set()
        self.logins = 0

    def _connect(self, reason=None):
        host, user, password, port, kg = self._args
        last = None
        for suite in self._suites:
            s = RmcpPlusSession(host, user, password, port=port, cipher_suite=suite, kg=kg)
            try:
                s.open()
            except IpmiError as e:
                # Only a cipher-suite mismatch is worth retrying with the next suite.
                if e.code in (0x04, 0x05, 0x10, 0x11) and suite != self._suites[-1]:
                    last = e
                    continue
                raise
            self.logins += 1
            if reason:
                log(f"IPMI session re-established after {reason} (login #{self.logins}).",
                    level="WARNING")
            if len(self._suites) > 1:
                self._suites = [suite]  # remember what works
            return s
        raise last

    def _cmd(self, netfn, cmd, data=(), lun=0):
        with self._lock:
            now = time.monotonic()
            if self._session is not None and now - self._last_used > self.MAX_IDLE:
                self._session.close()
                self._session = None
            fresh = self._session is None
            if fresh:
                self._session = self._connect()
            try:
                rsp = self._session.command(netfn, cmd, data, lun)
            except IpmiTimeout:
                # The session may have died under us (iDRAC reset drops all
                # sessions). Log in again once and retry before giving up.
                self._session.close(graceful=False)
                self._session = None
                if fresh:
                    raise
                self._session = self._connect("a timeout")
                rsp = self._session.command(netfn, cmd, data, lun)
            self._last_used = time.monotonic()
            return rsp

    def close(self):
        with self._lock:
            if self._session is not None:
                self._session.close()
                self._session = None

    # -- the backend interface --------------------------------------------

    def set_manual_mode(self):
        self._cmd(0x30, 0x30, [0x01, 0x00])

    def set_auto_mode(self):
        self._cmd(0x30, 0x30, [0x01, 0x01])

    def set_fan_speed(self, percent):
        self._cmd(0x30, 0x30, [0x02, 0xFF, max(0, min(100, int(percent)))])

    def _read_sdr_record(self, record_id, chunk):
        """Return (next_record_id, record_bytes), shrinking the read size
        if the BMC can't return as much as we asked for."""
        while True:
            res = self._cmd(0x0A, 0x22)[:2]          # Reserve SDR Repository
            try:
                hdr = self._cmd(0x0A, 0x23, res + struct.pack("<HBB", record_id, 0, 5))
                next_id = struct.unpack_from("<H", hdr)[0]
                rec = bytearray(hdr[2:7])
                total = 5 + rec[4]
                while len(rec) < total:
                    n = min(chunk[0], total - len(rec))
                    try:
                        part = self._cmd(0x0A, 0x23, res + struct.pack("<HBB", record_id,
                                                                       len(rec), n))
                    except IpmiError as e:
                        if e.code in (0xCA, 0xC7, 0xFF) and chunk[0] > 4:
                            chunk[0] //= 2
                            continue
                        raise
                    rec += part[2:]
                return next_id, bytes(rec[:total])
            except IpmiError as e:
                if e.code == 0xC5:   # reservation lost (someone else read the SDR); retry
                    continue
                raise

    def load_sensors(self):
        """Walk the SDR repository once, keeping BMC-owned temperature
        sensors. The SDR is static, so this survives reconnects."""
        sensors, record_id, chunk, seen = {}, 0x0000, [16], 0
        while record_id != 0xFFFF and seen < 1024:
            next_id, rec = self._read_sdr_record(record_id, chunk)
            seen += 1
            if (len(rec) >= 48 and rec[3] == 0x01 and rec[5] == self.BMC_ADDR
                    and rec[12] == self.SENSOR_TYPE_TEMPERATURE):
                s = SdrSensor(rec)
                sensors.setdefault(s.sensor_id, s)
            if next_id == record_id:
                break
            record_id = next_id
        self._sensors = sensors
        return sensors

    def read_sensor(self, sensor):
        """Converted reading, or None if the sensor has no valid reading."""
        try:
            rsp = self._cmd(0x04, 0x2D, [sensor.number], lun=sensor.lun)
        except IpmiError as e:
            if e.code in (0xCB, 0xCD, 0xD5):
                return None
            raise
        if len(rsp) < 2 or rsp[1] & 0x20 or not rsp[1] & 0x40:
            return None   # reading unavailable / scanning disabled
        return sensor.convert(rsp[0])

    def read_temps(self, sensor_ids):
        if self._sensors is None:
            self.load_sensors()
        readings = {}
        for sid in sensor_ids:
            sensor = self._sensors.get(normalize_sensor_id(sid))
            if sensor is None:
                if sid not in self._warned_missing:
                    self._warned_missing.add(sid)
                    found = ", ".join(f"{k.upper()} ({v.name})" for k, v in
                                      sorted(self._sensors.items())) or "none"
                    log(f"Sensor {sid} is not a temperature sensor in the iDRAC's SDR. "
                        f"Available: {found}", level="WARNING")
                continue
            value = self.read_sensor(sensor)
            if value is not None:
                readings[sid] = int(round(value))
        return readings


def normalize_sensor_id(sid):
    """'0Eh', '0x0e', '0e', '14' (hex) -> '0eh', the form ipmitool prints."""
    s = sid.strip().lower()
    s = s[2:] if s.startswith("0x") else s
    s = s[:-1] if s.endswith("h") else s
    try:
        return f"{int(s, 16):02x}h"
    except ValueError:
        return sid.strip().lower()


class IpmitoolIpmi:
    """Fallback backend (--ipmi-backend ipmitool): shells out to ipmitool,
    which opens and tears down a full RMCP+ session for every command."""

    def __init__(self, host, user, password, port=623, interface="lanplus"):
        self.host = host
        self.port = port
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
            "-H", self.host, "-p", str(self.port), "-U", self.user, "-E",
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
            m = re.search(r"(-?\d+(?:\.\d+)?)\s*degrees", parts[-1], re.IGNORECASE)
            if m:
                readings[sensor_id] = int(round(float(m.group(1))))
        return readings

    def close(self):
        pass


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

    def close(self):
        pass

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
    toward the curve in rise_splits equal steps (instantly at/above the
    hard cap), and falls at most fall_rate percent per poll. The curve
    is driven by a hysteresis-filtered control temperature (rises
    instantly, falls only after a temp_hysteresis drop) so
    single-degree sensor flicker doesn't bounce the fans.

    Parameters are read live from SharedState each poll, so web-UI
    changes take effect immediately.
    """

    def __init__(self, state):
        self.state = state
        self.current_speed = float(state.get_params()["base_speed"])
        self.control_temp = None
        self.last_desired = self.current_speed
        self._rise_target = None
        self._rise_step = 0.0

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
            if self.control_temp >= p["hard_cap_temp"]:
                # Emergency: no smoothing at/above the hard cap.
                self.current_speed = desired
                self._rise_target = None
            else:
                # Spread the rise evenly across rise_splits polls. If the
                # curve target moves mid-rise, recompute the step from the
                # remaining gap so the schedule restarts from here.
                if self._rise_target != desired:
                    self._rise_target = desired
                    self._rise_step = (desired - self.current_speed) / max(1, p["rise_splits"])
                self.current_speed = min(desired, self.current_speed + self._rise_step)
        else:
            self._rise_target = None
            self.current_speed = max(desired, self.current_speed - p["fall_rate"])
        return int(round(self.current_speed))


# ---------------------------------------------------------------------------
# Control loop
# ---------------------------------------------------------------------------

def fmt_readings(readings, aliases):
    return ", ".join(f"{aliases.get(sid, sid)}={v}C" for sid, v in readings.items())


def run_loop(ipmi, controller, state, sensor_ids, max_failed_polls):
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
                    f"--list-sensors")

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
            log(f"Poll failed ({consecutive_failures}/{max_failed_polls}): {e}",
                level="WARNING")
            state.set_status(state="failsafe" if failsafe else "degraded",
                             consecutive_failures=consecutive_failures,
                             last_error=str(e))

            if consecutive_failures >= max_failed_polls and not failsafe:
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
    """Hand control back to iDRAC, pin the exit speed, or -- with
    exit_speed=CURRENT_SPEED -- leave everything exactly as-is. Retries
    because this is the last line of defense before the process goes away."""
    if not revert_on_exit and exit_speed == CURRENT_SPEED:
        # Deliberately touch nothing: the iDRAC keeps manual mode at the
        # last speed the loop applied (or automatic mode, if the failsafe
        # had already handed control back).
        log("Exiting without changing fan control: fans stay at the last "
            "applied speed (exit-speed=current). Nothing is watching "
            "temperatures now.", level="WARNING")
        return
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
                snap["hostname"] = HOSTNAME
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
<title>quidrac &mdash; iDRAC fan control</title>
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
  <span class="sub" id="hostname"></span>
  <span class="chip" id="chip"><span class="dot" id="chipdot"></span><span id="chiptext">connecting&hellip;</span></span>
</header>

<div class="tiles">
  <div class="tile"><div class="label">Fan speed</div><div class="value" id="tSpeed">&ndash;</div><div class="note" id="tSpeedNote"></div></div>
  <div class="tile"><div class="label">Hottest sensor</div><div class="value" id="tMax">&ndash;</div><div class="note" id="tMaxNote"></div></div>
  <div class="tile"><div class="label">Control temp</div><div class="value" id="tCtrl">&ndash;</div><div class="note">hysteresis-filtered</div></div>
  <div class="tile"><div class="label">Headroom to hard cap</div><div class="value" id="tHead">&ndash;</div><div class="note" id="tHeadNote"></div></div>
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
    S.aliases = j.aliases || {}; S.hostname = j.hostname || "";
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

function fmtTemp(v) { return v == null ? "\u2013" : v + "\u00b0C"; }

function render() {
  if (S.hostname && $("hostname").textContent !== "\u00b7 " + S.hostname) {
    $("hostname").textContent = "\u00b7 " + S.hostname;
    document.title = "quidrac \u2014 " + S.hostname;
  }
  const st = S.status || {};
  if (st.state === "running") setChip(css("--good"), st.demo ? "running (demo)" : "running");
  else if (st.state === "degraded") setChip(css("--warning"), "degraded \u2014 " + st.consecutive_failures + " failed poll" + (st.consecutive_failures === 1 ? "" : "s"));
  else if (st.state === "failsafe") setChip(css("--critical"), "FAILSAFE \u2014 auto control");
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
    $("tHead").textContent = head + "\u00b0C";
    $("tHeadNote").textContent = "hard cap at " + last.cap + "\u00b0C";
  }
  if (S.settingsFile) {
    $("pdesc").textContent = "Applied on the next poll and saved to " + S.settingsFile +
      (S.overridesActive ? " (overriding script settings). " : ". ") +
      "\u201cRestore script settings\u201d returns to CLI flags/defaults and removes the file.";
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
    ctx.fillText("waiting for samples\u2026", (PADL + w - PADR) / 2, h / 2);
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
  geomT = drawChart($("chartT"), series, Math.floor(lo - pad), Math.ceil(hi + pad), "\u00b0");
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
  for (const htext of ["Time", ...S.sensorIds.map(s => aliasOf(s) + " \u00b0C"), "Control \u00b0C", "Speed %", "Curve %", "Target \u00b0C", "Cap \u00b0C"]) {
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
attachHover($("chartT"), $("tipT"), tempSeries, "\u00b0C");
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

def list_sensors(ipmi):
    """--list-sensors: print temperature sensors and exit. Returns an exit code."""
    if isinstance(ipmi, IpmitoolIpmi):
        print(ipmi._run(["sdr", "type", "Temperature"]), end="")
        return 0
    if isinstance(ipmi, DemoIpmi):
        print("--list-sensors needs a real iDRAC (not --demo).", file=sys.stderr)
        return 1
    try:
        sensors = ipmi.load_sensors()
        print(f"{'ID':<5} {'Name':<17} Reading")
        for sid, sensor in sorted(sensors.items()):
            value = ipmi.read_sensor(sensor)
            shown = "no reading" if value is None else f"{value:g} C"
            print(f"{sid[:-1].upper() + 'h':<5} {sensor.name:<17} {shown}")
        if not sensors:
            print("(no temperature sensors found in the SDR repository)")
        return 0
    except Exception as e:
        print(f"Failed to list sensors: {e}", file=sys.stderr)
        return 1
    finally:
        ipmi.close()


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
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"iDRAC IPMI-over-LAN UDP port (default: {DEFAULT_PORT})")
    parser.add_argument("--ipmi-backend", choices=("native", "ipmitool"),
                        default=DEFAULT_IPMI_BACKEND,
                        help="native: built-in IPMI client holding one persistent "
                             "session (no ipmitool needed). ipmitool: run ipmitool for "
                             "every command, logging in each time. "
                             f"(default: {DEFAULT_IPMI_BACKEND})")
    parser.add_argument("--cipher-suite", type=int, choices=(3, 17), default=None,
                        help="Native backend only: RMCP+ cipher suite. Default tries 3 "
                             "(iDRAC 7/8) and falls back to 17 if the iDRAC rejects it.")
    parser.add_argument("--list-sensors", action="store_true",
                        help="Print the iDRAC's temperature sensors (IDs, names, current "
                             "readings) and exit, without touching fan control")
    parser.add_argument("--revert-to-auto", action="store_true",
                        help="Hand fan control back to the iDRAC's automatic mode and "
                             "exit (for a systemd ExecStopPost backstop)")
    parser.add_argument("--demo", action="store_true",
                        help="Run against a simulated iDRAC (no credentials needed); "
                             "useful for previewing the web UI and tuning parameters")

    parser.add_argument("--sensors", default=DEFAULT_SENSORS,
                        help=f"Comma-separated sensor IDs to monitor (default: {DEFAULT_SENSORS})")
    parser.add_argument("--sensor-aliases", default=None,
                        help='Friendly sensor names as comma-separated ID=NAME pairs, '
                             'e.g. "0Eh=CPU1,0Fh=CPU2". Merged over the SENSOR_ALIASES '
                             'dict in this script (flag wins per sensor).')

    # The seven live-tunable parameters use default=argparse.SUPPRESS so we can
    # tell "explicitly passed" apart from "left at default": explicit flags
    # outrank the settings file, script defaults sit below it.
    parser.add_argument("--poll-interval", type=int, default=argparse.SUPPRESS,
                        help=f"Seconds between polls (default: {DEFAULT_POLL_INTERVAL})")
    parser.add_argument("--base-speed", type=int, default=argparse.SUPPRESS,
                        help=f"Minimum fan speed %%, used at/below --target-temp "
                             f"(default: {DEFAULT_BASE_SPEED})")
    parser.add_argument("--target-temp", type=float, default=argparse.SUPPRESS,
                        help=f"Curve start (C): at/below this temp fans run at "
                             f"--base-speed (default: {DEFAULT_TARGET_TEMP})")
    parser.add_argument("--hard-cap-temp", type=float, default=argparse.SUPPRESS,
                        help=f"Curve end (C): at/above this temp fans jump to "
                             f"--max-speed (default: {DEFAULT_HARD_CAP_TEMP})")
    parser.add_argument("--max-speed", type=int, default=argparse.SUPPRESS,
                        help=f"Maximum fan speed %% (default: {DEFAULT_MAX_SPEED})")
    parser.add_argument("--fall-rate", type=float, default=argparse.SUPPRESS,
                        help=f"Max fan speed decrease per poll when cooling (%%/poll). "
                             f"Rises are always instant. (default: {DEFAULT_FALL_RATE})")
    parser.add_argument("--rise-splits", type=int, default=argparse.SUPPRESS,
                        help=f"Spread each fan speed rise across this many polls; "
                             f"1 = instant. Rises at/above the hard cap are always "
                             f"instant. (default: {DEFAULT_RISE_SPLITS})")
    parser.add_argument("--temp-hysteresis", type=float, default=argparse.SUPPRESS,
                        help=f"Temp must fall this many degrees below its recent peak "
                             f"before fans follow it down (C). Prevents fan bounce from "
                             f"1C sensor flicker; 0 disables. (default: {DEFAULT_TEMP_HYSTERESIS})")
    # Config/flag-only (not live-tunable, not in the web UI or settings file).
    parser.add_argument("--max-failed-polls", type=int, default=DEFAULT_MAX_FAILED_POLLS,
                        help=f"Consecutive failed polls before the failsafe reverts "
                             f"iDRAC to automatic fan control (default: {DEFAULT_MAX_FAILED_POLLS})")
    parser.add_argument("--revert-on-exit", dest="revert_on_exit", action="store_true",
                        default=DEFAULT_REVERT_ON_EXIT,
                        help="Revert to automatic iDRAC fan control on exit (default: on)")
    parser.add_argument("--no-revert-on-exit", dest="revert_on_exit", action="store_false",
                        help="Do NOT revert to automatic control on exit; hold --exit-speed instead")
    parser.add_argument("--exit-speed", type=exit_speed_type, default=DEFAULT_EXIT_SPEED,
                        help="Fan speed %% to hold on exit when --no-revert-on-exit is set, "
                             "or 'current' to leave the fans at their last applied speed "
                             f"(default: {DEFAULT_EXIT_SPEED})")

    parser.add_argument("--settings-file", default=None,
                        help=f"Path where web-UI parameter changes are saved and, if "
                             f"present at startup, override script defaults "
                             f"(explicitly passed CLI flags still win; default: "
                             f"{SETTINGS_FILENAME} next to this script)")
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
        if args.ipmi_backend == "native":
            ipmi = NativeIpmi(args.host, args.user, password, port=args.port,
                              cipher_suite=args.cipher_suite)
        else:
            ipmi = IpmitoolIpmi(args.host, args.user, password, port=args.port)

    if args.list_sensors:
        sys.exit(list_sensors(ipmi))
    if args.revert_to_auto:
        try:
            ipmi.set_auto_mode()
            log("Reverted to automatic iDRAC fan control.")
        except Exception as e:
            log(f"Revert to automatic fan control failed: {e}", level="CRITICAL")
            sys.exit(1)
        finally:
            ipmi.close()
        sys.exit(0)

    sensor_ids = [s.strip().lower() for s in args.sensors.split(",") if s.strip()]
    settings_path = args.settings_file or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), SETTINGS_FILENAME)

    script_defaults = {
        "base_speed": DEFAULT_BASE_SPEED,
        "max_speed": DEFAULT_MAX_SPEED,
        "target_temp": DEFAULT_TARGET_TEMP,
        "hard_cap_temp": DEFAULT_HARD_CAP_TEMP,
        "fall_rate": DEFAULT_FALL_RATE,
        "rise_splits": DEFAULT_RISE_SPLITS,
        "temp_hysteresis": DEFAULT_TEMP_HYSTERESIS,
        "poll_interval": DEFAULT_POLL_INTERVAL,
    }
    if args.max_failed_polls < 1:
        parser.error("--max-failed-polls must be at least 1")
    explicit = {n: getattr(args, n) for n in PARAM_SPEC if hasattr(args, n)}
    try:
        baseline = validate_params({**script_defaults, **explicit})
    except ValueError as e:
        parser.error(str(e))

    # Precedence, lowest to highest: script defaults -> settings file ->
    # explicitly passed CLI flags (an explicit flag is an explicit request).
    file_overrides = read_settings_overrides(settings_path)
    try:
        params = validate_params({**script_defaults, **file_overrides, **explicit})
    except ValueError as e:
        log(f"Settings file {settings_path} conflicts with current flags/defaults "
            f"({e}); ignoring the file.", level="WARNING")
        file_overrides = {}
        params = baseline
    for n in sorted(set(explicit) & set(file_overrides)):
        try:
            file_value = PARAM_SPEC[n][0](file_overrides[n])
        except (TypeError, ValueError):
            continue
        if file_value != params[n]:
            log(f"CLI flag overrides settings file: {n} flag={params[n]} file={file_value}")
    applied = [f"{n}={params[n]}"
               for n in sorted(set(file_overrides) - set(explicit))
               if params[n] != baseline[n]]
    if applied:
        log(f"Loaded settings overrides from {settings_path}: {', '.join(applied)} "
            f"(web-UI 'Restore script settings' or deleting the file reverts).")

    aliases = {k.lower(): v for k, v in SENSOR_ALIASES.items()}
    if args.sensor_aliases:
        try:
            aliases.update(parse_sensor_aliases(args.sensor_aliases))
        except ValueError as e:
            parser.error(str(e))
    state = SharedState(params, baseline=baseline, settings_path=settings_path,
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
        f"{args.max_failed_polls} failed polls.")

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
        run_loop(ipmi, controller, state, sensor_ids, args.max_failed_polls)
    finally:
        # Don't let a second Ctrl-C interrupt the revert.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if engaged:
            cleanup(ipmi, args.revert_on_exit, args.exit_speed)
        ipmi.close()


if __name__ == "__main__":
    main()
