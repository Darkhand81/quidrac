# quidrac

**Keep iDRAC quiet.** A software thermal control loop for Dell servers that
holds fan speeds as low as possible without overheating, using raw IPMI fan
override — with a built-in web dashboard for live tuning and monitoring.

Dell's automatic fan profiles are conservative: they spin fans well above
what's needed for safe temperatures, which makes rack servers unpleasant in a
home or office. quidrac takes over fan control from the iDRAC and runs its own
loop: quiet at idle, instant response to heat, and multiple layers of failsafe
so a bug or network drop can't cook the machine.

> **Important:** while quidrac runs, it *is* the thermal control loop — the
> iDRAC's automatic algorithm is disabled. Run it under a process supervisor
> (systemd example below) and read the [Safety](#safety--failure-handling)
> section before deploying.

## Requirements

- Python 3.8+ (standard library only — no pip packages)
- `ipmitool`
- A Dell server with iDRAC and **IPMI over LAN enabled**
  (iDRAC Settings → Network → IPMI Settings)
- Tested against iDRAC generations that support the `0x30 0x30` raw fan
  commands (iDRAC 7/8 era; recent iDRAC 9 firmware removed them)

## Quick start

```sh
# Preview everything with no hardware (simulated server + dashboard):
./quidrac.py --demo

# Real server:
export IPMI_PASSWORD='yourpassword'
./quidrac.py --host 192.168.1.100 --user root
```

Then open `http://<machine-running-quidrac>:8080/` for the dashboard.

Find your temperature sensor IDs with:

```sh
ipmitool -I lanplus -H <idrac-ip> -U root -E sdr type Temperature
```

and pass them as `--sensors "0Eh,0Fh"` (the defaults are the two CPU package
sensors on many 12th-gen machines). Give them friendly names with
`--sensor-aliases "0Eh=CPU1,0Fh=CPU2"` or by editing `SENSOR_ALIASES` at the
top of the script.

## How it works

### The fan curve

Fan speed is a pure function of the hottest monitored sensor:

```
temp ≤ target-temp   (65°C)  →  base-speed (30%)
temp ≥ hard-cap-temp (80°C)  →  max-speed (100%)
in between                   →  linear interpolation

 speed
 100% ┤                        ╭────────
      │                      ╱
      │                    ╱
      │                  ╱
  30% ┤────────────────╱
      └───────────────┬────────┬───────── temp
                    65°C     80°C
                   target   hard cap
```

Two asymmetric rules turn that curve into quiet-but-safe behavior:

- **Rises are instant.** If the curve asks for a higher speed, it's applied on
  that same poll. Hitting the hard cap jumps straight to max speed — there is
  no gradual ramp during an emergency.
- **Falls are slow.** Speed decays at most `--fall-rate` percent per poll
  (default 2%/poll) and never below the curve. After a load spike passes, the
  fans drift down and settle at the *lowest* speed that holds the temperature
  at the curve — which is the whole point.

### Control temperature (and how it differs from target and CPU temps)

The dashboard plots three kinds of temperature, and they're easy to conflate:

- **CPU temps** are the raw per-sensor readings from the iDRAC, exactly as
  reported, once per poll. IPMI reports whole degrees only.
- **Target temp** isn't a measurement at all — it's a setting: the curve's
  lower endpoint. Nothing tries to "reach" it; it's simply the temperature
  where fan speed starts lifting off the base. Below it, fans idle at base
  speed and the CPUs settle wherever the workload puts them.
- **Control temp** is the value actually fed into the curve — a *filtered copy
  of the hottest CPU temp*. It follows raw readings **upward instantly**, but
  only follows them **downward** after they've fallen at least
  `--temp-hysteresis` degrees (default 2°C) below it. In effect, it remembers
  the recent peak and holds it until the cooling is real.

Why the filter exists: because IPMI reports whole degrees, a CPU sitting at
"67.5-ish" reads as a square wave flickering between 67 and 68. Without
hysteresis, every 68 would snap the fans up and every 67 would let them decay
— a constant, audible sawtooth. With it, control temp latches onto the top of
the flicker band and the fan speed sits perfectly still until the temperature
genuinely falls.

What you'll see on the chart:

- **Steady state:** control temp rides at the top of the flicker band, often
  1°C above the current raw reading, and the fan speed holds rock steady.
- **After a load drop:** raw temps fall away while control temp stays flat,
  then snaps down once the drop exceeds the hysteresis. The fans are always
  sized for the *recent worst case*, at a cost of at most ~2°C worth of extra
  fan speed during cooldowns.
- **During a rise:** control temp equals the hottest raw temp on the same
  poll. There is deliberately zero lag in that direction, so spike response
  and the hard-cap jump are unaffected.

## Safety / failure handling

Because quidrac replaces the iDRAC's own algorithm, it's built to fail toward
handing control *back*:

- **Dead-man's switch.** After `--max-failed-polls` consecutive failed polls
  (IPMI errors, timeouts, or no sensor readings — ~50 s at defaults), quidrac
  assumes it's flying blind, reverts the iDRAC to automatic fan control, and
  retries that revert every poll until it succeeds. When polling recovers,
  manual control re-engages automatically.
- **Self-healing override.** Manual mode and fan speed are re-asserted on
  *every* poll (both raw commands are cheap and idempotent), so an iDRAC
  reset — firmware update, `racreset`, watchdog — can't silently drop the
  override while quidrac keeps believing it's active.
- **Cleanup on every exit path.** On SIGINT/SIGTERM *and* on crashes, quidrac
  reverts to automatic control (or holds `--exit-speed` if you pass
  `--no-revert-on-exit`), retrying up to three times.
- **External backstop.** Nothing can catch SIGKILL or a power loss, so under
  systemd add a stop hook that reverts to automatic control no matter how the
  unit died — see below.

## Web dashboard

<img width="40%" src="https://github.com/user-attachments/assets/55a3ede7-32db-4527-a5d6-531d9bd4a532" />

Zero-dependency (stdlib HTTP server, fully inline page — works on an offline
management LAN), served on `--web-port` 8080 by default:

- Live charts of per-sensor temperatures, control temperature, target and
  hard-cap thresholds, and applied vs. curve-requested fan speed, with
  tooltips, 15m/1h/6h/24h ranges, a table view, and light/dark themes
  (~55 hours of history at the default poll interval).
- Status tiles and a header chip that shows `running` / `degraded` /
  `FAILSAFE` at a glance.
- **Every control parameter is editable live** — changes are validated, apply
  on the next poll, and persist (see below).

There is **no authentication**. Bind it to a trusted network only
(`--web-bind 127.0.0.1` for local-only), or disable it with `--no-web`.


## Settings precedence

Parameters resolve lowest → highest:

1. **Script defaults** — the `DEFAULT_*` constants at the top of the file
2. **Settings file** — `quidrac-settings.json` next to the script (or
   `--settings-file`), written automatically whenever you change a parameter
   in the dashboard
3. **Explicitly passed CLI flags** — typing a flag is an explicit request, so
   it outranks the saved file at startup

Dashboard changes always apply live, flags or not. On a restart, a flag that's
still on the command line re-pins its parameter; parameters not named by a
flag keep their dashboard-saved values. The **Restore script settings** button
returns to the launch configuration (defaults + flags) and deletes the file.

## Options

| Flag | Default | Description |
|---|---|---|
| `--host` / `--user` / `--password` | — | iDRAC connection. Prefer the `IPMI_PASSWORD` env var over `--password` (keeps it out of `ps`; it's passed to ipmitool via environment either way) |
| `--sensors` | `0Eh,0Fh` | Comma-separated sensor IDs to monitor |
| `--sensor-aliases` | from script | Friendly names, e.g. `"0Eh=CPU1,0Fh=CPU2"` |
| `--base-speed` | 30 | Minimum fan %, used at/below target temp |
| `--max-speed` | 100 | Maximum fan % |
| `--target-temp` | 65 | Curve start (°C) |
| `--hard-cap-temp` | 80 | Curve end (°C); at/above this, fans jump to max |
| `--fall-rate` | 2.0 | Max fan % decrease per poll when cooling |
| `--temp-hysteresis` | 2.0 | °C a temp must fall below its recent peak before fans follow (0 disables) |
| `--poll-interval` | 10 | Seconds between polls |
| `--max-failed-polls` | 5 | Failed polls before the failsafe reverts to automatic control |
| `--revert-on-exit` / `--no-revert-on-exit` | revert | On exit: hand control back to iDRAC, or hold `--exit-speed` |
| `--exit-speed` | 30 | Fan % to hold on exit with `--no-revert-on-exit` |
| `--settings-file` | next to script | Where dashboard changes are saved |
| `--web-bind` / `--web-port` / `--no-web` | `0.0.0.0:8080` | Dashboard exposure |
| `--demo` | off | Simulated server — preview the dashboard and tuning with no hardware |

## Running under systemd

```ini
[Unit]
Description=quidrac iDRAC fan control
After=network-online.target
Wants=network-online.target

[Service]
Environment=IPMI_PASSWORD=yourpassword
ExecStart=/usr/local/bin/quidrac.py --host 192.168.1.100 --user root
# Backstop: revert to automatic fan control however the unit stops.
ExecStopPost=/usr/bin/ipmitool -I lanplus -H 192.168.1.100 -U root -E \
    raw 0x30 0x30 0x01 0x01
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

(Consider `chmod 600` on the unit file, or use an `EnvironmentFile`, since it
contains the iDRAC password.)

## Disclaimer

quidrac drives your server's cooling. The defaults are conservative for
12th-gen dual-Xeon machines, but every chassis is different — watch your
temperatures after deploying (the dashboard makes this easy), and verify that
the failsafe hands control back by testing with the iDRAC unreachable. Use at
your own risk.
