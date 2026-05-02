#!/usr/bin/env python3
"""
smoother_scenario.py — On-hardware trapezoidal profile exerciser.

Writes motor commands through IPC shared memory so cpcu_io's smoother
handles the interpolation. You see the REAL servo motion with your
current velocity / accel / deadband settings from runtime.json.

Requires: cpcu_kernel + cpcu_io running (./launch.sh tui, or ./launch.sh kernel).

Usage:
    python3 smoother_scenario.py                    # interactive menu
    python3 smoother_scenario.py --servo 0 --scenario sweep
    python3 smoother_scenario.py --servo all --scenario step
    python3 smoother_scenario.py --list              # show all scenarios

Author: bugrASl
Date:   May 2026
"""

import sys
import os
import time
import argparse
import signal

# ── Path setup (same pattern as cpcu_dsp.py) ──────────────────────────
_SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
_ALT_DIR     = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "python"))
for _p in (_SCRIPT_DIR, _ALT_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from cpcu_ipc_bridge import (
    IPCBridge, NUM_SERVOS, OFF_CTRL, CTRL_STATE,
)

# ── Constants ─────────────────────────────────────────────────────────

TICK_HZ  = 50           # motor command rate (matches cpcu_io servo tick)
TICK_S   = 1.0 / TICK_HZ

IPC_STATE_RUNNING = 1

# ── Servo definitions ─────────────────────────────────────────────────

SERVOS = [
    {"id": 0, "name": "S0 Base",    "type": "MG995", "min": 498,  "max": 2500, "neutral": 1500},
    {"id": 1, "name": "S1 Upper",   "type": "MG995", "min": 1074, "max": 1953, "neutral": 1500},
    {"id": 2, "name": "S2 Last",    "type": "MG995", "min": 1074, "max": 1953, "neutral": 1500},
    {"id": 3, "name": "S3 Joint-1", "type": "SG90",  "min": 1001, "max": 2002, "neutral": 1500},
    {"id": 4, "name": "S4 Joint-2", "type": "SG90",  "min": 1001, "max": 2002, "neutral": 1500},
    {"id": 5, "name": "S5 Gripper", "type": "SG90",  "min": 976,  "max": 1733, "neutral": 1350},
]

# ── Scenarios ─────────────────────────────────────────────────────────

def _mid(s):
    return (s["min"] + s["max"]) // 2

SCENARIOS = {
    "step": {
        "desc": "Step response: neutral -> max -> neutral (classic control test)",
        "waypoints": lambda s: [
            (1.0, s["neutral"]),
            (2.5, s["max"]),
            (2.5, s["neutral"]),
        ],
    },
    "sweep": {
        "desc": "Full sweep: neutral -> min -> max -> neutral",
        "waypoints": lambda s: [
            (1.0, s["neutral"]),
            (2.0, s["min"]),
            (2.0, s["max"]),
            (2.0, s["neutral"]),
        ],
    },
    "triangle": {
        "desc": "Triangle wave: mid -> max -> min -> max -> mid (2 cycles)",
        "waypoints": lambda s: [
            (0.5, _mid(s)),
            (1.5, s["max"]),
            (1.5, s["min"]),
            (1.5, s["max"]),
            (1.5, _mid(s)),
        ],
    },
    "small_step": {
        "desc": "Small step: +/-50 us around neutral (tests deadband)",
        "waypoints": lambda s: [
            (1.0, s["neutral"]),
            (2.0, s["neutral"] + 50),
            (2.0, s["neutral"] - 50),
            (2.0, s["neutral"]),
        ],
    },
    "burst": {
        "desc": "Rapid burst: fast min/max toggles (stress test)",
        "waypoints": lambda s: [
            (0.5, s["neutral"]),
            (0.8, s["max"]),
            (0.8, s["min"]),
            (0.8, s["max"]),
            (0.8, s["min"]),
            (1.0, s["neutral"]),
        ],
    },
    "slow_creep": {
        "desc": "Slow creep: neutral -> neutral+200 (see acceleration ramp)",
        "waypoints": lambda s: [
            (1.0, s["neutral"]),
            (4.0, min(s["neutral"] + 200, s["max"])),
            (2.0, s["neutral"]),
        ],
    },
}

# ── Runner ────────────────────────────────────────────────────────────

g_run = True

def _on_sig(signum, frame):
    global g_run
    g_run = False

def run_scenario(ipc, servo_idx, scenario_name, quiet=False):
    """Run one scenario on one servo. Other servos held at neutral."""
    s       = SERVOS[servo_idx]
    sc      = SCENARIOS[scenario_name]
    wps     = sc["waypoints"](s)

    total_time = sum(w[0] for w in wps)

    if not quiet:
        print(f"\n{'─' * 60}")
        print(f"  Servo:    {s['name']} ({s['type']})")
        print(f"  Range:    {s['min']} – {s['max']} us")
        print(f"  Scenario: {scenario_name} — {sc['desc']}")
        print(f"  Duration: {total_time:.1f} s  ({len(wps)} waypoints)")
        print(f"  Tick:     {TICK_HZ} Hz (matching cpcu_io servo tick)")
        print(f"{'─' * 60}")
        print(f"  {'time':>6s}  {'target':>7s}  {'phase':>20s}")
        print(f"  {'─'*6}  {'─'*7}  {'─'*20}")

    # Build timeline: list of (abs_time, target_us)
    timeline = []
    t_acc = 0.0
    for hold_s, target in wps:
        timeline.append((t_acc, target))
        t_acc += hold_s

    # Neutral baseline for all servos
    targets = [sv["neutral"] for sv in SERVOS]

    t_start = time.monotonic()
    wp_idx  = 0
    tick    = 0
    last_print = 0.0

    while g_run:
        t_now   = time.monotonic()
        elapsed = t_now - t_start

        if elapsed > total_time + 1.0:
            break

        # Advance waypoints
        while wp_idx < len(timeline) - 1 and elapsed >= timeline[wp_idx + 1][0]:
            wp_idx += 1

        current_target = timeline[wp_idx][1]
        targets[servo_idx] = current_target

        # Write motor command through IPC bridge
        ipc.write_motor_cmd(targets, gesture_id=0, confidence=99)

        # Print at ~4 Hz
        if not quiet and (t_now - last_print) >= 0.25:
            last_print = t_now
            phase = f"wp {wp_idx}/{len(timeline)-1}"
            if wp_idx < len(timeline) - 1:
                remaining = timeline[wp_idx + 1][0] - elapsed
                if remaining > 0:
                    phase += f" (hold {remaining:.1f}s)"

            bar_len = 30
            frac = (current_target - s["min"]) / max(1, s["max"] - s["min"])
            frac = max(0.0, min(1.0, frac))
            filled = int(frac * bar_len)
            bar = "#" * filled + "." * (bar_len - filled)

            print(f"  {elapsed:6.2f}  {current_target:5d}us  [{bar}] {phase}")

        # Sleep until next tick
        tick += 1
        next_t = t_start + tick * TICK_S
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)

    # Return to neutral
    targets[servo_idx] = s["neutral"]
    for _ in range(10):
        ipc.write_motor_cmd(targets, gesture_id=0, confidence=99)
        time.sleep(TICK_S)

    if not quiet:
        print(f"  {'DONE':>6s}  {s['neutral']:5d}us  (returned to neutral)")
        print()

# ── Interactive menu ──────────────────────────────────────────────────

def interactive(ipc):
    print()
    print("+" + "=" * 58 + "+")
    print("|     SMOOTHER SCENARIO RUNNER — on-hardware profiler     |")
    print("+" + "=" * 58 + "+")
    print()
    print("  This tool writes motor commands through IPC so cpcu_io's")
    print("  smoother handles the interpolation. You see the REAL")
    print("  servo motion with your current runtime.json settings.")
    print()
    print("  To tune: edit velocity/accel/deadband in the TUI editor")
    print("  (CONFIG page -> 'e'), Ctrl+S to save, then re-run a")
    print("  scenario here to see the difference.")
    print()

    state = ipc._r8(OFF_CTRL + CTRL_STATE)
    state_str = {0: "INIT", 1: "RUNNING", 2: "SAFE"}.get(state, f"?({state})")
    if state != IPC_STATE_RUNNING:
        print(f"  WARNING: System state is {state_str} (expected RUNNING).")
        print(f"           Servo commands may be ignored by cpcu_io.")
        print()

    while g_run:
        print("  SERVOS:")
        for s in SERVOS:
            print(f"    {s['id']}) {s['name']:14s}  {s['type']}  "
                  f"({s['min']}-{s['max']} us)")
        print(f"    a) ALL servos (one at a time)")
        print(f"    q) quit")
        print()

        choice = input("  Select servo [0-5/a/q]: ").strip().lower()
        if choice == "q":
            break
        if choice == "a":
            servo_list = list(range(NUM_SERVOS))
        elif choice.isdigit() and 0 <= int(choice) < NUM_SERVOS:
            servo_list = [int(choice)]
        else:
            print("  Invalid choice.\n")
            continue

        print()
        print("  SCENARIOS:")
        sc_names = list(SCENARIOS.keys())
        for i, name in enumerate(sc_names):
            print(f"    {i}) {name:14s} -- {SCENARIOS[name]['desc']}")
        print(f"    a) ALL scenarios (sequential)")
        print()

        sc_choice = input(f"  Select scenario [0-{len(sc_names)-1}/a]: ").strip().lower()
        if sc_choice == "a":
            sc_list = sc_names
        elif sc_choice.isdigit() and 0 <= int(sc_choice) < len(sc_names):
            sc_list = [sc_names[int(sc_choice)]]
        else:
            print("  Invalid choice.\n")
            continue

        print()
        input("  Press ENTER to start (Ctrl+C to abort)...")

        for si in servo_list:
            for sc in sc_list:
                if not g_run:
                    break
                run_scenario(ipc, si, sc)

        if g_run:
            print("  All scenarios complete.\n")

# ── Main ──────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="On-hardware smoother scenario runner. "
                    "Exercises servos through the cpcu_io smoother pipeline.",
        epilog="Requires cpcu_kernel + cpcu_io running. "
               "Start with: ./launch.sh tui (or ./launch.sh kernel)")
    ap.add_argument("--servo", default=None,
                    help="Servo index 0-5, or 'all' (default: interactive)")
    ap.add_argument("--scenario", default=None,
                    help="Scenario name (default: interactive)")
    ap.add_argument("--list", action="store_true",
                    help="List all scenarios and exit")
    args = ap.parse_args()

    if args.list:
        print("\nAvailable scenarios:\n")
        for name, sc in SCENARIOS.items():
            print(f"  {name:14s} -- {sc['desc']}")
        print()
        return 0

    try:
        ipc = IPCBridge()
    except FileNotFoundError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        print(f"  Start the system with: ./launch.sh tui", file=sys.stderr)
        return 1
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    try:
        if args.servo is not None and args.scenario is not None:
            if args.servo.lower() == "all":
                servo_list = list(range(NUM_SERVOS))
            else:
                servo_list = [int(args.servo)]

            if args.scenario not in SCENARIOS:
                print(f"ERROR: unknown scenario '{args.scenario}'. "
                      f"Use --list to see options.", file=sys.stderr)
                return 1

            for si in servo_list:
                if not g_run:
                    break
                run_scenario(ipc, si, args.scenario)
        else:
            interactive(ipc)
    finally:
        neutrals = [sv["neutral"] for sv in SERVOS]
        for _ in range(5):
            ipc.write_motor_cmd(neutrals, gesture_id=0, confidence=99)
            time.sleep(TICK_S)
        ipc.close()

    return 0

if __name__ == "__main__":
    sys.exit(main())
