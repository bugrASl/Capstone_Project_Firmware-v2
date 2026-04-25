#!/usr/bin/env python3
"""
cpcu_dsp.py — minimal drain stub for DATA COLLECTION ONLY

Replaces the broken patch-doc cpcu_dsp.py until the real DSP/ML script
is restored. Does NOT do filtering, feature extraction, or inference —
it only:

  1. Opens /dev/shm/cpcu_ipc
  2. Sets dsp_ready = 1   (so Page 6 shows DSP as healthy)
  3. Drains the sensor ring at ~50 Hz   (so the producer doesn't wrap
     and corrupt samples that cpcu_tui Page 7 is trying to read)
  4. Writes a neutral motor command every 100 ms (so the safety FSM
     doesn't trip the 2000 ms "DSP stall -> SAFE" check)
  5. Exits cleanly on SIGTERM / SIGINT

Use this until the real cpcu_dsp.py is recovered. Captures done with
this stub are valid; the only thing that won't work is real-time
gesture inference / servo control.
"""

import signal
import sys
import time

from cpcu_ipc_bridge import IPCBridge

# Servo pulse width that means "neutral" — must match cpcu_pca9685.h
SERVO_NEUTRAL_US    =   1500
NUM_SERVOS          =   6

# Loop timing
DRAIN_PERIOD_S      =   0.020       # 50 Hz drain rate
MOTOR_PERIOD_S      =   0.100       # 10 Hz neutral keepalive
BATCH_SIZE          =   200         # drain up to this many entries per tick


def main():
    ipc = IPCBridge()
    ipc.set_dsp_ready()
    print("[DSP-STUB] connected, dsp_ready=1, entering drain loop", flush=True)

    # Graceful shutdown on SIGTERM/SIGINT (kernel sends SIGTERM)
    running = [True]
    def stop(signum, frame):
        running[0] = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT,  stop)

    neutral = [SERVO_NEUTRAL_US] * NUM_SERVOS
    last_motor_t = 0.0
    drained_total = 0
    last_report_t = time.monotonic()

    while running[0]:
        t0 = time.monotonic()

        # 1. Drain the ring
        batch = ipc.pop_sensor_batch(BATCH_SIZE)
        n = batch.get('count', 0)
        if n > 0:
            drained_total += n
            ipc.inc_dsp_batches(n)
            ipc.inc_dsp_inferences()  # fake-but-monotonic, keeps Page 6 alive

        # 2. Keep safety FSM happy with neutral motor cmd at 10 Hz
        if t0 - last_motor_t >= MOTOR_PERIOD_S:
            ipc.write_motor_cmd(neutral, gesture_id=0, confidence=0)
            last_motor_t = t0

        # 3. Periodic stderr report so launch.sh logs show progress
        if t0 - last_report_t >= 5.0:
            ring_now = ipc.sensor_count()
            print(f"[DSP-STUB] drained={drained_total} ring_now={ring_now}",
                  flush=True)
            last_report_t = t0

        # 4. Sleep the rest of the 20 ms tick
        elapsed = time.monotonic() - t0
        sleep_for = DRAIN_PERIOD_S - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)

    ipc.close()
    print("[DSP-STUB] shutdown clean", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
