#!/usr/bin/env python3
"""
cpcu_ipc_bridge.py — Python interface to /dev/shm/cpcu_ipc

Mirrors cpcu_ipc.h v2.1 binary layout exactly.
Uses mmap + struct for zero-copy shared memory access.

Author: bugrASl
Date:   April 2026

CRITICAL: If you change ANY struct size or field order in cpcu_ipc.h,
          you MUST update the offsets here AND run test_ipc_offsets.py.
"""

import mmap
import os
import struct
import time
import numpy as np

# ══════════════════════════════════════════════════════════════════════
#  CONSTANTS — must match cpcu_ipc.h exactly
# ══════════════════════════════════════════════════════════════════════

IPC_SHM_PATH            =   "/dev/shm/cpcu_ipc"
IPC_MAGIC               =   0x494E4654
IPC_VERSION             =   0x0201

RING_SIZE               =   1024
RING_MASK               =   RING_SIZE - 1
NUM_CHANNELS            =   8
SAMPLES_PER_PKT         =   2
NUM_SERVOS              =   6
MAX_GESTURE_NAME        =   16
MAX_CLASSES             =   10

# ══════════════════════════════════════════════════════════════════════
#  STRUCT SIZES — from _Static_assert in cpcu_ipc.h
# ══════════════════════════════════════════════════════════════════════

SZ_CTRL                 =   192         # 3 cache lines
SZ_ENTRY                =   64          # 1 cache line
SZ_MOTOR                =   128         # 2 cache lines
SZ_DIAG                 =   128         # 2 cache lines
SZ_EXPORT               =   256         # 4 cache lines
SZ_RING                 =   SZ_ENTRY * RING_SIZE    # 65536

# ══════════════════════════════════════════════════════════════════════
#  SECTION OFFSETS — sequential in shared memory
# ══════════════════════════════════════════════════════════════════════

OFF_CTRL                =   0
OFF_RING                =   OFF_CTRL + SZ_CTRL                  # 192
OFF_MOTOR               =   OFF_RING + SZ_RING                  # 65728
OFF_DIAG                =   OFF_MOTOR + SZ_MOTOR                # 65856
OFF_EXPORT              =   OFF_DIAG + SZ_DIAG                  # 65984
SHM_TOTAL               =   OFF_EXPORT + SZ_EXPORT              # 66240

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_ControlBlock (192 bytes)
# ══════════════════════════════════════════════════════════════════════
#  Cache line 0 (0-63): header
#    magic(4) + version(2) + io_ready(1) + dsp_ready(1)
#    + system_state(1) + pad(3) + heartbeat(8) + motor_cmd_ack(4) + reserved(36)
#  Cache line 1 (64-127): sensor_head(4) + pad(60)
#  Cache line 2 (128-191): sensor_tail(4) + pad(60)

CTRL_MAGIC              =   0
CTRL_VERSION            =   4
CTRL_IO_READY           =   6
CTRL_DSP_READY          =   7
CTRL_STATE              =   8
CTRL_HEARTBEAT          =   12          # uint64
CTRL_MOTOR_ACK          =   20          # uint32

# v2.3.4: edit-mode handshake bytes (live in the cache-line 0 reserve region)
CTRL_EDIT_REQUEST       =   24          # uint8 — TUI -> world
CTRL_EDIT_ACTIVE        =   25          # uint8 — io  -> TUI
CTRL_EDIT_DSP_ACK       =   26          # uint8 — dsp -> TUI (ack flag)
CTRL_EDIT_REQUEST_US    =   32          # uint64 (5B pad at 27..31 for align)

CTRL_HEAD               =   64          # cache line 1
CTRL_TAIL               =   128         # cache line 2

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_SensorEntry (64 bytes)
# ══════════════════════════════════════════════════════════════════════
#  samples[0]: 8 x uint16 = 16 bytes (offset 0)
#  samples[1]: 8 x uint16 = 16 bytes (offset 16)
#  seq(1) flags(1) tx_retry(1) pkt_loss(1) timestamp(2) vbat_raw(2) rx_time(8) pad(16)

ENTRY_SAMPLES           =   0           # 32 bytes total
ENTRY_SEQ               =   32
ENTRY_FLAGS             =   33
ENTRY_RETRY             =   34
ENTRY_LOSS              =   35
ENTRY_TIMESTAMP         =   36
ENTRY_VBAT              =   38
ENTRY_RXTIME            =   40

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_MotorCommand (128 bytes)
# ══════════════════════════════════════════════════════════════════════
#  seq(4) + servo_us[6](12) + gesture_id(1) + confidence(1) + pad(2)
#  + timestamp_us(8) + pad(28) + reserved(64)

MOTOR_SEQ               =   0
MOTOR_SERVO             =   4           # 6 x uint16
MOTOR_GESTURE           =   16
MOTOR_CONF              =   17
MOTOR_TIMESTAMP         =   20          # uint64 (after 2 bytes pad at 18)

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_Diagnostics (128 bytes)
# ══════════════════════════════════════════════════════════════════════

DIAG_PKTS               =   0
DIAG_DROPPED            =   4
DIAG_OVERFLOWS          =   8
DIAG_GAPS               =   12
DIAG_NRF_STATUS         =   16
DIAG_SAFE               =   20
DIAG_MAXPOLL            =   24
DIAG_BATCHES            =   28
DIAG_MAXLAT             =   32
DIAG_UNDERFLOWS         =   36
DIAG_INFERENCES         =   40

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_DSPExport (256 bytes)
# ══════════════════════════════════════════════════════════════════════
#  channel_rms[8](32) + gesture_name[16](16) + class_confidence[10](40)
#  + num_classes(1) + active_class(1) + pad(2) + inference_time_us(4)
#  + update_seq(4) + pad(132)

EXPORT_RMS              =   0           # 8 x float32 = 32 bytes
EXPORT_NAME             =   32          # 16 bytes
EXPORT_CONFIDENCE       =   48          # 10 x float32 = 40 bytes
EXPORT_NUM_CLASSES      =   88
EXPORT_ACTIVE_CLASS     =   89
EXPORT_INF_TIME         =   92          # uint32
EXPORT_UPDATE_SEQ       =   96          # uint32

# ══════════════════════════════════════════════════════════════════════
#  IPC STATE CONSTANTS
# ══════════════════════════════════════════════════════════════════════

IPC_STATE_INIT          =   0
IPC_STATE_RUNNING       =   1
IPC_STATE_SAFE          =   2


class IPCBridge:
    """
    Read/write interface to CPCU shared memory.
    
    Usage:
        ipc = IPCBridge()
        ipc.set_dsp_ready()
        entries = ipc.pop_sensor_batch(100)
        ipc.write_motor_cmd(servo_us, gesture_id, confidence)
        ipc.close()
    """

    def __init__(self, path=IPC_SHM_PATH):
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"Shared memory not found: {path}\n"
                f"Is cpcu_kernel running? (creates /dev/shm/cpcu_ipc)"
            )
        fd                  =   os.open(path, os.O_RDWR)
        self.mm             =   mmap.mmap(fd, SHM_TOTAL, mmap.MAP_SHARED)
        os.close(fd)
        self._verify_magic()

    def _verify_magic(self):
        magic               =   self._r32(OFF_CTRL + CTRL_MAGIC)
        version             =   self._r16(OFF_CTRL + CTRL_VERSION)
        if magic != IPC_MAGIC:
            raise RuntimeError(
                f"IPC magic mismatch: got 0x{magic:08X}, expected 0x{IPC_MAGIC:08X}"
            )
        if version != IPC_VERSION:
            print(f"[IPC] WARNING: version mismatch: got 0x{version:04X}, expected 0x{IPC_VERSION:04X}")

    # ── Primitive read/write helpers ──

    def _r8(self, off):         return self.mm[off]
    def _w8(self, off, val):    self.mm[off] = val & 0xFF
    def _r16(self, off):        return struct.unpack_from('<H', self.mm, off)[0]
    def _w16(self, off, val):   struct.pack_into('<H', self.mm, off, val)
    def _r32(self, off):        return struct.unpack_from('<I', self.mm, off)[0]
    def _w32(self, off, val):   struct.pack_into('<I', self.mm, off, val)
    def _r64(self, off):        return struct.unpack_from('<Q', self.mm, off)[0]
    def _w64(self, off, val):   struct.pack_into('<Q', self.mm, off, val)
    def _rf32(self, off):       return struct.unpack_from('<f', self.mm, off)[0]
    def _wf32(self, off, val):  struct.pack_into('<f', self.mm, off, val)

    # ══════════════════════════════════════════════════════════════════
    #  CONTROL BLOCK
    # ══════════════════════════════════════════════════════════════════

    def read_system_state(self):    return self._r8(OFF_CTRL + CTRL_STATE)
    def read_io_ready(self):        return self._r8(OFF_CTRL + CTRL_IO_READY)
    def read_dsp_ready(self):       return self._r8(OFF_CTRL + CTRL_DSP_READY)
    def set_dsp_ready(self):        self._w8(OFF_CTRL + CTRL_DSP_READY, 1)
    def read_heartbeat(self):       return self._r64(OFF_CTRL + CTRL_HEARTBEAT)

    # v2.3.4: Edit-mode handshake helpers.
    def read_edit_request(self):    return self._r8(OFF_CTRL + CTRL_EDIT_REQUEST)
    def read_edit_active(self):     return self._r8(OFF_CTRL + CTRL_EDIT_ACTIVE)
    def write_edit_request(self, v):
        # TUI is the sole writer of request + request_us.
        self._w8(OFF_CTRL + CTRL_EDIT_REQUEST, v)
        # Stamp request time so the TUI can implement its 500 ms timeout.
        if v:
            import time as _t
            self._w64(OFF_CTRL + CTRL_EDIT_REQUEST_US,
                      int(_t.monotonic_ns() // 1000))
    def read_edit_dsp_ack(self):    return self._r8(OFF_CTRL + CTRL_EDIT_DSP_ACK)
    def write_edit_dsp_ack(self, v):
        # cpcu_dsp.py acknowledges that it has seen the request and stopped
        # publishing motor commands. The TUI doesn't gate edit-active on
        # this — cpcu_io's SMOOTH_AllSettled is the authoritative gate —
        # but the ack is exposed in the diagnostic banner.
        self._w8(OFF_CTRL + CTRL_EDIT_DSP_ACK, v)

    def _read_head(self):           return self._r32(OFF_CTRL + CTRL_HEAD)
    def _read_tail(self):           return self._r32(OFF_CTRL + CTRL_TAIL)
    def _write_tail(self, val):     self._w32(OFF_CTRL + CTRL_TAIL, val)

    # ══════════════════════════════════════════════════════════════════
    #  RING BUFFER CONSUMER
    # ══════════════════════════════════════════════════════════════════

    def pop_sensor_batch(self, max_count=100):
        """
        Pop up to max_count entries from the SPSC ring buffer.
        
        Returns a dict with numpy arrays for efficient batch processing:
            {
                'count':     int,
                'samples':   np.ndarray (n, 2, 8) uint16,
                'seq':       np.ndarray (n,) uint8,
                'flags':     np.ndarray (n,) uint8,
                'tx_retry':  np.ndarray (n,) uint8,
                'pkt_loss':  np.ndarray (n,) uint8,
                'timestamp': np.ndarray (n,) uint16,
                'vbat_raw':  np.ndarray (n,) uint16,
            }
        
        WARNING: Only ONE process may call this (sole SPSC consumer).
        """
        tail                =   self._read_tail()
        head                =   self._read_head()
        avail               =   (head - tail) & 0xFFFFFFFF      # unsigned wrap

        if avail == 0:
            return {'count': 0}

        # Overflow detection: producer lapped us
        if avail > RING_SIZE:
            lost            =   avail - RING_SIZE
            tail           +=   lost
            avail           =   RING_SIZE

        n                   =   min(avail, max_count)

        # Pre-allocate numpy arrays for the whole batch
        samples             =   np.zeros((n, SAMPLES_PER_PKT, NUM_CHANNELS), dtype=np.uint16)
        seq_arr             =   np.zeros(n, dtype=np.uint8)
        flags_arr           =   np.zeros(n, dtype=np.uint8)
        retry_arr           =   np.zeros(n, dtype=np.uint8)
        loss_arr            =   np.zeros(n, dtype=np.uint8)
        ts_arr              =   np.zeros(n, dtype=np.uint16)
        vbat_arr            =   np.zeros(n, dtype=np.uint16)

        for i in range(n):
            idx             =   (tail + i) & RING_MASK
            base            =   OFF_RING + idx * SZ_ENTRY

            # Read 2 sample sets (each: 8 x uint16 = 16 bytes)
            for s in range(SAMPLES_PER_PKT):
                off         =   base + ENTRY_SAMPLES + s * NUM_CHANNELS * 2
                samples[i, s]   =   np.frombuffer(
                    self.mm, dtype='<u2', count=NUM_CHANNELS, offset=off
                ).copy()

            seq_arr[i]      =   self.mm[base + ENTRY_SEQ]
            flags_arr[i]    =   self.mm[base + ENTRY_FLAGS]
            retry_arr[i]    =   self.mm[base + ENTRY_RETRY]
            loss_arr[i]     =   self.mm[base + ENTRY_LOSS]
            ts_arr[i]       =   self._r16(base + ENTRY_TIMESTAMP)
            vbat_arr[i]     =   self._r16(base + ENTRY_VBAT)

        # Advance tail (we are the sole consumer)
        self._write_tail(tail + n)

        return {
            'count':        n,
            'samples':      samples,
            'seq':          seq_arr,
            'flags':        flags_arr,
            'tx_retry':     retry_arr,
            'pkt_loss':     loss_arr,
            'timestamp':    ts_arr,
            'vbat_raw':     vbat_arr,
        }

    def sensor_count(self):
        head    =   self._read_head()
        tail    =   self._read_tail()
        diff    =   (head - tail) & 0xFFFFFFFF
        return min(diff, RING_SIZE)

    # ══════════════════════════════════════════════════════════════════
    #  MOTOR COMMAND (SeqLock Writer)
    # ══════════════════════════════════════════════════════════════════

    def write_motor_cmd(self, servo_us, gesture_id, confidence):
        """
        Write motor command using SeqLock protocol.
        
        servo_us:       list/array of NUM_SERVOS uint16 pulse widths
        gesture_id:     uint8 classified gesture (0-9)
        confidence:     uint8 confidence percentage (0-100)
        """
        base                =   OFF_MOTOR

        # Step 1: seq -> odd (write in progress)
        seq                 =   self._r32(base + MOTOR_SEQ)
        self._w32(base + MOTOR_SEQ, seq + 1)

        # Step 2: write data
        for i in range(NUM_SERVOS):
            self._w16(base + MOTOR_SERVO + i * 2, int(servo_us[i]))
        self._w8(base + MOTOR_GESTURE, int(gesture_id))
        self._w8(base + MOTOR_CONF, int(confidence))
        ts                  =   int(time.monotonic() * 1_000_000) & 0xFFFFFFFFFFFFFFFF
        self._w64(base + MOTOR_TIMESTAMP, ts)

        # Step 3: seq -> even (write complete)
        self._w32(base + MOTOR_SEQ, seq + 2)

    # ══════════════════════════════════════════════════════════════════
    #  DIAGNOSTICS (atomic-ish increments)
    # ══════════════════════════════════════════════════════════════════

    def inc_dsp_inferences(self):
        off                 =   OFF_DIAG + DIAG_INFERENCES
        self._w32(off, self._r32(off) + 1)

    def update_dsp_max_latency(self, lat_us):
        off                 =   OFF_DIAG + DIAG_MAXLAT
        if lat_us > self._r32(off):
            self._w32(off, int(lat_us))

    def inc_dsp_batches(self, n):
        off                 =   OFF_DIAG + DIAG_BATCHES
        self._w32(off, self._r32(off) + n)

    def read_diagnostics(self):
        """Read all diagnostic counters as a dict."""
        b                   =   OFF_DIAG
        return {
            'io_pkts_received':     self._r32(b + DIAG_PKTS),
            'io_pkts_dropped':      self._r32(b + DIAG_DROPPED),
            'io_ring_overflows':    self._r32(b + DIAG_OVERFLOWS),
            'io_seq_gaps':          self._r32(b + DIAG_GAPS),
            'io_nrf_init_status':   self._r32(b + DIAG_NRF_STATUS),
            'dsp_batches':          self._r32(b + DIAG_BATCHES),
            'dsp_max_latency_us':   self._r32(b + DIAG_MAXLAT),
            'dsp_inferences':       self._r32(b + DIAG_INFERENCES),
        }

    # ══════════════════════════════════════════════════════════════════
    #  DSP EXPORT (Python -> TUI)
    # ══════════════════════════════════════════════════════════════════

    def write_dsp_export(self, channel_rms, gesture_name, class_confidence,
                         active_class, inference_time_us):
        """
        Write DSP telemetry for the TUI to display.
        
        channel_rms:        list/array of 8 floats (per-channel RMS)
        gesture_name:       string, max 15 chars (null-terminated)
        class_confidence:   list/array of up to 10 floats (per-class probability)
        active_class:       int (0-9)
        inference_time_us:  int (microseconds)
        """
        b                   =   OFF_EXPORT

        # Channel RMS: 8 x float32
        for i in range(min(len(channel_rms), NUM_CHANNELS)):
            self._wf32(b + EXPORT_RMS + i * 4, float(channel_rms[i]))

        # Gesture name: null-terminated string
        name_bytes          =   gesture_name[:MAX_GESTURE_NAME - 1].encode('ascii', errors='replace')
        name_bytes         +=   b'\x00' * (MAX_GESTURE_NAME - len(name_bytes))
        self.mm[b + EXPORT_NAME : b + EXPORT_NAME + MAX_GESTURE_NAME] = name_bytes

        # Class confidence: up to 10 x float32
        nc                  =   min(len(class_confidence), MAX_CLASSES)
        for i in range(nc):
            self._wf32(b + EXPORT_CONFIDENCE + i * 4, float(class_confidence[i]))

        self._w8(b + EXPORT_NUM_CLASSES, nc)
        self._w8(b + EXPORT_ACTIVE_CLASS, int(active_class) & 0xFF)
        self._w32(b + EXPORT_INF_TIME, int(inference_time_us))

        # Bump update sequence
        seq                 =   self._r32(b + EXPORT_UPDATE_SEQ)
        self._w32(b + EXPORT_UPDATE_SEQ, seq + 1)

    # ══════════════════════════════════════════════════════════════════
    #  CLEANUP
    # ══════════════════════════════════════════════════════════════════

    def close(self):
        if self.mm:
            self.mm.close()
            self.mm         =   None
