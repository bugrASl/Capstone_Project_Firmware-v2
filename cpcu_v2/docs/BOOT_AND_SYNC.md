# Boot and Sync — Cold-Start Choreography

**Author:** bugrASl
**Date:** April 2026
**Version:** v2.3.1 (introduced)
**Last updated:** v2.3.1
**Audience:** Anyone debugging "why did CPCU fault on boot?" or
"why do I have to power-cycle BSAU and CPCU in the right order?"

---

## TL;DR

Pre-v2.3.1, you had to power on CPCU **after** BSAU was already
transmitting (or hold BSAU in reset until CPCU was ready) to avoid
the system declaring a radio fault during the first second of boot.
This was annoying and led to the workaround of holding the BSAU
reset button while booting CPCU, then releasing it.

v2.3.1 adds a **5 second cold-start grace period** to the radio
timeout. You can now power on CPCU and BSAU **in any order** with
several seconds of slack between them, and the system reaches
steady state without ever spuriously entering SAFE.

The grace doesn't compromise safety: if BSAU is genuinely dead
(unconnected, dead battery, broken NRF), CPCU still flags the radio
fault — just 5 seconds later than during normal operation. That
delay is acceptable for cold-boot, unacceptable for runtime drops.

---

## What "sync" means in this system

The BSAU and CPCU are two independent processors with no direct wire
connection. Their only shared channel is the 2.4 GHz NRF link. So
"sync" doesn't mean clock synchronization or handshake negotiation —
it means **they have to start agreeing about packet ordering and
timing without coordinating their boot moments.**

There are three sync facets to think about, each handled by a
different mechanism:

### Facet 1 — Sequence number alignment

The BSAU emits a `seq` byte in every packet, incrementing 0..255 then
wrapping. CPCU keeps an `expected_seq` and reports a "gap" whenever
the received seq doesn't match.

**Problem on cold start:** When CPCU starts after BSAU has already
been emitting packets, CPCU's `expected_seq` starts at 0 but the
incoming packet might have seq=137. That's a "gap" of 137 packets
which would (a) flood the gap counter, (b) trigger the seq-gap-storm
fault if the gap is big enough.

**Mechanism:** BSAU sets `WL_FLAG_FIRST_PACKET` in the flags byte of
its very first packet after boot (and after any NRF recovery).
CPCU's `SAFETY_FeedPacket` checks that flag and, when set, copies
the incoming `seq` into `expected_seq` directly — initializing the
counter from the BSAU's perspective rather than from CPCU's
arbitrary starting value.

**Result:** No spurious gap on the first packet, regardless of
power-on ordering.

This was already in place before v2.3.1.

### Facet 2 — Radio fault timeout

CPCU watches the elapsed time since the last received packet
(`SAFETY_CheckTimeout`). If silence exceeds `SAFETY_RADIO_TIMEOUT_MS`
(750 ms), the radio is declared faulty and the FSM transitions
RUNNING → DEGRADED → SAFE.

**Problem on cold start:** If CPCU boots before BSAU, the silence
counter starts ticking from CPCU's startup. If BSAU takes longer
than 750 ms to reach steady state and start transmitting (which is
normal — STM32 boot + NRF init takes ~600 ms by itself, plus any
delay from the user pressing the reset button), CPCU declares a
spurious radio fault and parks the arm at neutral. Then when BSAU
finally starts transmitting, CPCU has to wait through the 3 second
SAFE-recovery hold before it can use the data.

**Mechanism (v2.3.1):** A new "boot grace" gate in
`SAFETY_CheckTimeout`. The radio timeout is suppressed if both:

  - No packet has ever been received (`first_packet_seen == false`)
  - The boot grace period has not yet elapsed
    (`now - boot_us < SAFETY_RADIO_BOOT_GRACE_MS`)

Once either condition flips, normal timeout semantics resume.

**Result:** The first 5 seconds after CPCU boot are tolerant of
silence. After that, OR after the first received packet (whichever
comes first), normal 750 ms timeout applies.

### Facet 3 — Battery state cold-start

CPCU's safety FSM tracks battery hysteresis (low / critical /
recover) based on the `vbat_raw` field in each packet. With no
packets, there's no battery reading, so the safety check defaults to
"battery OK".

**Problem:** No problem in practice. If BSAU eventually starts
transmitting with a critical battery, the next FeedPacket transitions
the FSM into the battery fault. There's no race window here because
the absence of battery data is treated as no-fault (rather than
defaulting to "assume the worst").

**Mechanism:** None needed. The hysteresis design treats the absence
of new data as "no change", so battery state simply waits for first
data.

---

## Why 5 seconds and not 1 second or 30 seconds

### Lower bound: 3 seconds

BSAU's reset → first transmit takes about 600 ms in the well-behaved
case. With v2.4's bounded-retry NRF init, a marginal radio rail can
add another ~400 ms (200 ms POR delay + 100 ms backoff × 2 retries).
That's already 1 second consumed by BSAU's normal boot sequence.

If you're powering on with a switch (rather than carefully
coordinating reset releases), there's an additional human reaction
time of ~500 ms between flipping CPCU on and remembering to flip
BSAU on.

3 seconds gives a comfortable margin without making the user wait.

### Upper bound: ~10 seconds

Two interactions push back against making this much longer:

**`SAFETY_AUTO_CLEAR_MS` is 300 seconds (5 minutes).** If a fault
clears, the cumulative diagnostic counters auto-zero. With a grace
of 5 seconds, the system is well clear of any auto-clear corner case.
A grace of 30+ seconds starts feeling like "the fault detection is
just slow on boot" rather than "we're waiting for sync".

**Genuinely-dead BSAU diagnosis.** If BSAU is broken (NRF chip dead,
power rail failed, MCU not booted), CPCU should flag the problem.
Today it does so 750 ms after BSAU stops transmitting; with 5 second
grace, the very first detection takes 5 seconds 750 ms. Acceptable.
With 30 seconds grace, you'd be looking at the dashboard for half a
minute thinking "everything's fine" while it actually isn't.

5 seconds is the sweet spot: long enough to absorb realistic boot
timing variance, short enough that genuine boot failures are flagged
within 6 seconds.

### Why it's a `#define`, not a runtime config

Like all safety thresholds (`SAFETY_RADIO_TIMEOUT_MS`,
`SAFETY_VBAT_CRITICAL_V`, etc.), the grace period defines what
"safe" means. Live-tunable safety thresholds are a recipe for a
misclick disabling protection. So this lives in `cpcu_safety.h`
where it can only be changed via `configure.sh` (when that lands in
v2.3.3) or by directly editing the header and rebuilding.

---

## Implementation details

### Code change footprint

Three small additions:

`cpcu_v2/include/cpcu_safety.h`:
```c
#define SAFETY_RADIO_BOOT_GRACE_MS      5000
```
And two new fields in `SAFETY_Context`:
```c
uint64_t        boot_us;
bool            first_packet_seen;
```

`cpcu_v2/src/cpcu_safety.c`, `SAFETY_Init`:
```c
ctx->boot_us = safety_now_us();
/* memset above already cleared first_packet_seen */
```

`cpcu_v2/src/cpcu_safety.c`, `SAFETY_FeedPacket`:
```c
ctx->first_packet_seen = true;     /* added at top */
```

`cpcu_v2/src/cpcu_safety.c`, `SAFETY_CheckTimeout`:
```c
if(!ctx->first_packet_seen &&
   (now_us - ctx->boot_us) / 1000 < SAFETY_RADIO_BOOT_GRACE_MS)
{
    return;
}
/* ...existing timeout logic unchanged... */
```

That's the entire mechanism. No changes to cpcu_io.c, no changes to
the IPC layout, no changes to the BSAU side.

### Why `first_packet_seen` is separate from `seq_init`

`seq_init` already exists; it's set when a packet with
`WL_FLAG_FIRST_PACKET` is seen. Couldn't we reuse it for the grace
gate?

No, for two reasons:

1. **A BSAU restart re-asserts FIRST_PACKET.** After v2.4's NRF
   recovery, the BSAU sends a new FIRST_PACKET-flagged packet, which
   would re-set `seq_init`. If the grace gate used `seq_init`, every
   NRF recovery would re-enter grace mode — defeating the whole
   point of the timeout for any BSAU restart after the first one.

2. **`first_packet_seen` is intentionally a "we've ever heard from
   them" flag, not a "they've identified themselves" flag.** A bug
   in BSAU that drops the FIRST_PACKET flag should still let the
   grace expire on first packet receipt. Decoupling them keeps the
   grace logic robust.

### Compatibility

- **No public API change.** No new arguments, no removed functions.
  Existing callers compile and link unchanged.
- **No IPC schema change.** The new fields live in the in-process
  SAFETY_Context, which is private to cpcu_safety.c (and tests).
- **No test regressions.** Existing tests go through `warm_up`
  which calls `SAFETY_FeedPacket` 20 times, setting
  `first_packet_seen = true` long before any `CheckTimeout` call.
- **Profile-independent on BSAU side.** No BSAU code touched. All
  BSAU profiles (RELEASE / DEBUG / DATASET / TEST_*) emit packets
  with FIRST_PACKET set the same way; CPCU's grace gate doesn't care
  which profile is on the other end.

### Interaction with existing safety mechanisms

- **NRF recovery (BSAU v2.4):** When BSAU recovers its NRF mid-session
  and resumes transmitting, `first_packet_seen` is already true on
  the CPCU side, so the grace gate is a no-op. The radio timeout
  uses normal 750 ms semantics. Recovery feels identical to v2.3.0.
- **Ring overflow recovery (CPCU v2.3):** Independent of grace.
  Different fault, different recovery timer.
- **SAFE auto-recovery (`SAFETY_SAFE_RECOVER_MS = 3 s`):** Unchanged.
  If the system *does* enter SAFE (e.g. after a real fault), the
  3-second hold-clear-recover semantics still apply.
- **Auto-clear of cumulative counters (`SAFETY_AUTO_CLEAR_MS =
  300 s`):** Unchanged. Grace expires long before auto-clear is
  relevant.

---

## Testing

`safety_testbench` gained a TB-SAF09 group with four sub-checks:

| Test | Setup | Expected |
|---|---|---|
| TB-SAF09a | RUNNING + 2 s of silence inside grace | stays RUNNING |
| TB-SAF09b | RUNNING + grace expired with no packets | transitions to DEGRADED |
| TB-SAF09c | First packet during grace, then 800 ms silence | normal timeout fires (DEGRADED) |
| TB-SAF09d | Post-warmup timeout (regression test) | unchanged 750 ms behaviour |

Run from `cpcu_v2/`:
```bash
build/safety_testbench
# Expected: 38 PASS, 0 FAIL  (was 33 pre-v2.3.1)
```

Or via the full Phase 1:
```bash
./launch.sh test
# Expected: 7 + 38 + 65 = 110 PASS
```

---

## Operating procedure (the practical impact)

### Before v2.3.1

```
Power on CPCU → wait for kernel to start → quickly press BSAU reset
  → release reset within 750 ms → both running together
  → if you're slow, system enters SAFE for 3 seconds
```

The "manual reset dance" was annoying for development and
unacceptable for a real prosthetic.

### After v2.3.1

```
Power on either side. Wait. Power on the other side. Done.
```

Three valid orderings, all handled identically:

- **CPCU first**: CPCU boots, sees no packets, sits in grace, waits.
  BSAU boots later, transmits, CPCU lifts grace, normal operation.
- **BSAU first**: BSAU emits packets to nobody. CPCU boots, immediately
  starts receiving, lifts grace, normal operation.
- **Together**: Whichever finishes initializing first; the
  late-starter joins normally.

### Edge case: BSAU is dead

```
Power on CPCU. Wait 5 seconds. Radio fault triggers. Arm parks at
neutral. TUI shows RADIO_SAFE in red. Diagnostics show "no packet
received".
```

This is the *correct* behaviour. The grace doesn't hide failures; it
just gives them a 5-second sync window before flagging.

### Edge case: BSAU starts late

```
Power on CPCU at t=0. BSAU starts at t=20s.
- t=0 to t=5s:    grace active, sits in RUNNING.
- t=5s to t=5.75s: grace expired, last_pkt_rcv_us = boot_us, silence
                   already huge → fault.
- t=5.75s:        DEGRADED.
- t=7.25s:        SAFE (DEGRADED + 1.5s).
- t=20s:          BSAU starts transmitting.
- t=20s+ε:        FeedPacket lifts first_packet_seen, but state ==
                   SAFE so we wait for SAFE_RECOVER.
- t=20s + 3s:     SAFE → RUNNING via normal recovery path.
```

So a 20-second-late BSAU costs about 3 seconds of recovery, which
matches the normal "we lost the radio for a while" recovery time.
Reasonable.

---

## See also

- [`ARCHITECTURE.md`](ARCHITECTURE.md) §6 — full safety FSM
- [`cpcu_safety.h`](../include/cpcu_safety.h) — header with the new constant
- [`cpcu_safety.c`](../src/cpcu_safety.c) — `SAFETY_CheckTimeout` and `SAFETY_FeedPacket`
- [`safety_testbench.c`](../test/safety_testbench.c) — TB-SAF09
- BSAU v2.4 NRF non-fatal init (`bsau_v2/docs/BSAU_ARCHITECTURE.md` §7) — the
  other half of "make the system tolerant of cold-start oddities"
