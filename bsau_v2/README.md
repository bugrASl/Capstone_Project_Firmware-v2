# BSAU — Bio-Signal Acquisition Unit

[![Platform: STM32L432KC](https://img.shields.io/badge/Platform-STM32L432KC-303030.svg)](#hardware)
[![Toolchain: STM32CubeIDE](https://img.shields.io/badge/Toolchain-STM32CubeIDE-blue.svg)](https://www.st.com/en/development-tools/stm32cubeide.html)
[![Version: v2.4](https://img.shields.io/badge/Version-v2.4-brightgreen.svg)](#)

**The transmitter half of the prosthetic hand system.** A
Nucleo-32 STM32L432KC that samples 8 EMG channels at 2 kHz, packs
them into 32-byte wireless frames, and transmits 1000 packets per
second over a 2.4 GHz NRF24L01+ link to the CPCU on a Raspberry Pi 5.

> **First time with this repo?** The whole-system walkthrough lives
> at [`../SYSTEM_GUIDE.md`](../SYSTEM_GUIDE.md). This README is the
> BSAU-specific quick reference.

---

## What this side does

```
PA0..PA7  →  ADC1 (12-bit, 32× OS, 2 kHz scan)  →  DMA double-buffer
                                                        ↓
   PB0   →  IN15 (battery, post 2:1 divider) ─────────┐
                                                        ↓
                                            BSAU_Run main loop
                                                        ↓
                                      WL_Pack (32-byte frame)
                                                        ↓
                                NRF24L01+ via SPI1 (5 MHz, 2 Mbps air)
                                                        ↓
                                                    CPCU
```

**Key numbers:**

- 8 EMG channels (PA0–PA7) + 1 battery sense (PB0) = 9 channels per scan
- 2 samples per packet → 1000 pkt/s at 2 kHz scan rate
- 32× hardware oversampling → ~14.5 ENOB
- 2 Mbps air rate (was 250 kbps in v1) → 54% radio idle margin
- 32-byte payload, channel 76, address `E7:E7:E7:E7:E7`

---

## Build flavours

BSAU has four compile-time profiles, selected by exactly one
`#define BSAU_MODE_*` in `Core/Inc/bsau_config.h`:

| Profile | Behaviour |
|---|---|
| `BSAU_MODE_RELEASE` | Production. ADC + radio. UART silent. Lowest latency. |
| `BSAU_MODE_DEBUG`   | ADC + radio + verbose UART logs (LOG_I/W/E/D). Use during bring-up. |
| `BSAU_MODE_DATASET` | ADC + radio + simultaneous CSV stream over UART. The CSV stream is non-blocking and never starves the radio path; controlled by `BSAU_DATASET_CSV_DECIMATION`. |
| `BSAU_MODE_TEST_*`  | One of several `BSAU_MODE_TEST_PKT_LOG`, `BSAU_MODE_TEST_NRF`, `BSAU_MODE_TEST_DFT`, `BSAU_MODE_TEST_RAW_FAST`, `BSAU_MODE_TEST_RAW_SLOW` — bring-up testbenches. See `BSAU_TEST_GUIDE.md`. |

To switch: edit `bsau_config.h`, comment out the current
`#define BSAU_MODE_*`, uncomment the one you want, rebuild, re-flash.

---

## v2.4 — NRF init non-fatal

**The big v2.4 change.** Pre-v2.4, a failed `NRF_Init()` (radio
unreachable, brownout, dead chip) called `Error_Handler()` — which
disables interrupts and spins forever. Result: a bad-rail or dead-NRF
board would lock up at boot, even though the ADC was fine and the UART
was fine.

v2.4 introduces a `g_nrf_alive` flag and three file-local helpers:

| Helper | Role |
|---|---|
| `nrf_bringup()`     | Bounded-retry `NRF_Init`. Updates `g_nrf_alive`. |
| `nrf_try_recover()` | Cold recovery: drain FIFOs → power-cycle → bringup. |
| `nrf_is_healthy()`  | Sanity-check via `RF_CH` / `CONFIG` register read-back. |

The TX path is gated by `if (g_nrf_alive)` — uniform across all four
profiles, so DATASET, RELEASE, and DEBUG all behave identically when
the radio is dead. Every `NRF_HEALTH_CHECK_INTERVAL = 500` packets,
`BSAU_Run` calls `nrf_is_healthy()`; if it fails, `nrf_try_recover()`
fires once. After a successful recovery, `prev_loss` and `prev_retry`
are zeroed so stale `OBSERVE_TX` values don't leak into the first new
packet.

**End user effect:** a board with a sagging radio rail boots, keeps
ADC + UART alive (and the DATASET CSV stream flowing), and picks up
the radio link as soon as it becomes reachable.

Full design doc: [`docs/BSAU_ARCHITECTURE.md` §7](docs/BSAU_ARCHITECTURE.md).

---

## Build & flash

Open `InfiniTech_BSAU_Skeleton_v1.0.ioc` in STM32CubeIDE. Build / Run
buttons handle everything (ST-Link auto-detected over the on-board
USB connector of the Nucleo-32).

Wireless packet format (shared with CPCU verbatim — same .c source
file is linked into both binaries):

```
Byte    Field       Size    Description
[0]     seq         1 B     Sequence number (0–255, wraps)
[1]     flags       1 B     Status flags + 2-bit battery level
[2]     tx_retry    1 B     NRF ARC_CNT (from previous TX)
[3]     pkt_loss    1 B     NRF PLOS_CNT (cumulative, saturates at 15)
[4–5]   timestamp   2 B     TIM2 µs counter, little-endian
[6–7]   vbat_raw    2 B     12-bit battery ADC (high-nibble aligned)
[8–19]  sample[0]   12 B    8 channels × 12-bit packed
[20–31] sample[1]   12 B    8 channels × 12-bit packed
```

Flag bits: `FIRST_PACKET`, `CLIPPING`, `ELEC_OFF`, `ADC_OVRN`,
`TX_SAT`, `CAL` + 2-bit battery level (`OK / LOW / CRIT / CHARG`).

---

## Documentation

- **[`docs/BSAU_ARCHITECTURE.md`](docs/BSAU_ARCHITECTURE.md)** (v2.4)
  — every BSAU design decision: ADC pipeline, DMA double-buffer
  strategy, NRF non-fatal init flow, packet format derivation, profile
  matrix.
- **[`docs/BSAU_RUN_GUIDE.md`](docs/BSAU_RUN_GUIDE.md)** (v2.4) —
  bring-up, profile selection, DATASET workflow, troubleshooting.
- **[`docs/BSAU_TEST_GUIDE.md`](docs/BSAU_TEST_GUIDE.md)** (v2.4) —
  TB-100..TB-309 procedures + Part B testbench reference.

For the whole-system walkthrough (both BSAU and CPCU together):
[`../SYSTEM_GUIDE.md`](../SYSTEM_GUIDE.md).
For the receiver side: [`../cpcu_v2/README.md`](../cpcu_v2/README.md).

---

## License

Part of the InfiniTech Prosthetic Hand project. MIT — see
[`../LICENSE`](../LICENSE).
