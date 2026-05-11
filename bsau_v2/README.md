# BSAU — Bio-Signal Acquisition Unit

**The transmitter half of the InfiniTech prosthetic hand system.** A battery-powered
STM32L432KC wearable that captures 8-channel surface EMG, digitizes at 2 kHz, and
transmits 1000 packets/s to the CPCU over a 2.4 GHz NRF24L01+ radio link.

---

## Signal Chain

```
  8× Electrode Pair → 8× InAmp (200 V/V) → +1.1V DC bias
       │
       ▼
  STM32L432KC ADC1 (32× oversampling, 2 kHz, DMA circular)
       │
       ▼
  WL_Pack (12-bit codec, 32-byte NRF payload)
       │
       ▼
  NRF24L01+ PTX → 2 Mbps Enhanced ShockBurst → CPCU PRX
```

## Key Numbers

| Metric | Value |
|--------|-------|
| EMG channels | 8 (6 populated on current board) |
| Sample rate | 2000 Hz per channel |
| ADC resolution | 12-bit + 32× hardware oversampling (~14.5 ENOB) |
| Packet rate | 1000 pkt/s |
| Payload | 32 bytes (8 ch × 2 samples × 12 bits + 8 B metadata) |
| Battery life | ~66 hours (500 mAh @ 7.58 mA) |
| Radio | 2.4 GHz, 2 Mbps, CRC-16, auto-ACK, 15 retries |

---

## Firmware Profiles

Set in `bsau_config.h`:

| Profile | Purpose |
|---------|---------|
| `BSAU_MODE_RELEASE` | Normal operation — TX only, no UART |
| `BSAU_MODE_DEBUG` | TX + UART debug prints |
| `BSAU_MODE_DATASET` | TX + UART CSV output for ML training data capture |
| `BSAU_MODE_TEST` | Hardware self-test suite (ADC, NRF, SPI, GPIO) |

---

## Packet Format (32 bytes)

| Field | Bytes | Description |
|-------|-------|-------------|
| Samples | 24 | 8 channels × 2 samples × 12 bits (packed) |
| Sequence | 1 | 0–255 rolling counter |
| Flags | 1 | Bit 0: FIRST_PACKET (cold-start sync) |
| TX retries | 1 | NRF auto-retransmit count |
| Loss counter | 1 | Cumulative packet loss estimate |
| Timestamp | 2 | TIM6 capture (µs resolution) |
| Battery voltage | 2 | Raw 12-bit ADC (PB0, 2:1 divider) |

Codec shared with CPCU: `wireless_packet.h` / `wireless_packet.c`.

---

## NRF Recovery

NRF initialization is **non-fatal**. If the radio module fails at startup,
the MCU continues sampling and retries every 3 seconds via `nrf_try_recover()`.
This prevents a marginal power rail from bricking the entire acquisition chain.

`WL_FLAG_FIRST_PACKET` is set on the first packet after any NRF init/recovery,
enabling the CPCU to re-synchronize its sequence counter without declaring a gap fault.

---

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| MCU | STM32L432KC (Nucleo-L432KC) | — |
| Radio | NRF24L01+ | SPI1 @ 5 MHz |
| Power | 2S LiPo (7.4 V nominal) | LM7805 +5 V, LM1117 +3.3 V, LM7905 −5 V |
| Analog front-end | 8× TL074 op-amp channels | Gain 200 V/V, DC bias +1.1 V |

---

## File Map

```
bsau_v2/
├── Core/Src/
│   ├── main.c              ← Entry point, HAL init, profile dispatch
│   ├── bsau_app.c          ← Application layer (sampling, TX, LED)
│   ├── bsau_adc.c          ← ADC + DMA driver
│   ├── bsau_test.c         ← Hardware self-test suite
│   ├── nrf24l01.c          ← NRF24L01+ SPI driver
│   └── nrf24l01_test.c     ← NRF register validation
├── Core/Inc/
│   ├── bsau_app.h, bsau_adc.h, bsau_config.h, bsau_test.h
│   ├── nrf24l01.h, nrf24l01_test.h, main.h, log.h
│   └── (STM32 HAL headers)
└── README.md
```

Build with STM32CubeIDE. Flash via ST-Link or drag-and-drop to the Nucleo mass storage.
