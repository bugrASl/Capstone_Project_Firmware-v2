# BSAU System Architecture — v2.0 Redesign

**Author:** bugrASl  
**Board:** NUCLEO-L432KC (STM32L432KC, Cortex-M4F @ 80 MHz)  
**Radio:** NRF24L01+ (2.4 GHz ISM, Enhanced ShockBurst)  
**Receiver:** CPCU — Raspberry Pi 5 (Cortex-A76 × 4, isolated real-time cores)  
**Date:** 2026

---

## 1. System Overview

BSAU (Bio-Signal Acquisition Unit) is a battery-powered wearable board that acquires
surface EMG signals from 8 differential electrode pairs, digitises them at 2 kHz with
hardware oversampling, and streams the data wirelessly to a receiver unit (CPCU) for
real-time DSP and ML-based gesture classification.

### 1.1 Design Goals (v2.0)

| Goal | v1 baseline | v2 target | How achieved |
|------|-------------|-----------|--------------|
| More channels | 6 EMG | 8 EMG | Add PA6 (IN11), PA7 (IN12) |
| Better SNR | 4× OS (~13 ENOB) | 32× OS (~14.5 ENOB) | Hardware oversampler, 5-bit right shift |
| Lower power | 15.2 mA system | 7.58 mA system | 2 Mbps air → radio idle 54% of cycle |
| Link diagnostics | seq gaps only | 3 orthogonal metrics | tx_retry, pkt_loss, timestamp fields |
| Battery telemetry | 2-bit categorical | 12-bit raw ADC | vbat_raw field in packet |

All changes fit within the existing 32-byte NRF payload constraint.

### 1.2 Signal Chain

```
Electrode → Instrumentation Amp → Anti-alias Filter → ADC (32× OS)
→ DMA circular buffer → ISR snapshot → WL_Pack → SPI → NRF24L01+
→ 2.4 GHz air → NRF24L01+ (CPCU) → SPI → WL_Unpack → Ring buffer
→ DSP/ML pipeline (Cores 1-2) → Gesture output → Servo actuation (Core 3)
```

### 1.3 Key Specifications Summary

| Parameter | Value | Derivation reference |
|-----------|-------|---------------------|
| Sampling rate | 2000 Hz per channel | TIM6: 80 MHz / (0+1) / (39999+1) = 2000 Hz |
| ADC resolution | 12-bit native, ~14.5 ENOB effective | 32× OS, 5-bit right shift |
| EMG channels | 8 (PA0–PA7) | Ranks 1–8 in ADC scan |
| Battery channel | 1 (PB0) | Rank 9, 100k/100k divider |
| Samples per packet | 2 | DMA half/full ISR |
| Packet rate | 1000 pkt/s | 2000 Hz ÷ 2 samples/pkt |
| Air data rate | 2 Mbps | NRF RF_DR_HIGH = 1 |
| TX power | 0 dBm | NRF RF_PWR = 11 |
| RF channel | 76 (2476 MHz) | NRF RF_CH = 76 |
| Packet size | 32 bytes fixed | NRF_PAYLOAD_SIZE |
| Auto-retransmit | 15 retries, 500 µs ARD | NRF SETUP_RETR = 0x1F |
| CRC | 2-byte (X^16+X^12+X^5+1) | NRF CONFIG: EN_CRC=1, CRCO=1 |
| System current | 7.58 mA @ 3.0V | §8 detailed breakdown |
| Battery life (500 mAh) | ~66 hours | 500 / 7.58 = 65.96 h |

---

## 2. Pin Allocation

The STM32L432KC provides 26 GPIOs in a UFQFPN32 package. Every pin is accounted
for below. Pins not listed are power/ground/reset/oscillator.

### 2.1 ADC Channels (9 total)

| Rank | Pin | ADC Input | Function | Channel type | RAIN max @ 47.5 cyc |
|------|-----|-----------|----------|-------------|---------------------|
| 1 | PA0 | IN5 | EMG ch0 | Fast | 2200 Ω |
| 2 | PA1 | IN6 | EMG ch1 | Fast | 2200 Ω |
| 3 | PA2 | IN7 | EMG ch2 | Slow | 1800 Ω |
| 4 | PA3 | IN8 | EMG ch3 | Slow | 1800 Ω |
| 5 | PA4 | IN9 | EMG ch4 | Slow | 1800 Ω |
| 6 | PA5 | IN10 | EMG ch5 | Slow | 1800 Ω |
| 7 | PA6 | IN11 | EMG ch6 | Slow | 1800 Ω |
| 8 | PA7 | IN12 | EMG ch7 | Slow | 1800 Ω |
| 9 | PB0 | IN15 | Battery | Slow | 1800 Ω |

PA0 and PA1 are "fast" ADC channels on the STM32L432KC (datasheet Table 64).
Fast channels have a shorter internal multiplexer settling time, allowing higher RAIN
with the same sampling time. All channels use 47.5-cycle sampling — even the slow
channels have 1800 Ω RAIN margin, well above the typical 100–500 Ω output impedance
of an instrumentation amplifier.

### 2.2 NRF24L01+ Interface (SPI1 + GPIO)

| Pin | Function | Mode | Speed | Pull | Notes |
|-----|----------|------|-------|------|-------|
| PB3 | SPI1_SCK | AF5 (alternate function) | VERY_HIGH | None | JTDO shared — SWD still works |
| PB4 | SPI1_MISO | AF5 | VERY_HIGH | Pull-up | NJTRST shared — pull-up prevents float |
| PB5 | SPI1_MOSI | AF5 | VERY_HIGH | None | |
| PB7 | NRF_CSN | GPIO output, push-pull | HIGH | None | Init state: HIGH (SPI deselected) |
| PA8 | NRF_CE | GPIO output, push-pull | LOW | None | Init state: LOW (radio standby) |
| PB6 | NRF_IRQ | EXTI, falling edge | — | Pull-up | Active-low interrupt from NRF |

SPI1 runs at 5 MHz (PCLK2 80 MHz / prescaler 16). The NRF24L01 datasheet specifies
8 MHz maximum SPI clock. At 5 MHz with ~10 pF trace capacitance, the SPI timing is
well within spec (Tcd < 55 ns, see datasheet Table 18).

### 2.3 Debug Interface

| Pin | Function | Notes |
|-----|----------|-------|
| PA9 | USART1_TX | Debug UART, 921600 baud, 8N1 |
| PA10 | USART1_RX | Debug UART (unused in TX-only LOG) |
| PA13 | SWDIO | Serial Wire Debug — always connected |
| PA14 | SWCLK | Serial Wire Debug — always connected |

### 2.4 Other GPIO

| Pin | Function | Notes |
|-----|----------|-------|
| PA15 | STATUS_LED | GPIO output, push-pull, active high |
| PB1 | Unused | Configured as analog input (no current leak) |
| PA11 | Unused | Configured as analog input (USB_DM on NUCLEO, free when USB device is not used) |
| PH3 | BOOT0 | Configured as analog input (boot from Flash) |
| PC14 | OSC32_IN | Not used (no LSE crystal on NUCLEO) |
| PC15 | OSC32_OUT | Not used |

Every unused pin is configured as analog input with no pull resistor. This eliminates
leakage current through the input stage, which would otherwise be up to 1 µA per pin
(datasheet Table 57, Ilkg).

---

## 3. Clock Tree

### 3.1 Clock Source and PLL Configuration

```
Source:     MSI RC oscillator, 4 MHz (factory trimmed ±1%)
PLL input:  MSI 4 MHz / PLLM = 1 → 4 MHz VCO input
PLL VCO:    4 MHz × PLLN = 40 → 160 MHz
PLL output: 160 MHz / PLLR = 2 → 80 MHz SYSCLK

SYSCLK = 80 MHz
├─ AHB prescaler  = /1  → HCLK  = 80 MHz  (core, DMA, SRAM)
├─ APB1 prescaler = /1  → PCLK1 = 80 MHz  (TIM6, TIM7, TIM2, I2C)
├─ APB2 prescaler = /1  → PCLK2 = 80 MHz  (TIM1, TIM15, TIM16, USART1, SPI1)
└─ ADC clock      = PCLK synchronous /1 → 80 MHz (CKMODE = synchronous)

Timer clocks = PCLK × 2 when APB prescaler > 1.
Since APB prescaler = 1, timer clocks = PCLK = 80 MHz.
```

No external HSE crystal is used. MSI + PLL provides 80 MHz SYSCLK with ~±1%
frequency accuracy, sufficient for UART at 921600 baud (requires < ±2% for reliable
communication). The ADC clock is synchronous with PCLK (CKMODE bits = 01 in
ADC_CFGR2), giving zero jitter relative to the APB bus.

### 3.2 CubeMX Clock Configuration

```
RCC → Clock Source:      MSI (4 MHz, Range 6)
RCC → PLL:               Source = MSI, PLLM = 1, PLLN = 40, PLLR = 2
RCC → System Clock Mux:  PLLCLK
RCC → AHB Prescaler:     1
RCC → APB1 Prescaler:    1
RCC → APB2 Prescaler:    1
RCC → ADC Clock Source:  Synchronous clock mode (PCLK / 1)
```

### 3.3 TIM6 — ADC Trigger Timer (2 kHz)

TIM6 is a basic 16-bit up-counter with no channels. It generates a TRGO (trigger output)
event on each update, routed to ADC1 as an external trigger.

```
f_TIM6 = PCLK1 / (PSC + 1) / (ARR + 1)

Given:
  PCLK1 = 80,000,000 Hz
  PSC   = 0       (no prescaler, counter runs at full clock)
  ARR   = 39,999  (auto-reload value)

f_TIM6 = 80,000,000 / (0 + 1) / (39,999 + 1)
       = 80,000,000 / 1 / 40,000
       = 2,000.000 Hz exactly

T_TIM6 = 1 / 2000 = 500.000 µs per trigger
```

The period is exact (no rounding error) because 80,000,000 / 40,000 divides evenly.
This guarantees zero long-term frequency drift at precisely 2 kHz.

CubeMX configuration:
- Instance: TIM6
- Prescaler: 0
- Counter Mode: Up
- Period: 39999
- Auto-reload preload: Disabled
- Master Output Trigger: Update Event
- Master/Slave Mode: Disabled

### 3.4 TIM2 — Microsecond Timestamp (1 MHz, 32-bit)

TIM2 is a 32-bit general-purpose timer configured as a free-running counter for packet
timestamping. It runs continuously from power-on with no interrupts.

```
f_TIM2 = PCLK1 / (PSC + 1)
       = 80,000,000 / (79 + 1)
       = 80,000,000 / 80
       = 1,000,000 Hz = 1 µs resolution

Counter width: 32-bit
Full wrap period: 2^32 / 1,000,000 = 4,294.967 seconds ≈ 71.6 minutes
```

Only the low 16 bits are packed into the wireless packet:

```
16-bit wrap period = 2^16 / 1,000,000 = 0.065536 s = 65.536 ms
At 1000 pkt/s → ~65 packets per 16-bit wrap
```

The CPCU unwraps to a full 32-bit timestamp by tracking wraps. Since consecutive
packets are never more than ~12.5 ms apart (worst-case retransmit), a 65.5 ms wrap
window is always unambiguous.

CubeMX configuration:
- Instance: TIM2
- Prescaler: 79
- Counter Mode: Up
- Period: 0xFFFFFFFF (maximum, free-running)
- Auto-reload preload: Disabled
- No channels enabled
- Started in BSAU_Init() with `HAL_TIM_Base_Start(&htim2)` (no interrupt)

---

## 4. ADC Subsystem

### 4.1 CubeMX Configuration

| Parameter | Value | Register / Field |
|-----------|-------|-----------------|
| Instance | ADC1 | — |
| Clock Prescaler | Synchronous /1 | CKMODE = 01 (ADC_CFGR2) |
| Resolution | 12-bit | RES = 00 (ADC_CFGR) |
| Data Alignment | Right | ALIGN = 0 (ADC_CFGR) |
| Scan Conversion Mode | Enabled | SCAN = 1 |
| EOC Selection | Single conversion | — |
| Low Power Auto Wait | Disabled | AUTDLY = 0 |
| Continuous Conversion | Disabled | CONT = 0 |
| Number of Conversions | 9 | — |
| Discontinuous Mode | Disabled | DISCEN = 0 |
| External Trigger | TIM6 TRGO | EXTSEL (ADC_CFGR) |
| External Trigger Edge | Rising | EXTEN = 01 |
| DMA Continuous Requests | Enabled | DMACFG = 1 |
| Overrun Behaviour | Data overwritten | OVRMOD = 1 |
| Oversampling Mode | Enabled | OVSE = 1 (ADC_CFGR2) |
| Oversampling Ratio | 32× | OVSR = 100 (ADC_CFGR2) |
| Right Bit Shift | 5 | OVSS = 0101 (ADC_CFGR2) |
| Triggered Mode | Single trigger | TROVS = 0 (all 32× per trigger) |
| Oversampling Stop Reset | Continued mode | ROVSM = 0 |
| Sampling Time (all channels) | 47.5 ADC cycles | SMPx = 011 (ADC_SMPR1/2) |

### 4.2 Conversion Timing

Each conversion = sampling + successive approximation:

```
t_sample = 47.5 cycles / 80 MHz = 593.75 ns
t_SAR    = 12.5 cycles / 80 MHz = 156.25 ns
t_conv   = t_sample + t_SAR
         = 60 cycles / 80 MHz
         = 750 ns = 0.75 µs per single conversion
```

With 32× oversampling in single-trigger mode, all 32 conversions per channel happen
consecutively within one TIM6 trigger event:

```
t_scan = N_channels × N_oversampling × t_conv
       = 9 × 32 × 0.75 µs
       = 216 µs per trigger event
```

### 4.3 Timing Budget Verification

TIM6 fires every 500 µs. The ADC scan must complete before the next trigger:

```
Budget utilisation = t_scan / T_trigger × 100%
                   = 216 / 500 × 100%
                   = 43.2%

Headroom = 500 - 216 = 284 µs (56.8% free)
```

The 284 µs headroom accommodates:
- DMA ISR entry (tail-chain): 12 cycles / 80 MHz = 0.15 µs
- ISR execution (snapshot memcpy 9 half-words): ~0.45 µs
- Context save/restore overhead: ~1.5 µs total
- Trigger-to-conversion latency: max 2.5 / f_ADC = 31.25 ns (datasheet t_LATR)
- Margin for interrupt priority inversion with SPI or UART

Total overhead: < 3 µs, well within 284 µs headroom.

### 4.4 Oversampling Ratio Selection

| Ratio | t_scan | Budget % | Headroom | SNR gain | ENOB gain | Selected? |
|-------|--------|----------|----------|----------|-----------|-----------|
| 4× | 27 µs | 5.4% | 473 µs | 6.02 dB | +1.0 bit | v1 (old) |
| 16× | 108 µs | 21.6% | 392 µs | 12.04 dB | +2.0 bits | |
| 32× | 216 µs | 43.2% | 284 µs | 15.05 dB | +2.5 bits | **v2 ✓** |
| 64× | 432 µs | 86.4% | 68 µs | 18.06 dB | +3.0 bits | rejected |
| 128× | 864 µs | 172.8% | — | — | — | does not fit |

The SNR improvement from oversampling uncorrelated (thermal) noise:

```
SNR_gain = 10 × log₁₀(N) dB
ENOB_gain = ½ × log₂(N) bits

For N = 32:
  SNR_gain  = 10 × log₁₀(32) = 10 × 1.505 = 15.05 dB
  ENOB_gain = ½ × log₂(32)   = ½ × 5      = 2.5 bits

For N = 64:
  SNR_gain  = 10 × log₁₀(64) = 10 × 1.806 = 18.06 dB
  ENOB_gain = ½ × log₂(64)   = ½ × 6      = 3.0 bits

Marginal gain (64× vs 32×) = 3.01 dB = 0.5 effective bits
```

The STM32L432KC native ENOB is ~10.5 bits (datasheet Table 65, single-ended fast
channel at max speed). Systematic errors — INL (~2.5 LSB typical), DNL (~1.0 LSB),
gain error (~2.5 LSB) — create a noise floor that oversampling cannot reduce. At 32×,
effective resolution reaches ~14.5 ENOB, near the systematic error floor. Going to 64×
squeezes only 0.5 extra bits but consumes 86.4% of timing budget (68 µs headroom).
A single delayed ISR would cause an ADC overrun. 32× is the optimal trade-off.

### 4.5 Sampling Time Justification

From datasheet Table 64, at 80 MHz ADC clock and 12-bit resolution:

```
Sampling cycles: 47.5
Sampling time:   47.5 / 80 MHz = 593.75 ns

Fast channels (PA0, PA1):  RAIN_max = 2200 Ω
Slow channels (PA2–PA7, PB0): RAIN_max = 1800 Ω
```

Typical EMG front-end output impedance is 100–500 Ω (op-amp buffered). Both channel
types are well within spec. Longer sampling times (92.5, 247.5, 640.5 cycles) would
increase RAIN tolerance but proportionally increase t_scan.

### 4.6 DMA Circular Buffer Mechanics

DMA operates in circular mode with half-transfer and transfer-complete interrupts,
implementing a hardware double-buffer:

```
ADC_DMA_CHANNELS = 9     (8 EMG + 1 battery per scan)
ADC_DMA_SAMPLES  = 2     (2 scan sets fill one packet)
ADC_DMA_BUF_SIZE = 9 × 2 = 18 half-words = 36 bytes

Physical buffer (linear, 18 × uint16_t):
  ┌──────────────────────────────────────────────────────────────────┐
  │ Scan 0 (1st TIM6 trigger)              │ Scan 1 (2nd trigger)  │
  │ ch0 ch1 ch2 ch3 ch4 ch5 ch6 ch7 batt   │ ch0 ch1 ... batt      │
  │ [0] [1] [2] [3] [4] [5] [6] [7] [8]    │ [9] [10]  ... [17]    │
  └──────────────────────────────────────────────────────────────────┘
          ↑ Half-transfer ISR                    ↑ Transfer-complete ISR
```

DMA1 Channel 1 configuration:
- Direction: Peripheral → Memory
- Peripheral: ADC1_DR (data register), no increment
- Memory: g_adc_dma_buf[18], increment enabled
- Data width: Half-word (16-bit) on both sides
- Mode: Circular
- Priority: High
- Half-transfer interrupt: Enabled
- Transfer-complete interrupt: Enabled

### 4.7 ISR Behaviour (Pseudo-code)

```c
// Called when DMA has filled indices 0–8 (scan 0 complete)
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    // Scan 0 is stable — DMA is now writing indices 9–17
    memcpy(g_adc_snapshot, g_adc_dma_buf, 9 * sizeof(uint16_t));
    // Do NOT set g_pkt_ready — wait for scan 1
}

// Called when DMA has filled indices 9–17 (scan 1 complete, wraps to 0)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // Scan 1 is stable — DMA wraps back to index 0
    memcpy(&g_adc_snapshot[9], &g_adc_dma_buf[9], 9 * sizeof(uint16_t));

    // Both scans now in snapshot — signal main loop
    if (g_pkt_ready)
        g_adc_dropped++;   // Previous snapshot was not consumed in time
    g_pkt_ready = 1;
}
```

The half-complete callback copies scan 0 while DMA fills scan 1. The transfer-complete
callback copies scan 1 and sets `g_pkt_ready`. This ensures the main loop sees a
coherent pair of scans — never a mix of old and new data.

If `g_pkt_ready` is still set when transfer-complete fires, the previous snapshot was
not consumed (main loop was busy with a long retransmit). The `g_adc_dropped` counter
tracks this. In RELEASE mode with no UART overhead, drops should be zero.

---

## 5. NVIC Priority Map

Cortex-M4 configured with NVIC_PRIORITYGROUP_4 (4 bits of preemption priority,
0 bits of sub-priority). Gives 16 preemption levels (0 = highest).

| Priority | Interrupt | Peripheral | Rationale |
|----------|-----------|------------|-----------|
| 0 | DMA1_Channel1_IRQn | ADC DMA | Highest — must never miss a scan completion |
| 1 | DMA1_Channel4_IRQn | USART1 TX DMA | Debug output, can preempt SysTick but not ADC |
| 2 | EXTI9_5_IRQn | NRF_IRQ (PB6) | Radio events — not actively used in v2.1 |
| 3 | ADC1_IRQn | ADC error/overrun | Deferred error logging |
| 15 | SysTick_IRQn | HAL tick | Lowest — 1 ms timebase for HAL_Delay/HAL_GetTick |

The DMA ISR at priority 0 ensures that the snapshot memcpy (~0.5 µs) is never
interrupted by anything. No other ISR can delay the ADC data capture.

The NRF_IRQ EXTI is configured but not actively used in v2.1 — the blocking
`NRF_Transmit()` polls STATUS via SPI. A future optimisation (§15.1) would enable
the EXTI handler to set a flag, allowing WFI during radio transactions.

CubeMX NVIC configuration:
- NVIC → DMA1 channel 1 global interrupt → Preemption Priority: 0
- NVIC → DMA1 channel 4 global interrupt → Preemption Priority: 1
- NVIC → EXTI line [9:5] interrupts → Preemption Priority: 2
- NVIC → ADC1 global interrupt → Preemption Priority: 3
- NVIC → System tick timer → Preemption Priority: 15

---

## 6. NRF24L01+ Radio Configuration

### 6.1 Complete Register Map

| Register | Addr | Value | Bit-field encoding |
|----------|------|-------|--------------------|
| CONFIG | 0x00 | 0x4E | [7]MASK_RX_DR=0 [6]MASK_TX_DS=1 [5]MASK_MAX_RT=0 [4]EN_CRC=0→wait. Let me decode properly. |

Let me decode CONFIG = 0x4E = 0b_0100_1110:

```
CONFIG = 0x4E = 0b_0100_1110

Bit 7: Reserved     = 0
Bit 6: MASK_RX_DR   = 1  (mask RX_DR on IRQ pin — TX mode doesn't need it)
Bit 5: MASK_TX_DS   = 0  (unmask TX_DS — we want to see successful TX)
Bit 4: MASK_MAX_RT  = 0  (unmask MAX_RT — we want to see failed TX)
Bit 3: EN_CRC       = 1  (CRC enabled)
Bit 2: CRCO         = 1  (2-byte CRC)
Bit 1: PWR_UP       = 1  (powered up)
Bit 0: PRIM_RX      = 0  (TX mode)
```

Full register table:

| Register | Addr | Value | Description |
|----------|------|-------|-------------|
| CONFIG | 0x00 | 0x4E | MASK_RX_DR, EN_CRC, CRCO=2B, PWR_UP, TX mode |
| EN_AA | 0x01 | 0x01 | Auto-ACK on pipe 0 only |
| EN_RXADDR | 0x02 | 0x01 | Pipe 0 enabled only |
| SETUP_AW | 0x03 | 0x03 | 5-byte address width (AW = 11) |
| SETUP_RETR | 0x04 | 0x1F | ARD=500µs, ARC=15 retries |
| RF_CH | 0x05 | 0x4C | Channel 76 (2476 MHz) |
| RF_SETUP | 0x06 | 0x0F | 2 Mbps, 0 dBm, LNA high current |
| STATUS | 0x07 | (R/W) | Read to poll TX_DS / MAX_RT |
| OBSERVE_TX | 0x08 | (RO) | PLOS_CNT[7:4], ARC_CNT[3:0] |
| CD | 0x09 | (RO) | Carrier detect (for future freq hopping) |
| RX_ADDR_P0 | 0x0A | E7 E7 E7 E7 E7 | Pipe 0 address (matches TX_ADDR for auto-ACK) |
| TX_ADDR | 0x10 | E7 E7 E7 E7 E7 | Transmit address (5 bytes) |
| RX_PW_P0 | 0x11 | 0x20 | 32-byte fixed payload width |
| FIFO_STATUS | 0x17 | (RO) | TX_FULL, TX_EMPTY flags |
| DYNPD | 0x1C | 0x00 | No dynamic payload on any pipe |
| FEATURE | 0x1D | 0x00 | No ACK payload, no dynamic payload |

### 6.2 RF_SETUP Register (0x06) Bit Decoding

```
RF_SETUP = 0x0F = 0b_0000_1111

Bit 7:6 : Reserved     = 00
Bit 5   : RF_DR_LOW    = 0   (not 250 kbps)
Bit 4   : PLL_LOCK     = 0   (normal operation, not force PLL lock)
Bit 3   : RF_DR_HIGH   = 1   (2 Mbps — with DR_LOW=0)
Bit 2:1 : RF_PWR       = 11  (0 dBm, 11.3 mA TX current)
Bit 0   : LNA_HCURR    = 1   (LNA high current gain, max sensitivity)
```

Data rate encoding:

| RF_DR_LOW (bit 5) | RF_DR_HIGH (bit 3) | Data rate | RX sensitivity | TX current |
|----|----|----|----|-----|
| 0 | 0 | 1 Mbps | -85 dBm | 11.3 mA |
| 0 | 1 | 2 Mbps | -82 dBm | 11.3 mA |
| 1 | 0 | 250 kbps | -94 dBm | 11.3 mA |
| 1 | 1 | Reserved | — | — |

### 6.3 SETUP_RETR Register (0x04) Bit Decoding

```
SETUP_RETR = 0x1F = 0b_0001_1111

Bits 7:4 : ARD = 0001 → (0001 + 1) × 250 µs = 2 × 250 = 500 µs
Bits 3:0 : ARC = 1111 → 15 maximum retransmits
```

At 2 Mbps with no ACK payload, 500 µs is the datasheet minimum ARD for any payload
length (datasheet §7.5.2). v1 used ARD=1500 µs (0x5F) for 250 kbps — the reduction
tightens retransmit recovery by 3×.

### 6.4 On-Air Timing (2 Mbps)

Enhanced ShockBurst data packet:

```
┌──────────┬─────────┬───────────────────┬──────────────┬───────────┐
│ Preamble │ Address │ Packet Control    │   Payload    │    CRC    │
│  1 byte  │ 5 bytes │ 9 bits            │  32 bytes    │  2 bytes  │
└──────────┴─────────┴───────────────────┴──────────────┴───────────┘

Total bits = (1 + 5 + 32 + 2) × 8 + 9
           = 40 × 8 + 9 = 320 + 9 = 329 bits

T_OA = 329 / 2,000,000 = 164.5 µs
```

ACK packet (empty payload):

```
Total bits = (1 + 5 + 0 + 2) × 8 + 9
           = 8 × 8 + 9 = 64 + 9 = 73 bits

T_ACK = 73 / 2,000,000 = 36.5 µs
```

### 6.5 ESB Cycle Time (no retransmit)

```
T_ESB = T_OA + T_settle(TX→RX) + T_ACK + T_settle(RX→Standby-I)

T_settle = 130 µs (datasheet Table 13, Tstby2a)

T_ESB = 164.5 + 130 + 36.5 + 130 = 461 µs
```

### 6.6 Worst-Case Retransmit Time

```
T_worst = T_OA + ARC × (ARD + T_OA + T_settle)
        = 164.5 + 15 × (500 + 164.5 + 130)
        = 164.5 + 15 × 794.5
        = 164.5 + 11,917.5
        = 12,082 µs ≈ 12.1 ms

NRF_TX_TIMEOUT_MS = 20 ms → headroom factor = 20 / 12.1 = 1.65×
```

### 6.7 Packet Rate Feasibility

```
Packet period = 1000 µs (at 1000 pkt/s)
Radio active  = 461 µs = 46.1%
Radio idle    = 539 µs in Standby-I (22 µA)

One retransmit: ~461 + 795 = 1256 µs → exceeds period by 256 µs
  → next packet delayed, but system self-recovers on the following cycle
  → at 2 Mbps LoS, retransmits < 1% of packets
```

### 6.8 SPI Transfer Time

```
SPI1 at 5 MHz, 8-bit mode:

Payload write: (1 + 32) bytes × 8 bits / 5 MHz = 52.8 µs
STATUS read:   1 byte / 5 MHz                  = 1.6 µs
OBSERVE_TX:    (1 + 1) bytes / 5 MHz            = 3.2 µs
FIFO status:   (1 + 1) bytes / 5 MHz            = 3.2 µs
```

### 6.9 Channel Selection and Wi-Fi Coexistence

```
F₀ = 2400 + RF_CH = 2400 + 76 = 2476 MHz
Bandwidth at 2 Mbps: 2 MHz → occupies 2475–2477 MHz

Wi-Fi channel map:
  Ch 1:  2401–2423 MHz (centre 2412)
  Ch 6:  2426–2448 MHz (centre 2437)
  Ch 11: 2451–2473 MHz (centre 2462)

Channel 76 (2476 MHz) sits 3 MHz above Wi-Fi ch 11's upper edge (2473 MHz).
This is one of the quietest zones in the 2.4 GHz ISM band.
```

### 6.10 Link Budget

```
P_TX          = 0 dBm
P_RX_min      = -82 dBm (at 2 Mbps, BER 0.1%)
Link budget   = 0 - (-82) = 82 dB

Free-space path loss: FSPL(d) = 20×log₁₀(d) + 20×log₁₀(f) - 147.55
                               = 20×log₁₀(d) + 187.88 - 147.55
                               = 20×log₁₀(d) + 40.33

At d = 5 m:
  FSPL(5) = 20×log₁₀(5) + 40.33 = 13.98 + 40.33 = 54.31 dB
  With PCB antenna losses (3 dB × 2 sides) = 54.31 + 6 = 60.31 dB
  Margin = 82 - 60.31 = 21.69 dB

  In linear: 10^(21.69/10) = 147.6× power margin

Max free-space range:
  82 = 20×log₁₀(d_max) + 40.33  →  d_max = 10^2.08 = 121 m

Practical indoor range (with 10 dB fading margin):
  66 = 20×log₁₀(d) + 40.33  →  d = 10^1.28 = 19 m
```

---

## 7. Main Loop Execution Model

### 7.1 Initialisation Sequence

```
power_on → SystemClock_Config() → MX_GPIO_Init() → MX_DMA_Init()
→ MX_ADC1_Init() → MX_SPI1_Init() → MX_USART1_Init()
→ MX_TIM6_Init() → MX_TIM2_Init() → BSAU_Init() → BSAU_Run()
```

BSAU_Init() steps:
1. Assert NVIC priority grouping = NVIC_PRIORITYGROUP_4
2. If TEST_* mode → delegate to BSAU_Test_Init(), return
3. Wait NRF POR delay (200 ms for crystal startup, datasheet §6.1.7)
4. NRF_Init() with address and channel; if fails, retry once after 100 ms
5. BSAU_ADC_Init(): HAL_ADCEx_Calibration_Start(), HAL_ADC_Start_DMA(), TIM6 start
6. HAL_TIM_Base_Start(&htim2) — free-running timestamp counter
7. Set FIRST_PACKET flag for first transmitted packet

### 7.2 Main Loop Pseudo-Code

```c
void BSAU_Run(void)
{
    WL_Packet   pkt;
    uint8_t     raw[WL_PAYLOAD_SIZE];
    uint8_t     tx_seq       = 0;
    uint32_t    pkt_count    = 0;
    uint32_t    lost_count   = 0;
    uint8_t     prev_retry   = 0;
    uint8_t     prev_loss    = 0;

    while (1)
    {
        __WFI();                              // Sleep until any interrupt

        if (!g_pkt_ready) continue;           // Spurious wake → sleep again
        g_pkt_ready = 0;

        // ── Build metadata ────────────────────────────────────
        pkt.seq       = tx_seq++;
        pkt.flags     = 0;
        pkt.tx_retry  = prev_retry;           // From PREVIOUS transmit
        pkt.pkt_loss  = prev_loss;
        pkt.timestamp = (uint16_t)(TIM2->CNT & 0xFFFF);

        // Battery: average of scan 0 and scan 1
        uint16_t batt_0 = g_adc_snapshot[ADC_BATT_INDEX];       // index 8
        uint16_t batt_1 = g_adc_snapshot[9 + ADC_BATT_INDEX];   // index 17
        pkt.vbat_raw  = (batt_0 + batt_1) / 2;

        // Battery level flags
        if      (pkt.vbat_raw < 1675) pkt.flags = WL_BATT_SET(pkt.flags, WL_BATT_CRITICAL);
        else if (pkt.vbat_raw < 1861) pkt.flags = WL_BATT_SET(pkt.flags, WL_BATT_LOW);
        else                          pkt.flags = WL_BATT_SET(pkt.flags, WL_BATT_OK);

        // Deferred ADC error check
        if (g_adc_error_code) {
            pkt.flags |= WL_FLAG_ADC_OVRN;
            g_adc_error_code = 0;
        }

        // First packet flag
        if (pkt_count == 0) pkt.flags |= WL_FLAG_FIRST_PACKET;

        // ── Copy EMG samples (stride = ADC_DMA_CHANNELS = 9) ──
        for (int s = 0; s < 2; s++)
            for (int c = 0; c < 8; c++)
                pkt.samples[s].ch[c] = g_adc_snapshot[s * 9 + c];

        // Clipping detection
        for (int s = 0; s < 2; s++)
            for (int c = 0; c < 8; c++)
                if (pkt.samples[s].ch[c] == 0 || pkt.samples[s].ch[c] >= 4095)
                    pkt.flags |= WL_FLAG_CLIPPING;

        // ── Encode and transmit ───────────────────────────────
        WL_Pack(&pkt, raw);
        NRF_Status status = NRF_Transmit(&g_hnrf, raw);

        // Read link quality AFTER transmit (for NEXT packet)
        uint8_t observe = NRF_ReadReg(&g_hnrf, NRF_REG_OBSERVE_TX);
        prev_retry = observe & 0x0F;           // ARC_CNT
        prev_loss  = (observe >> 4) & 0x0F;    // PLOS_CNT

        if (status == NRF_ERR_TX_MAX_RT)
            lost_count++;

        pkt_count++;
    }
}
```

### 7.3 Execution Timeline (1 ms period, µs resolution)

```
Time (µs)   Event                                  Current state
──────────────────────────────────────────────────────────────────
0           TIM6 TRGO → ADC scan 0 starts          MCU: WFI (sleep)
0–216       ADC + DMA converting 9ch × 32×          ADC active
216         DMA half-complete ISR: memcpy scan 0     MCU: Run (~0.5 µs)
~217        ISR returns → CPU back in WFI           MCU: WFI

500         TIM6 TRGO → ADC scan 1 starts          MCU: WFI
500–716     ADC + DMA converting                    ADC active
716         DMA transfer-complete ISR:              MCU: Run (~0.5 µs)
              memcpy scan 1, set g_pkt_ready=1

~717        WFI wakes, main loop runs:              MCU: Run
              OBSERVE_TX read (SPI)      ~5 µs
              TIM2->CNT capture          ~0.1 µs
              Battery read + averaging   ~0.1 µs
              Build WL_Packet struct     ~3 µs
              WL_Pack() → 32 bytes       ~2 µs
              NRF_Transmit():
                SPI payload write         ~53 µs
                CE pulse                  ~15 µs
                STATUS poll loop          ~461 µs     NRF: TX→RX→Standby
~1256       NRF_Transmit returns (TX_DS)

~1257       OBSERVE_TX read (SPI)        ~5 µs
~1262       CPU enters WFI                          MCU: WFI
                                                     NRF: Standby-I (22 µA)
~1500       Next TIM6 trigger → cycle repeats
```

### 7.4 CPU Active Time Per Packet

```
t_ISR_half  ≈ 0.5 µs    (half-complete: memcpy 9 half-words)
t_ISR_full  ≈ 0.5 µs    (transfer-complete: memcpy + flag set)
t_build     ≈ 10 µs     (OBSERVE_TX + timestamp + struct fill + WL_Pack)
t_SPI_write ≈ 53 µs     (33 bytes payload write at 5 MHz)
t_CE_pulse  ≈ 15 µs     (volatile delay loop, Cortex-M4 at 80 MHz)
t_poll      ≈ 461 µs    (blocking poll during ESB cycle)
t_post_read ≈ 5 µs      (OBSERVE_TX read after transmit)

t_active = 0.5 + 0.5 + 10 + 53 + 15 + 461 + 5 = 545 µs

CPU duty cycle = 545 / 1000 = 54.5%
WFI time       = 455 µs per ms (45.5%)
```

---

## 8. Power Consumption Analysis

All values at VDD = 3.0V, T = 25°C.

### 8.1 STM32L432KC (MCU)

From datasheet:
- Table 25: Run mode @ 80 MHz, Flash with ART enabled = 3.60 mA typical
- Table 31: Sleep mode @ 80 MHz, PLL on = 2.23 mA typical
- Table 38: Peripheral current consumption:

| Peripheral | µA/MHz (Range 1) | Active? | Estimated current |
|-----------|-----------------|---------|-------------------|
| ADC1 | ~200 µA/Msps | Yes (0.576 Msps eff.) | 0.115 mA |
| SPI1 | 4.4 µA/MHz | Yes (5 MHz, ~10% duty) | 0.022 mA |
| TIM6 | 2.5 µA/MHz (basic) | Yes (80 MHz clock) | 0.020 mA |
| TIM2 | 4.3 µA/MHz (GP) | Yes (80 MHz clock) | 0.040 mA |
| DMA1 | 0.7 µA/MHz | Yes | 0.006 mA |
| USART1 | 4.8 µA/MHz | No (RELEASE mode) | 0.000 mA |
| GPIO | ~1 µA/pin | 3 active outputs | 0.003 mA |
| **Total peripherals** | | | **0.206 mA** |

ADC effective sample rate:

```
ADC conversions per second = 9 ch × 32 OS × 2000 triggers/s = 576,000 conv/s
But conversions are bursty (216 µs active, 284 µs idle per trigger)
Average: 576,000 conv/s × 0.75 µs/conv = 0.432 → duty = 43.2%
ADC current ≈ 200 µA/Msps × 0.576 Msps = 0.115 mA
```

Weighted MCU current:

```
I_MCU = (duty_active × I_run) + (duty_sleep × I_sleep) + I_peripherals
      = (0.545 × 3.60) + (0.455 × 2.23) + 0.206
      = 1.962 + 1.015 + 0.206
      = 3.183 mA
```

### 8.2 NRF24L01+ (Radio)

From NRF datasheet Table 4:

| State | Current | Duration per ESB cycle |
|-------|---------|----------------------|
| TX @ 0 dBm | 11.3 mA | 164.5 µs |
| TX→RX settling | 8.0 mA (est.) | 130 µs |
| RX @ 2 Mbps | 12.3 mA | 36.5 µs |
| RX→Standby settling | 8.0 mA (est.) | 130 µs |
| Standby-I | 0.022 mA | 539 µs (remainder of 1000 µs period) |

Weighted active current during one ESB cycle (461 µs):

```
I_active = (164.5×11.3 + 130.0×8.0 + 36.5×12.3 + 130.0×8.0) / 461
         = (1858.85 + 1040.0 + 448.95 + 1040.0) / 461
         = 4387.80 / 461
         = 9.518 mA
```

Average NRF current over full packet period (1000 µs):

```
I_NRF = (461/1000 × 9.518) + (539/1000 × 0.022)
      = 4.388 + 0.012
      = 4.400 mA
```

### 8.3 Total System Current

```
I_total = I_MCU + I_NRF = 3.183 + 4.400 = 7.583 mA
P_total = I_total × VDD = 7.583 × 3.0 = 22.75 mW
```

### 8.4 Battery Life

| Battery | Runtime | Calculation |
|---------|---------|-------------|
| 500 mAh LiPo | 65.9 hours (2.7 days) | 500 / 7.583 |
| 250 mAh (small) | 33.0 hours (1.4 days) | 250 / 7.583 |
| 1000 mAh (large) | 131.9 hours (5.5 days) | 1000 / 7.583 |

### 8.5 v1 vs v2 Comparison

| Parameter | v1 (250 kbps) | v2 (2 Mbps) | Change |
|-----------|---------------|-------------|--------|
| Air data rate | 250 kbps | 2 Mbps | 8× faster |
| T_ESB per packet | 1868 µs | 461 µs | 4× shorter |
| Packet period | 1500 µs (667 pkt/s) | 1000 µs (1000 pkt/s) | |
| NRF active duty | ~100% | 46.1% | -54% |
| NRF avg current | 11.8 mA | 4.40 mA | **-63%** |
| MCU avg current | 3.40 mA | 3.18 mA | -6% |
| **Total system** | **15.2 mA** | **7.58 mA** | **-50%** |
| Battery life (500 mAh) | 32.9 h | 65.9 h | **+100%** |

v1 NRF timing proof (radio never idled):

```
T_OA(250k) = 329 / 250,000 = 1,316 µs
T_ACK(250k) = 73 / 250,000 = 292 µs
T_ESB(250k) = 1316 + 130 + 292 + 130 = 1,868 µs

Packet period at 667 pkt/s = 1,500 µs
Since 1,868 > 1,500 → NRF never enters Standby-I → I_NRF ≈ 11.8 mA constant
```

---

## 9. Data Throughput

```
EMG per packet:      8 ch × 2 samples × 12 bits = 192 bits = 24 bytes
Metadata per packet: 8 bytes
Total per packet:    32 bytes
Packet rate:         1000 pkt/s

EMG throughput:      24,000 bytes/s = 192 kbps
Total throughput:    32,000 bytes/s = 256 kbps
Payload efficiency:  24/32 = 75.0%

On-air bits/pkt:     329 bits (with ESB overhead)
On-air throughput:   329,000 bps = 329 kbps
Air utilisation:     329/2000 = 16.45%
```

---

## 10. Reliability & Link Quality Monitoring

### 10.1 seq — Packet Loss Detection

8-bit sequence counter. CPCU detects gaps: if `received_seq != expected_seq`, then
`(received_seq - expected_seq) & 0xFF - 1` packets were lost.
Wraps every 256 ms. Extended to 32-bit on CPCU by counting wraps.

### 10.2 tx_retry — Early Warning Metric

ARC_CNT from OBSERVE_TX[3:0]. Read after NRF_Transmit(), stored in the NEXT packet.
Shows retransmit count for the previous transmission (0 = first-attempt success).
This is the EARLY WARNING signal — retries increase before total loss occurs.

### 10.3 pkt_loss — Cumulative Loss Counter

PLOS_CNT from OBSERVE_TX[7:4]. Saturating 4-bit counter. Increments on MAX_RT events.
Resets only when RF_CH is written. CPCU computes deltas to track loss rate over time.

### 10.4 timestamp — Jitter Detection

TIM2 µs counter (low 16 bits). CPCU computes `dt = (ts[n] - ts[n-1]) & 0xFFFF`.
Expected dt = 1000 ± 50 µs. Large dt values indicate retransmit events on the
previous packet. Critical for frequency-domain ML features.

---

## 11. Firmware Module Dependency Graph

```
main.c
  └─ bsau_app.h / bsau_app.c           (application layer: init, run, packet loop)
       ├─ bsau_adc.h / bsau_adc.c       (ADC/DMA init, ISR callbacks, battery read)
       ├─ wireless_packet.h / .c         (WL_Pack / WL_Unpack, 12-bit codec)
       ├─ nrf24l01.h / nrf24l01.c        (NRF driver, compiled TX or RX via #define)
       ├─ log.h                          (LOG / LOG_CSV macros)
       │    └─ bsau_config.h             (build mode select)
       │         └─ bsau_log.h           (BSAU-specific LOG wrappers)
       └─ bsau_test.h / bsau_test.c      (test harness, TEST_* modes only)
            └─ nrf24l01_test.h / .c       (NRF hardware self-test suite)

HAL-generated (CubeMX, edit only within USER CODE blocks):
  adc.c / adc.h, spi.c / spi.h, usart.c / usart.h
  tim.c / tim.h, dma.c / dma.h, gpio.c / gpio.h
  main.c / main.h, stm32l4xx_it.c / stm32l4xx_it.h
  stm32l4xx_hal_msp.c, stm32l4xx_hal_conf.h
  system_stm32l4xx.c, syscalls.c, sysmem.c
```

---

## 12. Build Modes

Defined in `bsau_config.h`. Exactly one must be active.

| Mode | LOG | CSV | Radio | ADC | Description |
|------|-----|-----|-------|-----|-------------|
| BSAU_MODE_RELEASE | off | off | TX | yes | Production — zero debug overhead |
| BSAU_MODE_DEBUG | on | on | TX | yes | Dev: LOG stats + decimated CSV |
| BSAU_MODE_TEST_ADC_CSV | off | binary | no | yes | Binary ADC stream for SerialPlot |
| BSAU_MODE_TEST_PKT_LOG | on | off | no | no | WL codec round-trip tests |
| BSAU_MODE_TEST_CSV | off | ASCII | no | yes | CSV capture with drop column |
| BSAU_MODE_TEST_DFT_LOG | on | off | no | yes | Goertzel DFT frequency verify |
| BSAU_MODE_TEST_NRF_LOG | on | off | TX | no | NRF self-test + TX stress loop |
| **BSAU_MODE_DATASET** | off | ASCII | TX | yes | **Dual-path: full RELEASE TX loop + 8-channel CSV on UART** (see §12.1) |

In RELEASE mode, all LOG/CSV macros compile to `((void)0)` — zero code, zero RAM.

---

### 12.1 DATASET mode — simultaneous UART + radio capture

The DSP/AI team needs a training dataset in the same shape that
`predict.py` expects to see at inference time. They also want to validate
the over-the-air path by comparing:

-  a **raw** stream captured directly at the BSAU UART (golden reference,
   no wireless in the loop), and
-  a **filtered** stream captured at the CPCU (same muscle contraction,
   but with the 4th-order Butterworth 20–450 Hz bandpass + 50 Hz notch
   already applied in the on-target processing chain).

DATASET mode makes both streams available from a single physical capture
session. The BSAU runs its normal release-grade acquisition and transmits
to CPCU every packet, and on top of that emits one ASCII CSV line on
USART1 per packet (scan 0 only, 1 kHz):

```
c0,c1,c2,c3,c4,c5,c6,c7\r\n
```

No index, no timestamp, no battery, no flags — just the eight 12-bit ADC
values. This matches the column layout `predict.py` trained on (widened
from 3 sensors to the full 8 BSAU channels) so the same loader works for
both the BSAU- and CPCU-side files.

#### Timing and UART budget

```
Packet period          = 1 ms          (1000 pkt/s at 2 Mbps air)
CSV line length        = up to 42 B    ("4095" × 8 + 7 commas + "\r\n")
UART baud              = 921600        (huart1 in main.c)
Bits per byte on wire  = 10            (1 start + 8 data + 1 stop)
Line time (blocking)   = 42 × 10 / 921600 = 0.456 ms
Steady-state bandwidth = 42 kB/s = 420 kbps (≈ 46 % of 921600 baud)
```

The `HAL_UART_Transmit` call inside `LOG_CSV` is blocking polling, but the
timeout (15 ms) is the upper bound — it returns as soon as the TX FIFO
drains, which at 921600 baud is ~half the packet period. Overlap with
the NRF SPI transfer during the same iteration is fine because they sit
on different AHB masters.

Decimation is configurable in `bsau_config.h`:

```
#define BSAU_DATASET_CSV_DECIMATION  1U      // every packet -> 1000 Hz CSV
#define BSAU_DATASET_CSV_DECIMATION  2U      // every 2nd packet -> 500 Hz
#define BSAU_DATASET_CSV_DECIMATION  5U      // every 5th packet -> 200 Hz
```

200 Hz is what `predict.py` was trained on, so if you want to reproduce
its pipeline exactly on BSAU-only data, set DECIMATION=5 and point the
same script at `/dev/ttyACM0`. At 1 kHz (default) you get 5× oversampling,
useful for training a wider-bandwidth model later.

#### What to expect on the wire

- UART at 921600 baud 8N1, no flow control (same as all other modes).
- First useful line arrives ~10 ms after BSAU_Init completes (LOG
  messages during init also go out this UART, but they start with `[BSAU`
  and the collector script skips any line that doesn't parse as 8 ints).
- A gap in the `idx` counter (exposed to the collector via line count)
  means the radio TX stalled for longer than 1 ms — look at CPCU-side
  `tx_retry` and `pkt_loss` to correlate.
- Bad lines (wrong column count) are almost always a baud mismatch or a
  USB-serial adapter that truncates at framing errors.

#### What NOT to do

- Do not use DATASET mode in production. The CSV emit path is on the hot
  loop and doubles the CPU duty cycle from ~54 % to ~60 %.
- Do not run DATASET against a *disconnected* CPCU for long periods —
  every packet goes through the full 15-retry NRF sequence (12 ms
  worst case), which eventually blows the 1 ms packet period and the
  CSV stream falls behind. Use a dummy receiver or move back to
  BSAU_MODE_TEST_CSV for radio-free capture.
- Do not enable both DATASET and the DEBUG-mode decimated CSV by accident
  — they share the UART and two concurrent writers will produce torn
  lines. The `#if defined(BSAU_MODE_DEBUG)` guard in `bsau_app.c`
  enforces this at compile time; don't remove it.

---

## 13. Error Handling Chain

| Error | Detection | BSAU action | CPCU visibility |
|-------|-----------|-------------|-----------------|
| ADC overrun | `HAL_ADC_ErrorCallback` ISR | Store `g_adc_error_code`, clear in main loop | `WL_FLAG_ADC_OVRN` in flags |
| DMA snapshot overwrite | `g_pkt_ready` still 1 in ISR | Increment `g_adc_dropped` | Detectable via timestamp jitter |
| NRF not detected | `NRF_Init()` readback fail | Retry once, then `Error_Handler()` | Board never transmits |
| NRF MAX_RT | `NRF_Transmit()` returns error | `NRF_FlushTX()`, increment `lost_count` | `seq` gap + `pkt_loss` increment |
| NRF timeout | Poll exceeds 20 ms | `NRF_FlushTX()` | `seq` gap |
| NRF FIFO full | FIFO_STATUS check | Return error | `WL_FLAG_TX_SAT` |
| Battery low | `vbat_raw < 1861` | Set `WL_BATT_LOW` | Flags + vbat_raw |
| Battery critical | `vbat_raw < 1675` | Set `WL_BATT_CRITICAL` | Flags + vbat_raw |
| Clipping | ch == 0x000 or 0xFFF | Set `WL_FLAG_CLIPPING` | Flags byte |
| SPI failure | HAL return != OK | `NRF_ERR_SPI` | `seq` gap |

Battery voltage thresholds (100k/100k divider):

```
V_batt = ADC_raw × VDDA / 4095 × 2

LOW threshold (1861):      V = 1861 × 3.3/4095 × 2 = 2.998 V ≈ 3.0 V
CRITICAL threshold (1675): V = 1675 × 3.3/4095 × 2 = 2.699 V ≈ 2.7 V
```

---

## 14. CPCU Receiver Side (Raspberry Pi 5)

### 14.1 Hardware Configuration (/boot/firmware/config.txt)

```
dtparam=pciex1_gen=3            PCIe Gen 3 for AI co-processors
arm_freq=2800                   Overclock Cortex-A76 (2.4 → 2.8 GHz)
dtparam=i2c_arm_baudrate=400000 I2C Fast Mode (400 kHz) for servo drivers
dtoverlay=disable-bt            Free Bluetooth kernel interrupts
dtparam=fan_temp0=10000         Force max fan speed
dtparam=fan_temp0_speed=255
```

### 14.2 Kernel Isolation (/boot/firmware/cmdline.txt)

```
isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3
```

- `isolcpus` — walls off cores 1–3 from CFS. No process scheduled unless pinned.
- `nohz_full` — disables 1 kHz tick on isolated cores (tickless execution).
- `rcu_nocbs` — moves RCU callbacks off isolated cores.

### 14.3 Core Allocation

```
Core 0:  Linux (CFS scheduler)
         ├─ Networking, SSH, daemons, filesystem I/O
         └─ Non-real-time housekeeping

Core 1:  DSP / AI — SMP pair (isolated, tickless)
Core 2:  DSP / AI — SMP pair (isolated, tickless)
         ├─ TensorFlow Lite inference (gesture classification)
         ├─ Feature extraction: RMS, MAV, waveform length, ZCR
         ├─ Goertzel DFT / spectral analysis
         └─ Sliding window: 200 ms (400 samples), 50 ms stride

Core 3:  Real-time I/O controller (isolated, tickless)
         ├─ NRF24L01 SPI driver (busy-poll NRF_DataAvailable)
         ├─ WL_Unpack → seq check → link quality → ring buffer push
         ├─ Servo control via I2C (PCA9685, 50 Hz update)
         ├─ Safety monitor: link timeout, battery, emergency stop
         └─ State machine: idle → armed → active → error → recovery
```

### 14.4 Core 3 Loop (Pseudo-code)

```
loop:
  1. Poll NRF_DataAvailable() — if no data, goto step 7
  2. NRF_ReadPayload() → 32 bytes
  3. WL_Unpack() → WL_Packet
  4. Sequence check: detect gaps, count lost packets
  5. Link quality: accumulate tx_retry, compute pkt_loss delta,
     timestamp jitter stats, battery voltage filter
  6. Push 8ch × 2 samples to lock-free SPSC ring buffer
  7. Read ML classification from shared atomic (written by Cores 1-2)
  8. Update servo positions via I2C if safety OK
  9. Watchdog: if no packet for > 50 ms → LINK_LOST state
  goto loop
```

### 14.5 DSP/ML Pipeline (Cores 1–2)

```
  1. Consume samples from SPSC ring buffer (spin-wait on isolated core)
  2. Per-channel bandpass: 20–450 Hz (2nd order HP + 4th order LP Butterworth)
  3. 50 Hz notch filter (power line, Q=30)
  4. Feature extraction per 200 ms window:
     RMS, MAV, waveform length, ZCR, mean frequency (Goertzel),
     spectral power in 4 sub-bands (20–60, 60–120, 120–250, 250–450 Hz)
  5. Feature vector: 8 channels × 8 features = 64 elements
  6. TF Lite INT8 inference → gesture class probabilities
  7. Post result to shared atomic for Core 3
```

### 14.6 Dataset Capture Path (CPCU TUI, Page 7)

Runs on Core 0 alongside the regular TUI render. Reads the sensor ring
buffer WITHOUT advancing the tail (the Python DSP process is still the
sole SPSC consumer), so the capture path is a passive tap — zero effect
on the real-time I/O or inference loop.

```
  1. User cycles through CLS_NAMES[] labels with <- / ->
  2. User presses 's' or SPACE. The TUI:
       a. sanitises the label to a filesystem-safe token
       b. scans <out_dir> for existing <label>_<N>.csv and picks next N
       c. opens <label>_<N>.csv, resets the 8-channel SOS filter state
       d. snapshots the current ring-head as its local "tail"
  3. Every frame (10 Hz), ds_tick() drains all new ring entries. For each
     entry, for each of the 2 scans, for each of 8 channels:
       raw -> volts (scale 3.3/4095, subtract 1.65V DC bias)
       volts -> filtered via DF-II-transposed SOS cascade
                (4 biquads for 20-450Hz bandpass + 1 biquad for 50Hz notch)
       one CSV row per scan written directly to the file
       -> 2 rows per packet, 2000 rows/s at full link rate
  4. User presses 's' or SPACE again -> fflush/fclose, show sample count
     and any seq-gap count (samples that would have been there if the
     air link hadn't dropped packets).
  5. User presses 'r' while collecting -> fclose + unlink (cancel).
  6. 't' before start toggles RAW-only vs FILTERED-only output.
```

Filter SOS coefficients are precomputed offline with
`scipy.signal.butter(4, [20/1000, 450/1000], btype='band', output='sos')`
and `iirnotch(50, 30, 2000)`, baked into `g_ds_sos[]` in `cpcu_tui.c`.
They are numerically identical to the coefficients used at inference
time in `cpcu_dsp.py` — if either side's coefficients are regenerated,
the other must be updated in the same commit or a subtle distribution
shift will appear between training-time and runtime features.

A BSAU-side capture (via `bsau_dataset_collector.py` on USART1) and a
CPCU-side capture (via TUI Page 7) of the same muscle contraction
produce two CSV files with the same 8-column shape and no header. The
BSAU file contains raw 12-bit ADC ints; the CPCU file contains either
raw ints (RAW mode) or filtered floats in volts (FILTERED mode). Column
order and row count convention match — row N of both files is the same
logical sample, modulo over-the-air packet loss.

---

## 15. Future Optimisation Opportunities

### 15.1 IRQ-Driven NRF Transmit

Replace blocking STATUS poll with NRF_IRQ EXTI + WFI. CPU active drops from
545 µs to ~84 µs per packet. Duty cycle: 8.4%. MCU current drops to ~2.55 mA.
System saving: ~0.63 mA (8%).

### 15.2 Voltage Scaling Range 2

At 26 MHz (Range 2): Run current ~1.2 mA, Sleep ~0.68 mA. But ADC at 26 MHz gives
t_conv = 2.31 µs → t_scan(32×) = 665 µs > 500 µs budget. Requires reducing OS to 16×
or sample rate to 1 kHz. Not viable for current 2 kHz / 32× spec.

### 15.3 Frequency Hopping

Use PLOS_CNT > threshold to trigger channel change. Requires predefined hop table
and CPCU-side synchronisation via FIRST_PACKET flag.

### 15.4 ACK Payload (Downlink Commands)

FEATURE = 0x02, DYNPD = 0x01 enables ACK payloads from CPCU → BSAU. Use cases:
gain adjust, calibration trigger, shutdown, sample rate change. ARD=500µs is sufficient
for up to 32-byte ACK payload at 2 Mbps.

### 15.5 DMA-Based SPI

DMA for the 33-byte NRF payload write, freeing CPU during ~53 µs. Marginal saving
(~0.05 mA) — only worthwhile alongside §15.1.
