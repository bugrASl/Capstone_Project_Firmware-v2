/**
 *  @file       bsau_app.c
 *  @brief      BSAU application module — implementation
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.1
 *  @details
 *                          What this file owns
 *              ────────────────────────────────────────────────────────────────
 *              BSAU_Init()     NVIC sanity, TIM2 timestamp counter, NRF init
 *                              with retry, ADC pipeline bring-up.
 *              BSAU_Run()      WFI -> WL_Packet fill (v2.1 format) -> WL_Pack
 *                              -> NRF_Transmit. Optional per-packet CSV emit
 *                              in DATASET mode, optional decimated CSV emit
 *                              in DEBUG mode.
 *
 *                          Packet assembly stride (critical)
 *              ────────────────────────────────────────────────────────────────
 *              The ADC DMA buffer contains ADC_DMA_CHANNELS values per scan
 *              (= 9: 8 EMG on PA0-PA7 + 1 battery on PB0). WL_NUM_CHANNELS
 *              is 8. The main loop indexes:
 *                  g_adc_snapshot[s * ADC_DMA_CHANNELS + c]    EMG ch c of scan s
 *                  g_adc_snapshot[s * ADC_DMA_CHANNELS + ADC_BATT_INDEX]
 *                                                              battery of scan s
 *              so stride is ADC_DMA_CHANNELS (= 9), NOT WL_NUM_CHANNELS.
 *
 *                          v2.1 packet fields
 *              ────────────────────────────────────────────────────────────────
 *              wireless_packet v2.1 replaces v1 reserved[] padding with:
 *                  tx_retry    ARC_CNT from PREVIOUS transmit (early-warning)
 *                  pkt_loss    PLOS_CNT from PREVIOUS transmit (cumulative)
 *                  timestamp   TIM2 1-MHz counter low 16 bits (jitter detect)
 *                  vbat_raw    12-bit averaged battery ADC reading
 *              and replaces the v1 bit-flag WL_FLAG_BATT_LOW with a 2-bit
 *              battery-level field in flags (WL_BATT_OK / LOW / CRIT / CHARG,
 *              set via WL_BATT_SET).
 *
 *              v2.1 changes:
 *                  - Deleted the stale v1-cutover TODO block at top of v2.0.
 *                    The claimed "6 EMG + batt" scan is no longer reality;
 *                    adc.c was widened to 9 channels in v2.0 (ranks 1-8 =
 *                    PA0-PA7, rank 9 = PB0). ch[6..7] are now real EMG data.
 *                  - Style polish: 98-char banners, column-aligned
 *                    assignments, Allman braces throughout.
 */

#include "bsau_app.h"

#include "bsau_adc.h"
#include "log.h"

#if defined(BSAU_MODE_TEST_ADC_CSV) || defined(BSAU_MODE_TEST_PKT_LOG) || \
    defined(BSAU_MODE_TEST_CSV)     || defined(BSAU_MODE_TEST_DFT_LOG) || \
    defined(BSAU_MODE_TEST_NRF_LOG)
    #include "bsau_test.h"
#endif

#include "nrf24l01.h"
#include "wireless_packet.h"
#include "spi.h"
#include "usart.h"
#include "tim.h"
#include "main.h"

#include <stdbool.h>
#include <string.h>

/*============= MODULE-PRIVATE STATE ===========================================================*/

static NRF_Handle           g_hnrf;

/*
 *  Periodic LOG cadence. At ~1000 pkt/s (2 Mbps air rate), 329 packets
 *  ≈ 329 ms -- about three status lines per second during DEBUG runs.
 */
#define LOG_STATS_INTERVAL          329U

/*
 *  DEBUG-only decimated CSV cadence for SerialPlot capture. In DEBUG mode
 *  (LOG on AND LOG_CSV on) we emit one CSV line every N packets for live
 *  plotting alongside the structured log. In RELEASE and DATASET modes,
 *  this block is compiled out (see the #if guard at the call site).
 */
#define LOG_CSV_DEBUG_INTERVAL      67U

/*
 *  NRF Power-On Reset timing (datasheet 6.1.7): up to 100 ms from VDD
 *  ramp. The 5 ms delay inside NRF_Init() is for CE->Standby, not POR.
 *  On fast-booting MCUs (L432KC @ 80 MHz), BSAU_Init() may reach NRF_Init
 *  before the NRF chip is ready, so we wait explicitly here.
 */
#define NRF_POR_DELAY_MS            200U
#define NRF_INIT_RETRIES            2U
#define NRF_RETRY_BACKOFF_MS        100U

/*============= BSAU_Init ======================================================================*/

void BSAU_Init(void)
{
    /*
     *  TEST MODE: delegate to the test harness.
     *  NOTE: BSAU_MODE_DATASET is NOT delegated — it uses the normal init
     *  path (NRF + ADC) and only adds a per-packet CSV emit to BSAU_Run.
     */
#if defined(BSAU_MODE_TEST_ADC_CSV) || defined(BSAU_MODE_TEST_PKT_LOG) || \
    defined(BSAU_MODE_TEST_CSV)     || defined(BSAU_MODE_TEST_DFT_LOG) || \
    defined(BSAU_MODE_TEST_NRF_LOG)
    BSAU_Test_Init();
    return;
#endif

    LOG("APP", "BSAU_Init", "RUN", "");

    assert_param(HAL_NVIC_GetPriorityGrouping() == NVIC_PRIORITYGROUP_4);
    LOG("APP", "BSAU_Init", "OK", "NVIC priority group verified");

    /*
     *  TIM2 as free-running 1-MHz timestamp counter (v2.1 packet field).
     *  PSC = 79, ARR = 0xFFFFFFFF -> 1 µs tick, ~71.6 min wrap.
     *  See BSAU_ARCHITECTURE.md §3.4.
     */
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
        LOG("APP", "TIM2_Start", "FAIL", "HAL_TIM_Base_Start(&htim2)");
        Error_Handler();
    }
    LOG("APP", "TIM2_Start", "OK", "1 MHz free-running counter live");

    /*-------------- NRF initialization with retry ---------------------------------------------*/
    static const uint8_t nrf_addr[NRF_ADDR_WIDTH]   =   NRF_ADDRESS;

    /* POR delay — wait for chip to finish internal reset after VDD ramp */
    HAL_Delay(NRF_POR_DELAY_MS);
    LOG("NRF", "NRF_Init", "RUN", "ch=%u (POR wait %ums done)",
        NRF_CHANNEL, NRF_POR_DELAY_MS);

    NRF_Status nrf_ret                              =   NRF_ERR_NOT_DETECTED;

    for (uint8_t attempt = 0; attempt < NRF_INIT_RETRIES; attempt++)
    {
        nrf_ret                                     =   NRF_Init(&g_hnrf, &hspi1,
                                                                 NRF_CHANNEL, nrf_addr);
        if (nrf_ret == NRF_OK)
        {
            break;
        }

        LOG("NRF", "NRF_Init", "WARN",
            "Attempt %u failed (err=%d), retrying in %ums...",
            attempt + 1, nrf_ret, NRF_RETRY_BACKOFF_MS);
        HAL_Delay(NRF_RETRY_BACKOFF_MS);
    }

    if (nrf_ret != NRF_OK)
    {
        LOG("NRF", "NRF_Init", "FAIL", "All %u attempts failed", NRF_INIT_RETRIES);
        Error_Handler();
    }

    LOG("NRF", "NRF_Init", "OK", "");

    /*-------------- ADC pipeline --------------------------------------------------------------*/
    BSAU_ADC_Init();

    LOG("APP", "BSAU_Init", "OK", "Pipeline live");
}

/*============= BSAU_Run =======================================================================*/

void BSAU_Run(void)
{
    /*
     *  TEST MODE: delegate to the test harness — never fall through.
     *  DATASET mode falls through and shares this loop with RELEASE/DEBUG.
     */
#if defined(BSAU_MODE_TEST_ADC_CSV) || defined(BSAU_MODE_TEST_PKT_LOG) || \
    defined(BSAU_MODE_TEST_CSV)     || defined(BSAU_MODE_TEST_DFT_LOG) || \
    defined(BSAU_MODE_TEST_NRF_LOG)
    BSAU_Test_Run();
    return;
#endif

    /*-------------- RELEASE / DEBUG / DATASET main loop ---------------------------------------*/
    WL_Packet       pkt;
    uint8_t         raw[WL_PAYLOAD_SIZE];

    uint8_t         tx_seq              =   0;
    uint32_t        pkt_count           =   0;
    uint32_t        lost_count          =   0;

    /*
     *  Link-quality stash. OBSERVE_TX[3:0] = ARC_CNT (retries for last TX),
     *  OBSERVE_TX[7:4] = PLOS_CNT (cumulative, saturating). These describe
     *  the transmit that JUST happened, so we carry them into the NEXT
     *  packet — the arch doc §10.2 explains why this one-packet delay is
     *  exactly what the CPCU-side early-warning detector wants.
     */
    uint8_t         prev_retry          =   0;
    uint8_t         prev_loss           =   0;
    bool            first_packet        =   true;

    LOG("APP", "BSAU_Run", "RUN", "Entering main loop");

    while (1)
    {
        __WFI();

        if (!g_pkt_ready)
        {
            continue;
        }

        g_pkt_ready                     =   0;

        /* Deferred ADC error from ISR (we never LOG from ISR context). */
        if (g_adc_error_code != 0)
        {
            LOG("ADC", "DeferredErr", "FAIL",
                "err=0x%08X", (unsigned int)g_adc_error_code);
            g_adc_error_code            =   0;
        }

        /*-------------- Metadata --------------------------------------------------------------*/
        pkt.seq                         =   tx_seq++;
        pkt.flags                       =   0;
        pkt.tx_retry                    =   prev_retry;
        pkt.pkt_loss                    =   prev_loss;
        pkt.timestamp                   =   (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFu);

        uint16_t vbat                   =   BSAU_ADC_GetBattery();
        pkt.vbat_raw                    =   vbat;

        /* Battery level into 2 low bits of flags (WL_BATT_SET preserves the rest). */
        if      (vbat < BATT_CRITICAL_THRESHOLD)
        {
            pkt.flags                   =   WL_BATT_SET(pkt.flags, WL_BATT_CRIT);
        }
        else if (vbat < BATT_LOW_THRESHOLD)
        {
            pkt.flags                   =   WL_BATT_SET(pkt.flags, WL_BATT_LOW);
        }
        else
        {
            pkt.flags                   =   WL_BATT_SET(pkt.flags, WL_BATT_OK);
        }

        /* First-packet flag — lets CPCU reset expected_seq cleanly. */
        if (first_packet)
        {
            pkt.flags                  |=   WL_FLAG_FIRST_PACKET;
            first_packet                =   false;
        }

        /*-------------- EMG samples (stride = ADC_DMA_CHANNELS) -------------------------------*/
        for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        {
            for (int c = 0; c < WL_NUM_CHANNELS; c++)
            {
                pkt.samples[s].ch[c]    =   g_adc_snapshot[s * ADC_DMA_CHANNELS + c];
            }
        }

        /* Clipping detection (12-bit rails at 0 and 0xFFF). */
        for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        {
            for (int c = 0; c < WL_NUM_CHANNELS; c++)
            {
                uint16_t v              =   pkt.samples[s].ch[c];
                if (v == 0u || v >= 4095u)
                {
                    pkt.flags          |=   WL_FLAG_CLIPPING;
                    s                   =   WL_SAMPLES_PER_PACKET;      /* break outer */
                    break;
                }
            }
        }

        /*-------------- Encode and transmit ---------------------------------------------------*/
        WL_Pack(&pkt, raw);
        NRF_Status status               =   NRF_Transmit(&g_hnrf, raw);

        /* Read OBSERVE_TX for the NEXT packet's tx_retry / pkt_loss. */
        NRF_GetTxStats(&g_hnrf, &prev_loss, &prev_retry);

        if (status == NRF_ERR_TX_MAX_RT)
        {
            NRF_FlushTX(&g_hnrf);
            lost_count++;
        }

        pkt_count++;

#if defined(BSAU_MODE_DATASET)
        /*
         *  DATASET mode — one CSV line per packet using scan 0 only.
         *
         *  Wire format (per DSP/AI team request: just the channels):
         *      c0,c1,c2,c3,c4,c5,c6,c7\r\n
         *
         *  Mirrors the 3-column format used in predict.py, widened to 8
         *  columns for the full BSAU channel set. No index, no flags,
         *  no battery, no drop counter — the collector script on the PC
         *  side adds whatever metadata it wants (wall-clock time, line
         *  number, chosen label) to the file it writes.
         *
         *  Sample rate = 1000 Hz per channel (scan 0 only). Line size
         *  ≤ 42 B -> ~420 kbps ≈ 45 % of 921600 baud. UART budget proof
         *  lives in log.h §5.
         *
         *  If you ever want the full 2 kHz stream, emit scan 1 too — but
         *  measure UART backpressure first.
         */
        if (pkt_count % BSAU_DATASET_CSV_DECIMATION == 0)
        {
            LOG_CSV("%u,%u,%u,%u,%u,%u,%u,%u",
                    pkt.samples[0].ch[0], pkt.samples[0].ch[1],
                    pkt.samples[0].ch[2], pkt.samples[0].ch[3],
                    pkt.samples[0].ch[4], pkt.samples[0].ch[5],
                    pkt.samples[0].ch[6], pkt.samples[0].ch[7]);
        }
#endif

        /* Periodic structured status log (LOG channel, DEBUG only). */
        if (pkt_count % LOG_STATS_INTERVAL == 0)
        {
            LOG("APP", "BSAU_Run", "INFO",
                "seq=%u batt=%u lvl=%u retry=%u loss=%u ok=%lu lost=%lu drop=%lu",
                pkt.seq,
                vbat,
                (unsigned)WL_BATT_GET(pkt.flags),
                prev_retry,
                prev_loss,
                (unsigned long)(pkt_count - lost_count),
                (unsigned long)lost_count,
                (unsigned long)g_adc_dropped);
        }

#if defined(BSAU_MODE_DEBUG)
        /*
         *  DEBUG mode only: decimated multi-column CSV for SerialPlot.
         *  Kept OFF in DATASET mode so the UART carries only the clean
         *  8-channel stream.
         */
        if (pkt_count % LOG_CSV_DEBUG_INTERVAL == 0)
        {
            LOG_CSV("%lu,%u,%u,%u,%u,%u,%u,%u",
                    (unsigned long)pkt_count,
                    pkt.samples[0].ch[0],
                    pkt.samples[0].ch[1],
                    pkt.samples[0].ch[2],
                    pkt.samples[0].ch[3],
                    pkt.samples[0].ch[4],
                    pkt.samples[0].ch[5],
                    vbat);
        }
#endif
    }
}

/*==============================================================================================*/
