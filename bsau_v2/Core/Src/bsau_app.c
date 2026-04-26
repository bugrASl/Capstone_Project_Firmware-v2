/**
 *  @file       bsau_app.c
 *  @brief      BSAU application module — implementation
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.4
 *  @details
 *                          What this file owns
 *              ────────────────────────────────────────────────────────────────
 *              BSAU_Init()     NVIC sanity, TIM2 timestamp counter, NRF init
 *                              with retry, ADC pipeline bring-up.
 *                              v2.4: NRF init failure is no longer fatal —
 *                              the chip is marked offline, the system boots
 *                              anyway, and the periodic health check in
 *                              BSAU_Run brings the radio up as soon as it
 *                              becomes reachable.
 *              BSAU_Run()      WFI -> WL_Packet fill (v2.1 format) -> WL_Pack
 *                              -> NRF_Transmit. Optional per-packet CSV emit
 *                              in DATASET mode, optional decimated CSV emit
 *                              in DEBUG mode. The radio path is gated by
 *                              g_nrf_alive: if the chip is dead, packets
 *                              are still assembled (so DATASET CSV keeps
 *                              flowing) but TX is skipped until recovery.
 *
 *                          v2.4 changes (April 2026)
 *              ────────────────────────────────────────────────────────────────
 *              1.  Non-fatal NRF init. Previous behaviour was to call
 *                  Error_Handler() (which __disable_irq + while(1)) when all
 *                  init retries failed. The system now keeps running and
 *                  retries the chip from BSAU_Run's existing health check
 *                  loop, so a board that powered up before the radio rail
 *                  finished settling, or with a transiently faulty SPI bus,
 *                  will recover automatically once conditions clear.
 *              2.  Single recovery helper nrf_try_recover() shared by every
 *                  profile (DATASET / RELEASE / DEBUG). The previous
 *                  inline block was duplicated logic and skipped some
 *                  edge cases (e.g. didn't invalidate prev_loss/prev_retry
 *                  after recovery so OBSERVE_TX cruft from before the
 *                  reset leaked into the first new packet).
 *              3.  TX path is gated by g_nrf_alive; profile-specific
 *                  blocks (DATASET vs RELEASE/DEBUG) just choose between
 *                  blocking and non-blocking transmit, the rest of the
 *                  loop (sample assembly, CSV emit, stats logging,
 *                  health check) is profile-independent.
 *
 *                          v2.2 changes (April 2026)
 *              ────────────────────────────────────────────────────────────────
 *              Fixes the "DATASET-mode UART freezes when CPCU enters SAFE"
 *              bug. The fix in DATASET mode is twofold:
 *                  1. Emit the CSV line BEFORE the radio TX, so UART output
 *                     is never gated by the radio.
 *                  2. Use NRF_TransmitNoBlock — fire-and-forget. The TX
 *                     FIFO is 3-deep, well within budget at 1 kHz packet
 *                     rate even with 100 % MAX_RT.
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

/*  v2.4: liveness flag drives whether the TX path runs. The bring-up /
 *  recovery helpers below are the only writers; everything else reads.   */
static bool                 g_nrf_alive                 =   false;

/*  v2.3: nrf_addr is file-scope so the periodic health check / recovery
 *  helper can re-init the chip if the user yanks the nRF module and
 *  plugs it back in.                                                     */
static const uint8_t        nrf_addr[NRF_ADDR_WIDTH]    =   NRF_ADDRESS;

/*  Periodic LOG cadence. At ~1000 pkt/s, 329 packets ≈ 329 ms.           */
#define LOG_STATS_INTERVAL          329U

/*  DEBUG-only decimated CSV cadence for SerialPlot capture.              */
#define LOG_CSV_DEBUG_INTERVAL      67U

/*  NRF Power-On Reset timing (datasheet 6.1.7).                          */
#define NRF_POR_DELAY_MS            200U
#define NRF_INIT_RETRIES            2U
#define NRF_RETRY_BACKOFF_MS        100U

/*  v2.3: how often the BSAU_Run loop sanity-checks the nRF chip.
 *  Cost: one ReadReg every N packets. At 1 kHz packet rate and N=500,
 *  that's ~2 SPI reads per second.                                       */
#define NRF_HEALTH_CHECK_INTERVAL   500U

/*============= NRF BRING-UP HELPERS (file-local) ==============================================*/
/**
 *  Single-shot init with bounded retry. Updates g_nrf_alive in place.
 *  Returns the final NRF_Status — caller decides whether the result is
 *  fatal (initial bring-up: no longer is, v2.4), or just informational
 *  (BSAU_Run health check: log and try again next interval).
 */
static NRF_Status nrf_bringup(void)
{
    NRF_Status      ret             =   NRF_ERR_NOT_DETECTED;

    for (uint8_t attempt = 0; attempt < NRF_INIT_RETRIES; attempt++)
    {
        ret                         =   NRF_Init(&g_hnrf, &hspi1,
                                                 NRF_CHANNEL, nrf_addr);
        if (ret == NRF_OK)
        {
            g_nrf_alive             =   true;
            return ret;
        }

        LOG("NRF", "NRF_Init", "WARN",
            "Attempt %u failed (err=%d), retrying in %ums...",
            attempt + 1U, ret, NRF_RETRY_BACKOFF_MS);
        HAL_Delay(NRF_RETRY_BACKOFF_MS);
    }

    g_nrf_alive                     =   false;
    return ret;
}

/**
 *  Cold-recovery: drain FIFOs, force PWR_DOWN, wait for POR, run nrf_bringup.
 *  Used by both BSAU_Init's initial attempt and BSAU_Run's periodic health
 *  check. Idempotent — safe to call repeatedly.
 */
static NRF_Status nrf_try_recover(void)
{
    ce_low();
    NRF_FlushTX(&g_hnrf);
    NRF_FlushRX(&g_hnrf);
    NRF_PowerDown(&g_hnrf);
    HAL_Delay(NRF_POR_DELAY_MS);

    return nrf_bringup();
}

/**
 *  Read RF_CH and CONFIG and decide whether the chip is in the state we
 *  programmed. Returns true if healthy, false if recovery is needed.
 *
 *  After a chip power-cycle (jumper pull, brownout) factory defaults are
 *  RF_CH=2 and CONFIG=0x08 (PWR_UP=0). If the chip is missing entirely,
 *  MISO floats and we read 0xFF.
 */
static bool nrf_is_healthy(void)
{
    uint8_t cur_ch                  =   NRF_ReadReg(&g_hnrf, NRF_REG_RF_CH);
    uint8_t cur_cfg                 =   NRF_ReadReg(&g_hnrf, NRF_REG_CONFIG);

    bool chip_gone                  =   (cur_ch == 0xFFu) || (cur_cfg == 0xFFu);
    bool wrong_channel              =   (cur_ch  != NRF_CHANNEL);
    bool not_powered                =   !(cur_cfg & 0x02u);     /* bit1 = PWR_UP */

    if (chip_gone || wrong_channel || not_powered)
    {
        LOG("NRF", "Health", "FAIL",
            "ch=0x%02X cfg=0x%02X (expect ch=%u, PWR_UP=1)",
            cur_ch, cur_cfg, (unsigned)NRF_CHANNEL);
        return false;
    }
    return true;
}

/*============= BSAU_Init ======================================================================*/

void BSAU_Init(void)
{
    /*  TEST MODE: delegate to the test harness.
     *  NOTE: BSAU_MODE_DATASET is NOT delegated.                           */
#if defined(BSAU_MODE_TEST_ADC_CSV) || defined(BSAU_MODE_TEST_PKT_LOG) || \
    defined(BSAU_MODE_TEST_CSV)     || defined(BSAU_MODE_TEST_DFT_LOG) || \
    defined(BSAU_MODE_TEST_NRF_LOG)
    BSAU_Test_Init();
    return;
#endif

    LOG("APP", "BSAU_Init", "RUN", "");

    assert_param(HAL_NVIC_GetPriorityGrouping() == NVIC_PRIORITYGROUP_4);
    LOG("APP", "BSAU_Init", "OK", "NVIC priority group verified");

    /*  TIM2 as free-running 1-MHz timestamp counter. */
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
        LOG("APP", "TIM2_Start", "FAIL", "HAL_TIM_Base_Start(&htim2)");
        Error_Handler();
    }
    LOG("APP", "TIM2_Start", "OK", "1 MHz free-running counter live");

    /*-------------- NRF initialization (non-fatal, v2.4) --------------------------------------*/
    /*
     *  POR delay first, then bounded-retry bring-up. If every retry
     *  still fails we *do not* call Error_Handler() — instead we boot
     *  with g_nrf_alive=false and let BSAU_Run's periodic health check
     *  pick the chip up as soon as it becomes reachable. This keeps
     *  ADC + UART + (in DATASET mode) the CSV stream alive even when
     *  the radio rail is misbehaving, and avoids a hard lock that
     *  required a manual power cycle.                                    */
    HAL_Delay(NRF_POR_DELAY_MS);
    LOG("NRF", "NRF_Init", "RUN", "ch=%u (POR wait %ums done)",
        NRF_CHANNEL, NRF_POR_DELAY_MS);

    NRF_Status nrf_ret              =   nrf_bringup();

    if (nrf_ret != NRF_OK)
    {
        LOG("NRF", "NRF_Init", "WARN",
            "All %u attempts failed (err=%d) — booting without radio, "
            "BSAU_Run will retry every %u packets",
            NRF_INIT_RETRIES, nrf_ret, NRF_HEALTH_CHECK_INTERVAL);
    }
    else
    {
        LOG("NRF", "NRF_Init", "OK", "");
    }

    /*-------------- ADC pipeline --------------------------------------------------------------*/
    BSAU_ADC_Init();

    LOG("APP", "BSAU_Init", "OK",
        "Pipeline live (radio %s)", g_nrf_alive ? "up" : "OFFLINE");
}

/*============= BSAU_Run =======================================================================*/

void BSAU_Run(void)
{
    /*  TEST MODE: delegate to the test harness.
     *  DATASET mode falls through and shares this loop with RELEASE/DEBUG. */
#if defined(BSAU_MODE_TEST_ADC_CSV) || defined(BSAU_MODE_TEST_PKT_LOG) || \
    defined(BSAU_MODE_TEST_CSV)     || defined(BSAU_MODE_TEST_DFT_LOG) || \
    defined(BSAU_MODE_TEST_NRF_LOG)
    BSAU_Test_Run();
    return;
#endif

    /*-------------- RELEASE / DEBUG / DATASET main loop ---------------------------------------*/
    WL_Packet       pkt;
    uint8_t         raw[WL_PAYLOAD_SIZE];

    uint8_t         tx_seq          =   0;
    uint32_t        pkt_count       =   0;
    uint32_t        lost_count      =   0;

    /*  Link-quality stash, see arch §10.2.                                 */
    uint8_t         prev_retry      =   0;
    uint8_t         prev_loss       =   0;
    bool            first_packet    =   true;

    LOG("APP", "BSAU_Run", "RUN",
        "Entering main loop (radio %s)", g_nrf_alive ? "up" : "OFFLINE");

    while (1)
    {
        __WFI();

        if (!g_pkt_ready)
        {
            continue;
        }

        g_pkt_ready                 =   0;

        /*  Deferred ADC error from ISR (we never LOG from ISR context). */
        if (g_adc_error_code != 0)
        {
            LOG("ADC", "DeferredErr", "FAIL",
                "err=0x%08X", (unsigned int)g_adc_error_code);
            g_adc_error_code        =   0;
        }

        /*-------------- Metadata --------------------------------------------------------------*/
        pkt.seq                     =   tx_seq++;
        pkt.flags                   =   0;
        pkt.tx_retry                =   prev_retry;
        pkt.pkt_loss                =   prev_loss;
        pkt.timestamp               =   (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFu);

        uint16_t vbat               =   BSAU_ADC_GetBattery();
        pkt.vbat_raw                =   vbat;

        /*  Battery level into 2 low bits of flags. */
        if      (vbat < BATT_CRITICAL_THRESHOLD)
            pkt.flags               =   WL_BATT_SET(pkt.flags, WL_BATT_CRIT);
        else if (vbat < BATT_LOW_THRESHOLD)
            pkt.flags               =   WL_BATT_SET(pkt.flags, WL_BATT_LOW);
        else
            pkt.flags               =   WL_BATT_SET(pkt.flags, WL_BATT_OK);

        /*  First-packet flag — lets CPCU reset expected_seq cleanly. */
        if (first_packet)
        {
            pkt.flags              |=   WL_FLAG_FIRST_PACKET;
            first_packet            =   false;
        }

        /*-------------- EMG samples (stride = ADC_DMA_CHANNELS) -------------------------------*/
        for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        {
            for (int c = 0; c < WL_NUM_CHANNELS; c++)
            {
                pkt.samples[s].ch[c] =  g_adc_snapshot[s * ADC_DMA_CHANNELS + c];
            }
        }

        /*  Clipping detection (12-bit rails). */
        for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        {
            for (int c = 0; c < WL_NUM_CHANNELS; c++)
            {
                uint16_t v          =   pkt.samples[s].ch[c];
                if (v == 0u || v >= 4095u)
                {
                    pkt.flags      |=   WL_FLAG_CLIPPING;
                    s               =   WL_SAMPLES_PER_PACKET;      /* break outer */
                    break;
                }
            }
        }

        /*-------------- Encode ----------------------------------------------------------------*/
        WL_Pack(&pkt, raw);

        /*-------------- DATASET-only: emit CSV BEFORE radio TX --------------------------------*/
        /*  v2.2: UART output must not be gated by the radio link.        */
#if defined(BSAU_MODE_DATASET)
        if (pkt_count % BSAU_DATASET_CSV_DECIMATION == 0)
        {
            LOG_CSV("%u,%u,%u,%u,%u,%u,%u,%u",
                    pkt.samples[0].ch[0], pkt.samples[0].ch[1],
                    pkt.samples[0].ch[2], pkt.samples[0].ch[3],
                    pkt.samples[0].ch[4], pkt.samples[0].ch[5],
                    pkt.samples[0].ch[6], pkt.samples[0].ch[7]);
        }
#endif

        /*-------------- Radio transmit (gated by liveness) ------------------------------------*/
        /*
         *  v2.4: profile picks blocking vs non-blocking transmit. The
         *  outer g_nrf_alive guard is shared so a dead radio short-
         *  circuits the TX in every profile and we never block the
         *  loop on an absent chip.
         */
        if (g_nrf_alive)
        {
#if defined(BSAU_MODE_DATASET)
            /*  DATASET — fire-and-forget; OBSERVE_TX is one packet stale. */
            NRF_GetTxStats(&g_hnrf, &prev_loss, &prev_retry);
            (void)NRF_TransmitNoBlock(&g_hnrf, raw);
#else
            /*  RELEASE / DEBUG — blocking TX with full retry tracking. */
            NRF_Status status       =   NRF_Transmit(&g_hnrf, raw);
            NRF_GetTxStats(&g_hnrf, &prev_loss, &prev_retry);

            if (status == NRF_ERR_TX_MAX_RT)
            {
                NRF_FlushTX(&g_hnrf);
                lost_count++;
            }
#endif
        }
        else
        {
            /*  Radio offline — clear stale stats so telemetry doesn't
             *  carry old OBSERVE_TX values from before the failure.    */
            prev_loss               =   0;
            prev_retry              =   0;
        }

        pkt_count++;

        /*-------------- Periodic structured status log (LOG channel) --------------------------*/
        if (pkt_count % LOG_STATS_INTERVAL == 0)
        {
            LOG("APP", "BSAU_Run", "INFO",
                "seq=%u batt=%u lvl=%u retry=%u loss=%u ok=%lu lost=%lu drop=%lu nrf=%s",
                pkt.seq,
                vbat,
                (unsigned)WL_BATT_GET(pkt.flags),
                prev_retry,
                prev_loss,
                (unsigned long)(pkt_count - lost_count),
                (unsigned long)lost_count,
                (unsigned long)g_adc_dropped,
                g_nrf_alive ? "UP" : "DOWN");
        }

        /*-------------- v2.4: NRF chip-presence + recovery ------------------------------------*/
        /*
         *  Two paths:
         *    a) Chip is alive — run the cheap register sanity check; on
         *       mismatch, attempt cold recovery.
         *    b) Chip is dead (init never succeeded, or we lost it
         *       earlier) — try recovery directly. Same cadence either
         *       way so a board that boots without a radio sees the
         *       chip come up exactly NRF_HEALTH_CHECK_INTERVAL packets
         *       after it becomes reachable.
         */
        if (pkt_count % NRF_HEALTH_CHECK_INTERVAL == 0)
        {
            bool need_recover       =   !g_nrf_alive || !nrf_is_healthy();

            if (need_recover)
            {
                NRF_Status r        =   nrf_try_recover();
                if (r == NRF_OK)
                {
                    LOG("NRF", "Health", "OK", "Recovered");
                    /*  Tell CPCU to reset its expected_seq on the next
                     *  packet (avoids one fake gap in stats), and re-
                     *  baseline lost_count so we don't blame this
                     *  outage on the new link.                         */
                    first_packet    =   true;
                    lost_count      =   pkt_count;
                    prev_loss       =   0;
                    prev_retry      =   0;
                }
                else
                {
                    LOG("NRF", "Health", "FAIL",
                        "Recovery returned %d (will retry in %u packets)",
                        r, NRF_HEALTH_CHECK_INTERVAL);
                }
            }
        }

#if defined(BSAU_MODE_DEBUG)
        /*  DEBUG mode only: decimated multi-column CSV for SerialPlot. */
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
