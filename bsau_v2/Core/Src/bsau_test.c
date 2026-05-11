/**
 *  @file   bsau_test.c
 *  @brief  BSAU hardware test suite — ADC, NRF, SPI, GPIO self-tests.
 */

#include "bsau_test.h"

#include "bsau_adc.h"
#include "log.h"
#include "wireless_packet.h"
#include "nrf24l01.h"
#include "nrf24l01_test.h"
#include "spi.h"
#include "usart.h"
#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#if defined(BSAU_MODE_TEST_DFT_LOG)
    #include <math.h>
#endif

/*============= MODULE-PRIVATE STATE ===========================================================*/

#if defined(BSAU_MODE_TEST_PKT_LOG)
static uint32_t         test_pass               =   0;
static uint32_t         test_fail               =   0;
#endif

#if defined(BSAU_MODE_TEST_ADC_CSV)
static uint32_t         g_adc_csv_seq           =   0;
#endif

#if defined(BSAU_MODE_TEST_CSV)
static uint32_t         g_ascii_csv_seq         =   0;
#endif

#if defined(BSAU_MODE_TEST_DFT_LOG)
typedef struct
{
    uint16_t            freq_hz;
    float               coeff;
    float               s1;
    float               s2;
} GoertzelBin;

static GoertzelBin      g_bins[GOERTZEL_NUM_BINS];
static uint32_t         g_dft_count             =   0;
static uint32_t         g_dft_block_num         =   0;
#endif

#if defined(BSAU_MODE_TEST_NRF_LOG)
static NRF_Handle       g_hnrf_test;
static uint32_t         nrf_tx_success          =   0;
static uint32_t         nrf_tx_fail             =   0;
static uint32_t         nrf_tx_start_tick       =   0;
#endif

#if defined(BSAU_MODE_TEST_ADC_CSV) || defined(BSAU_MODE_TEST_CSV) || defined(BSAU_MODE_TEST_DFT_LOG)
static const uint8_t    g_considered_ch[CONSIDERED_CHANNELS_COUNT] = CONSIDERED_CHANNELS;

_Static_assert(CONSIDERED_CHANNELS_COUNT >= 1,
               "CONSIDERED_CHANNELS_COUNT must be at least 1.");
_Static_assert(CONSIDERED_CHANNELS_COUNT <= WL_NUM_CHANNELS,
               "CONSIDERED_CHANNELS_COUNT exceeds WL_NUM_CHANNELS.");
#endif

/*============= PRIVATE: Deferred ADC error check ==============================================*/

static void check_deferred_adc_error(void)
{
    if (g_adc_error_code != 0)
    {
        LOG("ADC", "DeferredErr", "FAIL", "err=0x%08X", (unsigned int)g_adc_error_code);
        g_adc_error_code                        =   0;
    }
}

/*============= BSAU_Test_Init =================================================================*/

void BSAU_Test_Init(void)
{
    LOG("TEST", "BSAU_Test_Init", "RUN", "");

#if defined(BSAU_MODE_TEST_ADC_CSV) || defined(BSAU_MODE_TEST_CSV) || defined(BSAU_MODE_TEST_DFT_LOG)
    BSAU_ADC_Init();
#endif

#ifdef BSAU_MODE_TEST_CSV
    g_ascii_csv_seq                             =   0;

    /* Build CSV header — all columns explicit */
    char hdr[160];
    int  pos                                    =   0;

    pos                                        +=   snprintf(hdr, sizeof(hdr), "seq");

    for (int s = 0; s < ADC_DMA_SAMPLES; s++)
    {
        for (int i = 0; i < CONSIDERED_CHANNELS_COUNT; i++)
        {
            pos                                +=   snprintf(hdr + pos, sizeof(hdr) - pos,
                                                             ",s%dc%u", s, g_considered_ch[i]);
        }
    }

    snprintf(hdr + pos, sizeof(hdr) - pos, ",batt,drop");
    LOG_CSV("%s", hdr);
#endif

#if defined(BSAU_MODE_TEST_ADC_CSV)
    g_adc_csv_seq                               =   0;
#endif

#if defined(BSAU_MODE_TEST_DFT_LOG)
    const uint16_t bin_freqs[]                  =   GOERTZEL_BINS_HZ;

    for (int i = 0; i < GOERTZEL_NUM_BINS; i++)
    {
        g_bins[i].freq_hz                       =   bin_freqs[i];
        float k                                 =   roundf((float)GOERTZEL_BLOCK_SIZE *
                                                           (float)bin_freqs[i] /
                                                           GOERTZEL_FS_HZ);
        g_bins[i].coeff                         =   2.0f * cosf(2.0f * (float)M_PI * k /
                                                                (float)GOERTZEL_BLOCK_SIZE);
        g_bins[i].s1                            =   0.0f;
        g_bins[i].s2                            =   0.0f;
    }

    LOG("TEST", "BSAU_Test_Init", "OK",
        "DFT configured: %d bins, block=%d, fs=%d",
        GOERTZEL_NUM_BINS, GOERTZEL_BLOCK_SIZE, (int)GOERTZEL_FS_HZ);
#endif

#ifdef BSAU_MODE_TEST_PKT_LOG
    test_pass                                   =   0;
    test_fail                                   =   0;
    LOG("TEST", "BSAU_Test_Init", "OK", "Packet codec verify mode ready");
#endif

#ifdef BSAU_MODE_TEST_NRF_LOG
    static const uint8_t nrf_addr[NRF_ADDR_WIDTH]   =   NRF_ADDRESS;

    /* NRF POR delay — datasheet §6.1.7: up to 100 ms from VDD ramp */
    HAL_Delay(200);
    LOG("TEST", "NRF_Test_Init", "RUN", "POR wait done, initializing CH%u", NRF_CHANNEL);

    NRF_Status nrf_ret                          =   NRF_Init(&g_hnrf_test, &hspi1,
                                                             NRF_CHANNEL, nrf_addr);

    if (nrf_ret != NRF_OK)
    {
        LOG("TEST", "NRF_Test_Init", "WARN",
            "first attempt failed (err=%d), retrying...", nrf_ret);
        HAL_Delay(100);
        nrf_ret                                 =   NRF_Init(&g_hnrf_test, &hspi1,
                                                             NRF_CHANNEL, nrf_addr);
    }

    if (nrf_ret != NRF_OK)
    {
        LOG("TEST", "NRF_Test_Init", "FAIL", "NRF_Init failed after retry");
        Error_Handler();
    }

    LOG("TEST", "NRF_Test_Init", "OK", "Radio ready on CH%u", NRF_CHANNEL);

    /* Run full self-test suite before entering the repetitive TX loop */
    TestResult nrf_result                       =   NRF_Test_All(&g_hnrf_test, nrf_addr);
    LOG("TEST", "NRF_SelfTest", nrf_result == TEST_PASS ? "PASS" : "FAIL",
        "Self-test suite %s", nrf_result == TEST_PASS ? "passed" : "FAILED");

    nrf_tx_start_tick                           =   HAL_GetTick();
#endif
}

/*============= ADC BINARY STREAM MODE =========================================================*/

#if defined(BSAU_MODE_TEST_ADC_CSV)

static void test_adc_stream_send(void)
{
    /*
     *  Frame format: [seq_lo][seq_hi] + [ADC_DMA_BUF_SIZE × uint16 LE]
     *  Total = 2 + 36 = 38 bytes (9 ch × 2 samp × 2 B = 36).
     */
    uint8_t frame[2 + ADC_DMA_BUF_SIZE * sizeof(uint16_t)];

    frame[0]                                    =   (uint8_t)( g_adc_csv_seq       & 0xFF);
    frame[1]                                    =   (uint8_t)((g_adc_csv_seq >> 8) & 0xFF);
    g_adc_csv_seq++;

    for (int i = 0; i < ADC_DMA_BUF_SIZE; i++)
    {
        uint16_t val                            =   g_adc_snapshot[i];
        frame[2 + i * 2]                        =   (uint8_t)(val & 0xFF);
        frame[2 + i * 2 + 1]                    =   (uint8_t)(val >> 8);
    }

    HAL_UART_Transmit(&huart1, frame, sizeof(frame), 20);
}

#endif

/*============= CSV COLLECT MODE ===============================================================*/

#if defined(BSAU_MODE_TEST_CSV)

static void test_adc_collect_send(void)
{
    char line[160];
    int  pos                                    =   0;

    pos                                        +=   snprintf(line, sizeof(line), "%lu",
                                                             (unsigned long)g_ascii_csv_seq++);

    for (int s = 0; s < ADC_DMA_SAMPLES; s++)
    {
        for (int i = 0; i < CONSIDERED_CHANNELS_COUNT; i++)
        {
            pos                                +=   snprintf(line + pos, sizeof(line) - pos,
                                                             ",%u",
                                                             g_adc_snapshot[s * ADC_DMA_CHANNELS +
                                                                            g_considered_ch[i]]);
        }
    }

    /* Averaged battery + pipeline health counter */
    uint16_t batt_avg                           =   BSAU_ADC_GetBattery();
    snprintf(line + pos, sizeof(line) - pos, ",%u,%lu",
             batt_avg, (unsigned long)g_adc_dropped);
    LOG_CSV("%s", line);
}

#endif

/*============= DFT VERIFY MODE ================================================================*/

#if defined(BSAU_MODE_TEST_DFT_LOG)

static void dft_feed_sample(uint16_t sample)
{
    float x                                     =   (float)sample;

    for (int i = 0; i < GOERTZEL_NUM_BINS; i++)
    {
        float s0                                =   x + g_bins[i].coeff * g_bins[i].s1
                                                      - g_bins[i].s2;
        g_bins[i].s2                            =   g_bins[i].s1;
        g_bins[i].s1                            =   s0;
    }

    if (++g_dft_count < GOERTZEL_BLOCK_SIZE)
    {
        return;
    }

    /* Block complete — compute magnitudes and find dominant bin */
    float   mags[GOERTZEL_NUM_BINS];
    float   max_mag                             =   0.0f;
    int     max_idx                             =   0;
    float   total_power                         =   0.0f;

    for (int i = 0; i < GOERTZEL_NUM_BINS; i++)
    {
        float mag_sq                            =   g_bins[i].s1 * g_bins[i].s1
                                                  + g_bins[i].s2 * g_bins[i].s2
                                                  - g_bins[i].coeff * g_bins[i].s1 * g_bins[i].s2;
        mags[i]                                 =   sqrtf(mag_sq > 0.0f ? mag_sq : 0.0f);
        total_power                            +=   mags[i];

        if (mags[i] > max_mag)
        {
            max_mag                             =   mags[i];
            max_idx                             =   i;
        }

        g_bins[i].s1                            =   0.0f;
        g_bins[i].s2                            =   0.0f;
    }

    /* Build magnitude report */
    char report[160];
    int  rpos                                   =   0;

    for (int i = 0; i < GOERTZEL_NUM_BINS; i++)
    {
        rpos                                   +=   snprintf(report + rpos, sizeof(report) - rpos,
                                                             "%uHz=%u ",
                                                             g_bins[i].freq_hz,
                                                             (unsigned int)mags[i]);
    }

    /* Concentration: dominant / total — measures spectral purity */
    int conc_pct                                =   (total_power > 0.0f)
                                                  ? (int)(100.0f * max_mag / total_power)
                                                  : 0;

    LOG("TEST", "DFT_Verify", "INFO",
        "blk=%lu %s | peak=%uHz conc=%d%%",
        (unsigned long)g_dft_block_num, report,
        g_bins[max_idx].freq_hz, conc_pct);

    g_dft_block_num++;
    g_dft_count                                 =   0;
}

static void test_dft_verify_process(void)
{
    /* Feed channel 0 from both scans */
    for (int s = 0; s < ADC_DMA_SAMPLES; s++)
    {
        dft_feed_sample(g_adc_snapshot[s * ADC_DMA_CHANNELS + g_considered_ch[0]]);
    }
}

#endif

/*============= PACKET CODEC VERIFY MODE =======================================================*/

#if defined(BSAU_MODE_TEST_PKT_LOG)

static bool pkt_check(const char *name, bool condition, const char *fmt, ...)
{
    char    detail[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(detail, sizeof(detail), fmt, args);
    va_end(args);

    if (condition)
    {
        test_pass++;
        LOG("TEST", name, "PASS", "%s", detail);
    }
    else
    {
        test_fail++;
        LOG("TEST", name, "FAIL", "%s", detail);
    }

    return condition;
}

/*-------------- TB-104a: channel ramp with v2.1 flags and metadata ----------------------------*/
static void test_pkt_ramp(void)
{
    LOG("TEST", "PKT_Ramp", "RUN",
        "ramp 0x100..0x800, seq=0x55, FIRST_PACKET|CLIPPING|BATT_CRIT");

    WL_Packet   tx, rx;
    uint8_t     raw[WL_PAYLOAD_SIZE];

    memset(&tx, 0, sizeof(tx));
    tx.seq                                      =   0x55;
    tx.flags                                    =   WL_FLAG_FIRST_PACKET | WL_FLAG_CLIPPING;
    tx.flags                                    =   WL_BATT_SET(tx.flags, WL_BATT_CRIT);
    tx.tx_retry                                 =   7;
    tx.pkt_loss                                 =   2;
    tx.timestamp                                =   0xBEEF;
    tx.vbat_raw                                 =   0x0ABC;

    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
        {
            tx.samples[s].ch[c]                 =   (uint16_t)(0x100 * (c + 1) + s * 0x10);
        }
    }

    WL_Pack(&tx, raw);
    WL_Unpack(raw, &rx);

    pkt_check("PKT_Seq",       rx.seq       == tx.seq,       "got=0x%02X exp=0x%02X",
              rx.seq,       tx.seq);
    pkt_check("PKT_Flags",     rx.flags     == tx.flags,     "got=0x%02X exp=0x%02X",
              rx.flags,     tx.flags);
    pkt_check("PKT_TxRetry",   rx.tx_retry  == tx.tx_retry,  "got=%u exp=%u",
              rx.tx_retry,  tx.tx_retry);
    pkt_check("PKT_PktLoss",   rx.pkt_loss  == tx.pkt_loss,  "got=%u exp=%u",
              rx.pkt_loss,  tx.pkt_loss);
    pkt_check("PKT_Timestamp", rx.timestamp == tx.timestamp, "got=0x%04X exp=0x%04X",
              rx.timestamp, tx.timestamp);
    pkt_check("PKT_VbatRaw",   rx.vbat_raw  == tx.vbat_raw,  "got=0x%04X exp=0x%04X",
              rx.vbat_raw,  tx.vbat_raw);

    bool samples_ok                             =   true;
    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
        {
            if (rx.samples[s].ch[c] != tx.samples[s].ch[c])
            {
                pkt_check("PKT_Sample", false, "s%dc%d: got=0x%03X exp=0x%03X",
                          s, c, rx.samples[s].ch[c], tx.samples[s].ch[c]);
                samples_ok                      =   false;
            }
        }
    }

    if (samples_ok)
    {
        pkt_check("PKT_Samples", true, "All %d samples match",
                  WL_SAMPLES_PER_PACKET * WL_NUM_CHANNELS);
    }
}

/*-------------- TB-104b: 12-bit boundary values 0x000 and 0xFFF -------------------------------*/
static void test_pkt_boundary(void)
{
    LOG("TEST", "PKT_Boundary", "RUN", "12-bit boundary: 0x000 and 0xFFF");

    WL_Packet   tx, rx;
    uint8_t     raw[WL_PAYLOAD_SIZE];

    memset(&tx, 0, sizeof(tx));

    /* All zeros */
    WL_Pack(&tx, raw);
    WL_Unpack(raw, &rx);

    bool all_zero                               =   true;
    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
        {
            if (rx.samples[s].ch[c] != 0x000)
            {
                all_zero                        =   false;
            }
        }
    }
    pkt_check("PKT_AllZero", all_zero, "All channels = 0x000");

    /* All 0xFFF (12-bit max) */
    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
        {
            tx.samples[s].ch[c]                 =   0x0FFF;
        }
    }

    WL_Pack(&tx, raw);
    WL_Unpack(raw, &rx);

    bool all_max                                =   true;
    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
        {
            if (rx.samples[s].ch[c] != 0x0FFF)
            {
                all_max                         =   false;
            }
        }
    }
    pkt_check("PKT_AllMax", all_max, "All channels = 0xFFF");
}

/*-------------- TB-104c: sequence number extremes ---------------------------------------------*/
static void test_pkt_seq_wrap(void)
{
    LOG("TEST", "PKT_SeqWrap", "RUN", "seq 0x00 and 0xFF");

    WL_Packet   tx, rx;
    uint8_t     raw[WL_PAYLOAD_SIZE];

    memset(&tx, 0, sizeof(tx));

    tx.seq                                      =   0xFF;
    WL_Pack(&tx, raw);
    WL_Unpack(raw, &rx);
    pkt_check("PKT_SeqFF", rx.seq == 0xFF, "got=0x%02X exp=0xFF", rx.seq);

    tx.seq                                      =   0x00;
    WL_Pack(&tx, raw);
    WL_Unpack(raw, &rx);
    pkt_check("PKT_Seq00", rx.seq == 0x00, "got=0x%02X exp=0x00", rx.seq);
}

/*-------------- TB-104d: alternating bit patterns ---------------------------------------------*/
static void test_pkt_alternating(void)
{
    LOG("TEST", "PKT_Alt", "RUN", "alternating 0xAAA / 0x555");

    WL_Packet   tx, rx;
    uint8_t     raw[WL_PAYLOAD_SIZE];

    memset(&tx, 0, sizeof(tx));
    tx.seq                                      =   0xAB;

    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
        {
            tx.samples[s].ch[c]                 =   (c & 1) ? 0x0555 : 0x0AAA;
        }
    }

    WL_Pack(&tx, raw);
    WL_Unpack(raw, &rx);

    bool match                                  =   true;
    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
        {
            if (rx.samples[s].ch[c] != tx.samples[s].ch[c])
            {
                match                           =   false;
            }
        }
    }

    pkt_check("PKT_AltData", match, "0xAAA/0x555 %s", match ? "OK" : "MISMATCH");
}

/*-------------- TB-104e: 13-bit overflow masking ----------------------------------------------*/
static void test_pkt_overflow(void)
{
    LOG("TEST", "PKT_Overflow", "RUN", "13-bit overflow masking");

    WL_Packet   tx, rx;
    uint8_t     raw[WL_PAYLOAD_SIZE];

    memset(&tx, 0, sizeof(tx));
    tx.seq                                      =   0x01;

    /* Feed values above 12-bit range — WL_Pack masks with 0x0FFF */
    tx.samples[0].ch[0]                         =   0x1ABC;     /* expect 0x0ABC after mask */
    tx.samples[0].ch[1]                         =   0xFFFF;     /* expect 0x0FFF after mask */
    tx.samples[1].ch[2]                         =   0x2000;     /* expect 0x0000 after mask */

    WL_Pack(&tx, raw);
    WL_Unpack(raw, &rx);

    pkt_check("PKT_Ovf_0", rx.samples[0].ch[0] == 0x0ABC,
              "0x1ABC->0x%03X exp=0xABC", rx.samples[0].ch[0]);
    pkt_check("PKT_Ovf_1", rx.samples[0].ch[1] == 0x0FFF,
              "0xFFFF->0x%03X exp=0xFFF", rx.samples[0].ch[1]);
    pkt_check("PKT_Ovf_2", rx.samples[1].ch[2] == 0x0000,
              "0x2000->0x%03X exp=0x000", rx.samples[1].ch[2]);
}

/*-------------- TB-104f: raw byte verification — v2.1 wire layout -----------------------------
 *
 *  Known input:
 *      seq=0x42, flags=0x81 (FIRST_PACKET | BATT_LOW),
 *      tx_retry=0x03, pkt_loss=0x02,
 *      timestamp=0x1234, vbat_raw=0x0ABC,
 *      samples[0].ch[0]=0x0123, samples[0].ch[1]=0x0456
 *
 *  Expected wire bytes:
 *      raw[0]  = 0x42   (seq)
 *      raw[1]  = 0x81   (flags)
 *      raw[2]  = 0x03   (tx_retry)
 *      raw[3]  = 0x02   (pkt_loss)
 *      raw[4]  = 0x34   (timestamp low: 0x1234 & 0xFF)
 *      raw[5]  = 0x12   (timestamp high: 0x1234 >> 8)
 *      raw[6]  = 0xAB   (vbat_raw >> 4)
 *      raw[7]  = 0xC0   ((vbat_raw & 0x0F) << 4)
 *      raw[8]  = 0x23   (ch0 low: 0x123 & 0xFF)
 *      raw[9]  = 0x61   (ch0_hi:4 | ch1_lo:4 = 0x01 | 0x60)
 *      raw[10] = 0x45   (ch1 high: 0x456 >> 4)
 *----------------------------------------------------------------------------------------------*/
static void test_pkt_raw_bytes(void)
{
    LOG("TEST", "PKT_RawBytes", "RUN", "verify v2.1 packed byte layout");

    WL_Packet   tx;
    uint8_t     raw[WL_PAYLOAD_SIZE];

    memset(&tx, 0, sizeof(tx));
    tx.seq                                      =   0x42;
    tx.flags                                    =   0x81;
    tx.tx_retry                                 =   0x03;
    tx.pkt_loss                                 =   0x02;
    tx.timestamp                                =   0x1234;
    tx.vbat_raw                                 =   0x0ABC;
    tx.samples[0].ch[0]                         =   0x0123;
    tx.samples[0].ch[1]                         =   0x0456;

    WL_Pack(&tx, raw);

    /* Metadata bytes [0..7] */
    pkt_check("PKT_Raw_Seq",    raw[0]  == 0x42, "raw[0]=0x%02X exp=0x42",  raw[0]);
    pkt_check("PKT_Raw_Flags",  raw[1]  == 0x81, "raw[1]=0x%02X exp=0x81",  raw[1]);
    pkt_check("PKT_Raw_Retry",  raw[2]  == 0x03, "raw[2]=0x%02X exp=0x03",  raw[2]);
    pkt_check("PKT_Raw_Loss",   raw[3]  == 0x02, "raw[3]=0x%02X exp=0x02",  raw[3]);
    pkt_check("PKT_Raw_TsLo",   raw[4]  == 0x34, "raw[4]=0x%02X exp=0x34",  raw[4]);
    pkt_check("PKT_Raw_TsHi",   raw[5]  == 0x12, "raw[5]=0x%02X exp=0x12",  raw[5]);
    pkt_check("PKT_Raw_VbHi",   raw[6]  == 0xAB, "raw[6]=0x%02X exp=0xAB",  raw[6]);
    pkt_check("PKT_Raw_VbLo",   raw[7]  == 0xC0, "raw[7]=0x%02X exp=0xC0",  raw[7]);

    /* Sample bytes [8..10] — first channel pair of sample 0 */
    pkt_check("PKT_Raw_S0",     raw[8]  == 0x23, "raw[8]=0x%02X exp=0x23",  raw[8]);
    pkt_check("PKT_Raw_S1",     raw[9]  == 0x61, "raw[9]=0x%02X exp=0x61",  raw[9]);
    pkt_check("PKT_Raw_S2",     raw[10] == 0x45, "raw[10]=0x%02X exp=0x45", raw[10]);
}

/*-------------- TB-104g: v2.1 metadata round-trip ---------------------------------------------*/
static void test_pkt_metadata(void)
{
    LOG("TEST", "PKT_Metadata", "RUN", "v2.1 metadata fields round-trip");

    WL_Packet   tx, rx;
    uint8_t     raw[WL_PAYLOAD_SIZE];

    memset(&tx, 0, sizeof(tx));
    tx.seq                                      =   0xFE;
    tx.flags                                    =   0xFF;           /* All flags set */
    tx.tx_retry                                 =   0x05;
    tx.pkt_loss                                 =   0x03;
    tx.timestamp                                =   0xABCD;
    tx.vbat_raw                                 =   0x0F0F;

    WL_Pack(&tx, raw);
    WL_Unpack(raw, &rx);

    pkt_check("META_seq",   rx.seq       == 0xFE,   "seq=0x%02X",   rx.seq);
    pkt_check("META_flags", rx.flags     == 0xFF,   "flags=0x%02X", rx.flags);
    pkt_check("META_retry", rx.tx_retry  == 0x05,   "retry=%u",     rx.tx_retry);
    pkt_check("META_loss",  rx.pkt_loss  == 0x03,   "loss=%u",      rx.pkt_loss);
    pkt_check("META_ts",    rx.timestamp == 0xABCD, "ts=0x%04X",    rx.timestamp);
    pkt_check("META_vbat",  rx.vbat_raw  == 0x0F0F, "vbat=0x%04X",  rx.vbat_raw);
}

/*-------------- TB-104h: flag combinations — every flag bit + all BATT_LVL values -------------*/
static void test_pkt_flags(void)
{
    LOG("TEST", "PKT_Flags", "RUN", "all flag bits + BATT_LVL combinations");

    WL_Packet   tx, rx;
    uint8_t     raw[WL_PAYLOAD_SIZE];

    /* Each individual flag bit */
    const uint8_t flag_bits[]                   =   {
        WL_FLAG_FIRST_PACKET,
        WL_FLAG_CLIPPING,
        WL_FLAG_ELEC_OFF,
        WL_FLAG_ADC_OVRN,
        WL_FLAG_TX_SAT,
        WL_FLAG_CAL
    };

    for (int i = 0; i < (int)(sizeof(flag_bits) / sizeof(flag_bits[0])); i++)
    {
        memset(&tx, 0, sizeof(tx));
        tx.flags                                =   flag_bits[i];
        WL_Pack(&tx, raw);
        WL_Unpack(raw, &rx);
        pkt_check("FLAG_bit", rx.flags == tx.flags,
                  "bit=0x%02X got=0x%02X", tx.flags, rx.flags);
    }

    /* All BATT_LVL values */
    const uint8_t batt_lvls[]                   =   {
        WL_BATT_OK,
        WL_BATT_LOW,
        WL_BATT_CRIT,
        WL_BATT_CHARG
    };

    for (int i = 0; i < (int)(sizeof(batt_lvls) / sizeof(batt_lvls[0])); i++)
    {
        memset(&tx, 0, sizeof(tx));
        tx.flags                                =   WL_BATT_SET(WL_FLAG_FIRST_PACKET, batt_lvls[i]);
        WL_Pack(&tx, raw);
        WL_Unpack(raw, &rx);
        pkt_check("FLAG_batt", rx.flags == tx.flags,
                  "lvl=%u got=0x%02X exp=0x%02X", batt_lvls[i], rx.flags, tx.flags);
    }
}

/*-------------- TB-104i: vbat_raw exhaustive — all 4096 12-bit values -------------------------*/
static void test_pkt_vbat_exhaustive(void)
{
    LOG("TEST", "PKT_VbatExh", "RUN", "vbat_raw encode/decode 0..4095");

    int errors                                  =   0;

    for (uint32_t v = 0; v < 4096; v++)
    {
        uint8_t buf[2];
        WL_VBAT_ENCODE(buf, v);
        uint16_t decoded                        =   WL_VBAT_DECODE(buf);

        if (decoded != (uint16_t)v)
        {
            if (errors < 3)
            {
                LOG("TEST", "PKT_VbatExh", "FAIL",
                    "v=0x%03X enc=[0x%02X,0x%02X] dec=0x%03X",
                    (unsigned)v, buf[0], buf[1], decoded);
            }
            errors++;
        }
    }

    pkt_check("PKT_VbatExh", errors == 0,
              "4096 values tested, %d errors", errors);
}

/*-------------- Master runner -----------------------------------------------------------------*/
static void test_packet_verify_run(void)
{
    LOG("TEST", "PacketVerify", "RUN", "=== WL CODEC v2.1 ROUND-TRIP START ===");

    test_pkt_ramp();
    test_pkt_boundary();
    test_pkt_seq_wrap();
    test_pkt_alternating();
    test_pkt_overflow();
    test_pkt_raw_bytes();
    test_pkt_metadata();
    test_pkt_flags();
    test_pkt_vbat_exhaustive();

    bool all_pass                               =   (test_fail == 0);

    LOG("TEST", "PacketVerify", all_pass ? "PASS" : "FAIL",
        "=== WL CODEC: %lu PASS, %lu FAIL ===",
        (unsigned long)test_pass, (unsigned long)test_fail);
}

#endif  /* BSAU_MODE_TEST_PKT_LOG */

/*============= NRF VERIFY MODE ================================================================
 *
 *  Runtime test strategy — three interleaved phases:
 *
 *      TX ping         every iteration. Sends a real payload, tracks
 *                      ok/fail/PLOS/ARC.
 *      Health check    every NRF_HEALTH_CHECK_INTERVAL iterations. Re-runs
 *                      SPI loopback + register audit + FIFO exercise to
 *                      catch intermittent faults.
 *      Stress cycle    every NRF_STRESS_INTERVAL iterations. Power-cycles
 *                      the radio and re-verifies init state. Catches
 *                      thermal/EMI drift.
 *==============================================================================================*/

#if defined(BSAU_MODE_TEST_NRF_LOG)

#define NRF_TEST_TX_INTERVAL_MS         150U
#define NRF_HEALTH_CHECK_INTERVAL       20U         /* every 20 pings  -> ~3 s  */
#define NRF_STRESS_INTERVAL             100U        /* every 100 pings -> ~15 s */
#define NRF_SUMMARY_INTERVAL            50U         /* summary log every 50 pings -> ~7.5 s */

static uint32_t         nrf_iter                =   0;
static uint32_t         nrf_health_pass         =   0;
static uint32_t         nrf_health_fail         =   0;

/*-------------- Phase A: TX ping --------------------------------------------------------------*/
static void nrf_phase_tx_ping(void)
{
    /* Build a recognizable payload: [0xBE][iter_lo][iter_hi][zeros...] */
    uint8_t payload[WL_PAYLOAD_SIZE];
    memset(payload, 0, sizeof(payload));
    payload[0]                                  =   0xBE;
    payload[1]                                  =   (uint8_t)( nrf_iter       & 0xFF);
    payload[2]                                  =   (uint8_t)((nrf_iter >> 8) & 0xFF);

    NRF_Status status                           =   NRF_Transmit(&g_hnrf_test, payload);

    if (status == NRF_OK)
    {
        nrf_tx_success++;
    }
    else
    {
        nrf_tx_fail++;
        NRF_FlushTX(&g_hnrf_test);
    }

    /* Link quality stats */
    uint8_t lost, retx;
    NRF_GetTxStats(&g_hnrf_test, &lost, &retx);

    LOG("TEST", "NRF_TxPing", status == NRF_OK ? "PASS" : "FAIL",
        "iter=%lu PLOS=%u ARC=%u",
        (unsigned long)nrf_iter, lost, retx);
}

/*-------------- Phase B: Health check (non-destructive, no RF) --------------------------------*/
static void nrf_phase_health_check(void)
{
    LOG("TEST", "NRF_Health", "RUN",
        "Periodic SPI/REGs/FIFO check at iter=%lu", (unsigned long)nrf_iter);

    TestResult  r;
    bool        ok                              =   true;

    r                                           =   NRF_Test_SPI(&g_hnrf_test);
    if (r != TEST_PASS) ok                      =   false;

    r                                           =   NRF_Test_Registers(&g_hnrf_test);
    if (r != TEST_PASS) ok                      =   false;

    r                                           =   NRF_Test_FIFO(&g_hnrf_test);
    if (r != TEST_PASS) ok                      =   false;

    if (ok)
    {
        nrf_health_pass++;
        LOG("TEST", "NRF_Health", "PASS",
            "SPI+REGs+FIFO OK (pass=%lu)", (unsigned long)nrf_health_pass);
    }
    else
    {
        nrf_health_fail++;
        LOG("TEST", "NRF_Health", "FAIL",
            "Degradation detected (fail=%lu)", (unsigned long)nrf_health_fail);
    }
}

/*-------------- Phase C: Stress cycle (power down/up + re-verify) -----------------------------*/
static void nrf_phase_stress_cycle(void)
{
    LOG("TEST", "NRF_Stress", "RUN",
        "Power cycle + Address + TX at iter=%lu", (unsigned long)nrf_iter);

    static const uint8_t nrf_addr[NRF_ADDR_WIDTH]   =   NRF_ADDRESS;
    TestResult  r;
    bool        ok                              =   true;

    r                                           =   NRF_Test_PowerCycle(&g_hnrf_test);
    if (r != TEST_PASS) ok                      =   false;

    r                                           =   NRF_Test_Address(&g_hnrf_test, nrf_addr);
    if (r != TEST_PASS) ok                      =   false;

    r                                           =   NRF_Test_TX(&g_hnrf_test);
    if (r != TEST_PASS) ok                      =   false;

    LOG("TEST", "NRF_Stress", ok ? "PASS" : "FAIL",
        "Power cycle + Addr + TX %s", ok ? "OK" : "FAILED");
}

/*-------------- Summary log -------------------------------------------------------------------*/
static void nrf_log_summary(void)
{
    uint32_t total                              =   nrf_tx_success + nrf_tx_fail;
    uint32_t elapsed_s                          =   (HAL_GetTick() - nrf_tx_start_tick) / 1000;

    LOG("TEST", "NRF_Summary", "INFO",
        "tx_ok=%lu tx_fail=%lu rate=%lu%% health=%lu/%lu t=%lus",
        (unsigned long)nrf_tx_success,
        (unsigned long)nrf_tx_fail,
        total > 0 ? (unsigned long)(nrf_tx_success * 100 / total) : 0UL,
        (unsigned long)nrf_health_pass,
        (unsigned long)(nrf_health_pass + nrf_health_fail),
        (unsigned long)elapsed_s);
}

/*-------------- Master runtime iteration ------------------------------------------------------*/
static void test_nrf_verify_run(void)
{
    nrf_iter++;

    /* Phase C: stress cycle (least frequent, most invasive) */
    if (nrf_iter % NRF_STRESS_INTERVAL == 0)
    {
        nrf_phase_stress_cycle();
    }
    /* Phase B: health check (moderate frequency, non-destructive) */
    else if (nrf_iter % NRF_HEALTH_CHECK_INTERVAL == 0)
    {
        nrf_phase_health_check();
    }
    /* Phase A: TX ping (every iteration except health/stress) */
    else
    {
        nrf_phase_tx_ping();
    }

    /* Periodic aggregate summary */
    if (nrf_iter % NRF_SUMMARY_INTERVAL == 0)
    {
        nrf_log_summary();
    }

    HAL_Delay(NRF_TEST_TX_INTERVAL_MS);
}

#endif  /* BSAU_MODE_TEST_NRF_LOG */

/*============= BSAU_Test_Run ==================================================================*/

void BSAU_Test_Run(void)
{
#if defined(BSAU_MODE_TEST_PKT_LOG)
    /* Run codec tests once, then idle with LED blink */
    test_packet_verify_run();
    LOG("TEST", "BSAU_Test_Run", "INFO", "Tests complete. Idling.");

    while (1)
    {
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        HAL_Delay(500);
    }
#endif

#if defined(BSAU_MODE_TEST_NRF_LOG)
    while (1)
    {
        test_nrf_verify_run();
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    }
#endif

#if defined(BSAU_MODE_TEST_ADC_CSV) || defined(BSAU_MODE_TEST_CSV) || defined(BSAU_MODE_TEST_DFT_LOG)
    while (1)
    {
        __WFI();

        if (!g_pkt_ready)
        {
            continue;
        }

        g_pkt_ready                             =   0;

        check_deferred_adc_error();

        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);

    #if defined(BSAU_MODE_TEST_ADC_CSV)
        test_adc_stream_send();
    #endif

    #if defined(BSAU_MODE_TEST_CSV)
        test_adc_collect_send();
    #endif

    #if defined(BSAU_MODE_TEST_DFT_LOG)
        test_dft_verify_process();
    #endif
    }
#endif
}

/*==============================================================================================*/

