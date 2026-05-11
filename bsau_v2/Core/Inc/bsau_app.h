/**
 *  @file   bsau_app.h
 *  @brief  BSAU application API — init, main loop entry, profile-specific callbacks.
 */

#ifndef BSAU_APP_H
#define BSAU_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/*============= ROLE DEFINITIONS ===============================================================*/
/*
 *  BSAU is exclusively the transmitter side of the wireless link. These
 *  defines propagate to nrf24l01.h and wireless_packet.h so their
 *  role-conditional sections compile the right halves.
 */

#ifndef NRF_ROLE_TX
    #define NRF_ROLE_TX
#endif
#ifndef WL_ROLE_TX
    #define WL_ROLE_TX
#endif

/*============= ADC / DMA PARAMETERS ===========================================================*/
/*
 *  The ADC scan sequence contains 9 channels per trigger event:
 *      Ranks 1-8   → 8 EMG channels (PA0-PA7, IN5-IN12)
 *      Rank  9     → 1 battery channel (PB0, IN15)
 *  ADC_DMA_SAMPLES scans are assembled into one wireless packet.
 */

#define ADC_DMA_CHANNELS            9
#define ADC_DMA_SAMPLES             2
#define ADC_DMA_BUF_SIZE            (ADC_DMA_CHANNELS * ADC_DMA_SAMPLES)    /* = 18 */
#define ADC_BATT_INDEX              8

/*============= RADIO LINK PARAMETERS ==========================================================*/

#define NRF_CHANNEL                 108
#define NRF_ADDRESS                 { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 }

/*============= BATTERY THRESHOLDS =============================================================*/
/*
 *  100k/100k divider on PB0. VDDA = 3.3 V, 12-bit ADC.
 *      V_batt = ADC * (3.3 / 4095) * 2
 *      LOW   threshold 1861 → V ≈ 3.0 V
 *      CRIT  threshold 1675 → V ≈ 2.7 V
 */

#define BATT_LOW_THRESHOLD          1861
#define BATT_CRITICAL_THRESHOLD     1675

/*============= GOERTZEL DFT CONFIG (test mode) ================================================*/

#define GOERTZEL_NUM_BINS           5
#define GOERTZEL_BINS_HZ            { 50, 100, 200, 350, 500 }
#define GOERTZEL_BLOCK_SIZE         512
#define GOERTZEL_FS_HZ              2000.0f

/*============= CONSIDERED CHANNELS (dataset collect, DFT test) ================================*/

#define CONSIDERED_CHANNELS_COUNT   8
#define CONSIDERED_CHANNELS         { 0, 1, 2, 3, 4, 5, 6, 7 }

/*============= API ============================================================================*/

void    BSAU_Init  (void);
void    BSAU_Run   (void);

/*==============================================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* BSAU_APP_H */

