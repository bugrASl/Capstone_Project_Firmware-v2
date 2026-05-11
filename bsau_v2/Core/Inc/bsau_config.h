/**
 *  @file   bsau_config.h
 *  @brief  BSAU compile-time configuration — profile selection, channel count, timing.
 */

#ifndef BSAU_CONFIG_H
#define BSAU_CONFIG_H

/*============= BOARD IDENTITY (do not change) =================================================*/

#define LOG_BOARD_BSAU

/*============= SELECT EXACTLY ONE MODE ========================================================*/

/* #define BSAU_MODE_RELEASE      */
/* #define BSAU_MODE_DEBUG */
/* #define BSAU_MODE_TEST_ADC_CSV */
/* #define BSAU_MODE_TEST_PKT_LOG */
/* #define BSAU_MODE_TEST_CSV */
/* #define BSAU_MODE_TEST_DFT_LOG */
/* #define BSAU_MODE_TEST_NRF_LOG */
#define BSAU_MODE_DATASET

/*============= DATASET MODE PARAMETERS ========================================================*/
/**
 *  BSAU_DATASET_CSV_DECIMATION controls how many packets pass between UART
 *  CSV lines. The packet loop runs at 1000 pkt/s (2 scans per packet @ 2 kHz).
 *  At decimation = 1 we emit ONE CSV line per packet using scan 0 only, so
 *  the CSV rate equals the packet rate (1000 samples/s per channel). This
 *  keeps UART well below saturation at 921600 baud and leaves headroom for
 *  the NRF TX burst on every packet.
 *
 *  UART budget check (worst-case CSV line length):
 *      "4095,4095,4095,4095,4095,4095,4095,4095\r\n"   = 42 chars
 *      At 921600 baud, 10 bits/char                    = 86.8 µs/char
 *      Line wire-time                                  = 3.65 ms/line
 *      Transmit is DMA-backed (see log.h §5)           → ~6 µs CPU cost
 *      42 B × 1000 Hz = 42 kB/s = 420 kbps             ≈ 45 % of 921600 baud
 *
 *  If the CSV pace ever starves the NRF path (retries rising), raise this:
 *      DECIMATION = 2  → 500 Hz CSV (every other packet)
 *      DECIMATION = 5  → 200 Hz CSV (matches typical predict.py windowing)
 */
#define BSAU_DATASET_CSV_DECIMATION     1U

/*==============================================================================================*/

#endif /* BSAU_CONFIG_H */

