/**
 *  @file   log.h
 *  @brief  BSAU logging macros — UART printf with profile-conditional enable.
 */

#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================================
 * §1  BOARD IDENTITY VALIDATION
 *==============================================================================================*/

#if defined(LOG_BOARD_CPCU) && defined(LOG_BOARD_BSAU)
    #error "Define exactly one of LOG_BOARD_CPCU or LOG_BOARD_BSAU."
#endif

#if !defined(LOG_BOARD_CPCU) && !defined(LOG_BOARD_BSAU)
    #error "Define LOG_BOARD_CPCU or LOG_BOARD_BSAU in preprocessor settings."
#endif

/*==============================================================================================
 * §2  DERIVE LOG_ENABLED / LOG_CSV_ENABLED
 *==============================================================================================*/

/*-------------- CPCU -------------------------------------------------------------------------*/
#ifdef LOG_BOARD_CPCU

    #if (defined(CPCU_MODE_RELEASE)  + defined(CPCU_MODE_DEBUG) + \
         defined(CPCU_MODE_TEST_LOG) + defined(CPCU_MODE_TEST_CSV)) > 1
        #error "Define exactly one CPCU_MODE_*."
    #endif

    #if !defined(CPCU_MODE_RELEASE)  && !defined(CPCU_MODE_DEBUG) && \
        !defined(CPCU_MODE_TEST_LOG) && !defined(CPCU_MODE_TEST_CSV)
        #define CPCU_MODE_DEBUG
    #endif

    #if   defined(CPCU_MODE_DEBUG)
        #define LOG_ENABLED             1
        #define LOG_CSV_ENABLED         1
    #elif defined(CPCU_MODE_TEST_LOG)
        #define LOG_ENABLED             1
        #define LOG_CSV_ENABLED         0
    #elif defined(CPCU_MODE_TEST_CSV)
        #define LOG_ENABLED             0
        #define LOG_CSV_ENABLED         1
    #else   /* RELEASE */
        #define LOG_ENABLED             0
        #define LOG_CSV_ENABLED         0
    #endif

    #define LOG_TAG                     "CPCU"

#endif /* LOG_BOARD_CPCU */

/*-------------- BSAU -------------------------------------------------------------------------*/
#ifdef LOG_BOARD_BSAU

    /*
     *  Mode-count validation. BSAU_MODE_DATASET is a dual-path mode
     *  (production TX loop + UART CSV) and is mutually exclusive with
     *  the others, just like the rest.
     */
    #if (defined(BSAU_MODE_RELEASE)      + defined(BSAU_MODE_DEBUG)        + \
         defined(BSAU_MODE_TEST_ADC_CSV) + defined(BSAU_MODE_TEST_PKT_LOG) + \
         defined(BSAU_MODE_TEST_CSV)     + defined(BSAU_MODE_TEST_DFT_LOG) + \
         defined(BSAU_MODE_TEST_NRF_LOG) + defined(BSAU_MODE_DATASET)) > 1
        #error "Define exactly one BSAU_MODE_*."
    #endif

    /* No default for BSAU — force explicit choice. */
    #if !defined(BSAU_MODE_RELEASE)      && !defined(BSAU_MODE_DEBUG)        && \
        !defined(BSAU_MODE_TEST_ADC_CSV) && !defined(BSAU_MODE_TEST_PKT_LOG) && \
        !defined(BSAU_MODE_TEST_CSV)     && !defined(BSAU_MODE_TEST_DFT_LOG) && \
        !defined(BSAU_MODE_TEST_NRF_LOG) && !defined(BSAU_MODE_DATASET)
        #error "Define exactly one BSAU_MODE_* in preprocessor settings."
    #endif

    /*
     *  Derive flags. DATASET keeps LOG off (no human-readable noise on the
     *  collection stream) and CSV on (the actual data path).
     */
    #if   defined(BSAU_MODE_DEBUG)
        #define LOG_ENABLED             1
        #define LOG_CSV_ENABLED         1
    #elif defined(BSAU_MODE_TEST_PKT_LOG) || \
          defined(BSAU_MODE_TEST_DFT_LOG) || \
          defined(BSAU_MODE_TEST_NRF_LOG)
        #define LOG_ENABLED             1
        #define LOG_CSV_ENABLED         0
    #elif defined(BSAU_MODE_TEST_ADC_CSV) || \
          defined(BSAU_MODE_TEST_CSV)     || \
          defined(BSAU_MODE_DATASET)
        #define LOG_ENABLED             0
        #define LOG_CSV_ENABLED         1
    #else   /* RELEASE */
        #define LOG_ENABLED             0
        #define LOG_CSV_ENABLED         0
    #endif

    #define LOG_TAG                     "BSAU"

#endif /* LOG_BOARD_BSAU */

/*==============================================================================================
 * §3  TRANSPORT PRIMITIVES
 *==============================================================================================*/

#if LOG_ENABLED || LOG_CSV_ENABLED

#include <stdio.h>

#ifdef LOG_BOARD_CPCU
    #include "cpcu_ipc.h"
    #define LOG_LOCK()      do { while (HAL_HSEM_FastTake(IPC_HSEM_UART) != HAL_OK) {} } while (0)
    #define LOG_UNLOCK()    HAL_HSEM_Release(IPC_HSEM_UART, 0)
#endif

#ifdef LOG_BOARD_BSAU
    #include <string.h>
    #include "usart.h"
    #define LOG_LOCK()      ((void)0)
    #define LOG_UNLOCK()    ((void)0)
#endif

#endif  /* LOG_ENABLED || LOG_CSV_ENABLED */

/*==============================================================================================
 * §4  LOG — structured [BOARD - MOD]: func [STAT] message
 *
 *  BSAU UART timeout: 20 ms
 *      At 921600 baud → 92160 bytes/sec → 10.85 µs/byte.
 *      LOG_BUF_SIZE = 160 chars → worst case 1.74 ms on the wire.
 *      20 ms gives > 10× headroom (UART is polling, not DMA, for this path).
 *==============================================================================================*/

#if LOG_ENABLED

    /*-------------- CPCU ---------------------------------------------------------------------*/
    #ifdef LOG_BOARD_CPCU

    #define LOG(mod, fn, stat, fmt, ...)                                        \
        do                                                                      \
        {                                                                       \
            LOG_LOCK();                                                         \
            printf("[" LOG_TAG " - %-4s]: %-18s [%-4s]", (mod), (fn), (stat));  \
            if ((fmt)[0] != '\0') printf(" " fmt, ##__VA_ARGS__);               \
            printf("\r\n");                                                     \
            LOG_UNLOCK();                                                       \
        } while (0)

    #endif  /* LOG_BOARD_CPCU */

    /*-------------- BSAU ---------------------------------------------------------------------*/
    #ifdef LOG_BOARD_BSAU

    #define LOG_BUF_SIZE                160

    #define LOG(mod, fn, stat, fmt, ...)                                        \
        do                                                                      \
        {                                                                       \
            char _lb[LOG_BUF_SIZE];                                             \
            int  _ln;                                                           \
            if ((fmt)[0] != '\0')                                               \
                _ln = snprintf(_lb, sizeof(_lb),                                \
                    "[" LOG_TAG " - %-4s]: %-18s [%-4s] " fmt "\r\n",           \
                    (mod), (fn), (stat), ##__VA_ARGS__);                        \
            else                                                                \
                _ln = snprintf(_lb, sizeof(_lb),                                \
                    "[" LOG_TAG " - %-4s]: %-18s [%-4s]\r\n",                   \
                    (mod), (fn), (stat));                                       \
            if (_ln > 0)                                                        \
                HAL_UART_Transmit(&huart1, (uint8_t *)_lb, (uint16_t)_ln, 20);  \
        } while (0)

    #endif  /* LOG_BOARD_BSAU */

#else   /* LOG_ENABLED == 0 */

    #define LOG(mod, fn, stat, fmt, ...)    ((void)0)

#endif  /* LOG_ENABLED */

/*==============================================================================================
 * §5  LOG_CSV — raw prefixless CSV data
 *
 *  BSAU UART timeout: 15 ms
 *      LOG_CSV_BUF_SIZE = 128 chars → worst case 1.39 ms on the wire.
 *      15 ms gives > 10× headroom.
 *
 *  DATASET mode note:
 *      LOG_CSV is called once per packet inside the 1-ms BSAU_Run loop.
 *      With a 42-byte line and 921600 baud, one line is ~3.65 ms of UART
 *      time — longer than the 1-ms packet period. The HAL_UART_Transmit
 *      timeout below (15 ms) is the MAX we'll wait, not the real cost:
 *      the call returns as soon as the TX FIFO drains (CPU cost ~6 µs).
 *      Because USART1_TX is wired to DMA1 Channel 4 (see CubeMX), the
 *      transmit overlaps with NRF SPI and the next ADC scan.
 *==============================================================================================*/

#if LOG_CSV_ENABLED

    /*-------------- CPCU ---------------------------------------------------------------------*/
    #ifdef LOG_BOARD_CPCU

    #define LOG_CSV(fmt, ...)                                                   \
        do                                                                      \
        {                                                                       \
            LOG_LOCK();                                                         \
            printf(fmt "\r\n", ##__VA_ARGS__);                                  \
            LOG_UNLOCK();                                                       \
        } while (0)

    #endif  /* LOG_BOARD_CPCU */

    /*-------------- BSAU ---------------------------------------------------------------------*/
    #ifdef LOG_BOARD_BSAU

    #define LOG_CSV_BUF_SIZE            128

    #define LOG_CSV(fmt, ...)                                                   \
        do                                                                      \
        {                                                                       \
            char _cb[LOG_CSV_BUF_SIZE];                                         \
            int  _cn = snprintf(_cb, sizeof(_cb), fmt "\r\n", ##__VA_ARGS__);   \
            if (_cn > 0)                                                        \
                HAL_UART_Transmit(&huart1, (uint8_t *)_cb, (uint16_t)_cn, 15);  \
        } while (0)

    #endif  /* LOG_BOARD_BSAU */

#else   /* LOG_CSV_ENABLED == 0 */

    #define LOG_CSV(fmt, ...)           ((void)0)

#endif  /* LOG_CSV_ENABLED */

/*==============================================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* LOG_H */

