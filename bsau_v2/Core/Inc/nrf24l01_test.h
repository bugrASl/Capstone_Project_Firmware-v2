/**
 *  @file   nrf24l01_test.h
 *  @brief  NRF24L01+ test API — self-test entry point.
 */

#ifndef NRF24L01_TEST_H
#define NRF24L01_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf24l01.h"

/*============= TestResult (guarded for CPCU coexistence) ======================================
 *  On CPCU, cpcu_test.h defines TestResult (with TEST_SKIP = 2) and its
 *  include guard is CPCU_TEST_H. If cpcu_test.h is included before this
 *  header, skip our definition to avoid a redefinition error.
 *  On BSAU, CPCU_TEST_H is never defined so this always compiles.
 *==============================================================================================*/

#ifndef CPCU_TEST_H
typedef enum
{
    TEST_PASS   =   0,
    TEST_FAIL   =   1
} TestResult;
#endif

/*============= RX TEST TIMEOUT ================================================================
 *  NRF_TEST_RX_TIMEOUT_MS controls Phase B of the RX test:
 *      > 0  poll STATUS for RX_DR up to this many ms, then FAIL if no packet
 *      = 0  skip Phase B entirely (only run config checks)
 *  Default 5000 ms (5 s) gives BSAU time to boot and start transmitting.
 *  Override in project preprocessor if needed.
 *==============================================================================================*/

#ifndef NRF_TEST_RX_TIMEOUT_MS
    #define NRF_TEST_RX_TIMEOUT_MS  5000
#endif

/*============= API ============================================================================*/

/**
 *  @brief      Run all NRF self-tests applicable to the current role
 *  @param      hnrf    Initialized NRF_Handle (NRF_Init must have returned NRF_OK)
 *  @param      addr    The 5-byte address passed to NRF_Init (for verification)
 *  @retval     TEST_PASS   if all tests pass
 *  @retval     TEST_FAIL   if any test fails
 */
TestResult  NRF_Test_All        (NRF_Handle *hnrf, const uint8_t addr[NRF_ADDR_WIDTH]);

/*-------------- Individual tests --------------------------------------------------------------*/

TestResult  NRF_Test_SPI        (NRF_Handle *hnrf);
TestResult  NRF_Test_Registers  (NRF_Handle *hnrf);
TestResult  NRF_Test_Address    (NRF_Handle *hnrf, const uint8_t addr[NRF_ADDR_WIDTH]);
TestResult  NRF_Test_FIFO       (NRF_Handle *hnrf);
TestResult  NRF_Test_PowerCycle (NRF_Handle *hnrf);

#if defined(NRF_ROLE_TX)
TestResult  NRF_Test_TX         (NRF_Handle *hnrf);
#endif

#if defined(NRF_ROLE_RX)
TestResult  NRF_Test_RX         (NRF_Handle *hnrf);
#endif

/*==============================================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* NRF24L01_TEST_H */

