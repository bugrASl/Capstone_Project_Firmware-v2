/**
 *  @file       bsau_test.h
 *  @brief      BSAU hardware test module — public interface
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.1
 *  @details
 *                          Test modes (one enabled per build)
 *              ────────────────────────────────────────────────────────────────
 *              BSAU_MODE_TEST_ADC_CSV   Binary ADC stream (full snapshot, sync counter)
 *              BSAU_MODE_TEST_PKT_LOG   WL codec round-trip + raw byte + overflow tests
 *              BSAU_MODE_TEST_CSV       ASCII CSV capture with dropped-packet column
 *              BSAU_MODE_TEST_DFT_LOG   Goertzel DFT with dominant-frequency detection
 *              BSAU_MODE_TEST_NRF_LOG   Full NRF self-test suite + TX loop with stats
 *
 *              BSAU_Test_Init() and BSAU_Test_Run() are called only when one
 *              of the above is defined; bsau_app.c guards the delegation via
 *              #if blocks so the test harness does not bloat RELEASE builds.
 */

#ifndef BSAU_TEST_H
#define BSAU_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsau_app.h"
#include "main.h"

/*============= API ============================================================================*/

void    BSAU_Test_Init  (void);
void    BSAU_Test_Run   (void);

/*==============================================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* BSAU_TEST_H */
