/**
 *  @file   bsau_test.h
 *  @brief  BSAU test API — test runner entry point and result reporting.
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

