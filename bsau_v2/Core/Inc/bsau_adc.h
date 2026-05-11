/**
 *  @file       bsau_adc.h
 *  @brief      BSAU ADC/DMA acquisition module — public interface
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.1
 *  @details
 *              Public globals published by this module are filled by the
 *              DMA half-complete and complete ISRs (see bsau_adc.c). The main
 *              loop consumes g_adc_snapshot[] once per g_pkt_ready == 1 pulse.
 *
 *              v2.1 changes:
 *                  - Half-complete ISR now also checks g_pkt_ready and
 *                    increments g_adc_dropped if the main loop has not yet
 *                    consumed the prior pair (closes the scan-0 tearing race).
 */

#ifndef BSAU_ADC_H
#define BSAU_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsau_app.h"
#include "main.h"

/*============= PUBLIC GLOBALS (ISR -> main loop) ==============================================*/

extern volatile uint16_t    g_adc_snapshot[ADC_DMA_BUF_SIZE];
extern volatile uint8_t     g_pkt_ready;
extern volatile uint32_t    g_adc_dropped;
extern volatile uint32_t    g_adc_error_code;       /* Deferred ISR error — log from main loop */

/*============= API ============================================================================*/

void        BSAU_ADC_Init        (void);
uint16_t    BSAU_ADC_GetBattery  (void);

/*==============================================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* BSAU_ADC_H */
