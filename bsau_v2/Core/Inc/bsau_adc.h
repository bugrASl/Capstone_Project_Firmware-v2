/**
 *  @file   bsau_adc.h
 *  @brief  BSAU ADC API — channel read, battery measurement, DMA buffer access.
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

