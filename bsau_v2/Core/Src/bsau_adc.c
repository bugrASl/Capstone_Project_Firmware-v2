/**
 *  @file       bsau_adc.c
 *  @brief      BSAU ADC/DMA acquisition module — implementation
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.1
 *  @details
 *              Double-buffered ISR strategy:
 *                  Half-complete     scan 0 stable (DMA writing scan 1) -> copy scan 0
 *                  Transfer-complete scan 1 stable (DMA wraps to scan 0) -> copy scan 1,
 *                                    then set g_pkt_ready.
 *              Both halves of the snapshot come from the same adjacent pair,
 *              so the main loop never reads a mixed old/new sample set.
 *
 *              v2.1 changes:
 *                  - HAL_ADC_ConvHalfCpltCallback now also checks g_pkt_ready
 *                    and increments g_adc_dropped if the main loop has not
 *                    yet consumed the prior pair. Closes the tearing race
 *                    where a late main-loop read of scan 0 could overlap
 *                    the next pair's half-complete write of scan 0.
 *                  - 9-channel scan (8 EMG + battery), 2 samples/packet.
 *                  - Style polish: 98-char banners, column-aligned assignments.
 */

#include "bsau_adc.h"

#include "log.h"
#include "adc.h"
#include "tim.h"

/*============= MODULE STATE ===================================================================*/
/*
 *  g_adc_dma_buf      DMA circular target, filled by ADC hardware.
 *  g_adc_snapshot     Coherent copy for the main loop (published atomically
 *                     via g_pkt_ready == 1 at end of second scan).
 *  g_pkt_ready        Set by the transfer-complete ISR, cleared by the
 *                     main loop after consuming the snapshot.
 *  g_adc_dropped      Count of half/full ISRs that fired while the main
 *                     loop had not yet cleared g_pkt_ready.
 *  g_adc_error_code   Deferred ISR error for main-loop logging.
 */

volatile uint16_t   g_adc_dma_buf   [ADC_DMA_BUF_SIZE];
volatile uint16_t   g_adc_snapshot  [ADC_DMA_BUF_SIZE];
volatile uint8_t    g_pkt_ready         =   0;
volatile uint32_t   g_adc_dropped       =   0;
volatile uint32_t   g_adc_error_code    =   0;

/*============= BSAU_ADC_Init ==================================================================*/

void BSAU_ADC_Init(void)
{
    LOG("ADC", "BSAU_ADC_Init", "RUN", "Calibrating ADC1...");

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        LOG("ADC", "BSAU_ADC_Init", "FAIL", "Calibration failed");
        Error_Handler();
    }
    LOG("ADC", "BSAU_ADC_Init", "OK", "Calibration complete");

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma_buf, ADC_DMA_BUF_SIZE) != HAL_OK)
    {
        LOG("ADC", "BSAU_ADC_Init", "FAIL", "DMA start failed");
        Error_Handler();
    }

    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
    {
        LOG("ADC", "BSAU_ADC_Init", "FAIL", "TIM6 start failed");
        Error_Handler();
    }

    LOG("ADC", "BSAU_ADC_Init", "OK",
        "Pipeline running (TIM6 trig, DMA circ, %dch, %dsamp)",
        ADC_DMA_CHANNELS, ADC_DMA_SAMPLES);
}

/*============= BSAU_ADC_GetBattery ============================================================*/

uint16_t BSAU_ADC_GetBattery(void)
{
    uint32_t sum                        =   0;

    for (int s = 0; s < ADC_DMA_SAMPLES; s++)
    {
        sum                            +=   g_adc_snapshot[s * ADC_DMA_CHANNELS + ADC_BATT_INDEX];
    }

    return (uint16_t)(sum / ADC_DMA_SAMPLES);
}

/*============= HAL CALLBACKS (ISR CONTEXT) ====================================================*/
/*
 *  These run in DMA ISR context (NVIC priority 1). Do NOT call LOG() here —
 *  HAL_UART_Transmit internally uses HAL_GetTick(), which depends on SysTick;
 *  if the ISR priority is equal or lower than SysTick, HAL_GetTick() won't
 *  advance -> deadlock. Errors are stashed in g_adc_error_code and logged
 *  from the main loop.
 *
 *  Double-buffer strategy:
 *      Half-complete       scan 0 stable (DMA writing scan 1) -> copy scan 0
 *      Transfer-complete   scan 1 stable (DMA wraps to scan 0) -> copy scan 1,
 *                          publish via g_pkt_ready = 1.
 *
 *  v2.1 race fix:
 *      Both ISRs now gate their writes on !g_pkt_ready. If the main loop
 *      has not yet consumed the prior pair, the new pair is dropped as a
 *      single unit (symmetric handling) and g_adc_dropped is bumped. This
 *      prevents the previous failure mode where a late main-loop read of
 *      g_adc_snapshot[0..8] could race the next pair's half-complete write
 *      of the same slots, tearing scan 0.
 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        /*
         *  Race guard: if the main loop has not yet consumed the previous
         *  pair, drop this pair at the half-complete boundary. Symmetric
         *  with HAL_ADC_ConvCpltCallback below.
         */
        if (g_pkt_ready)
        {
            g_adc_dropped++;
            return;
        }

        /* Scan 0 is stable — DMA is now writing indices [ADC_DMA_CHANNELS..END] */
        for (int i = 0; i < ADC_DMA_CHANNELS; i++)
        {
            g_adc_snapshot[i]           =   g_adc_dma_buf[i];
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        if (g_pkt_ready)
        {
            g_adc_dropped++;
            return;
        }

        /* Scan 1 is stable — DMA wraps back to index 0 */
        for (int i = ADC_DMA_CHANNELS; i < ADC_DMA_BUF_SIZE; i++)
        {
            g_adc_snapshot[i]           =   g_adc_dma_buf[i];
        }

        g_pkt_ready                     =   1;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        /* Deferred log — main loop picks it up next iteration */
        g_adc_error_code                =   hadc->ErrorCode;
    }
}

/*==============================================================================================*/
