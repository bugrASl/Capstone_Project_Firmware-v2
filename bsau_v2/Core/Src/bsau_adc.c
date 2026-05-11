/**
 *  @file   bsau_adc.c
 *  @brief  BSAU ADC driver — 8-channel DMA acquisition, battery measurement.
 *
 *  Configures STM32L4 ADC1 for scan-mode acquisition across 8 EMG channels
 *  plus one battery voltage channel. DMA transfers complete buffers to the
 *  application layer at the TIM6-driven 2 kHz sample rate.
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

/* ADC scan-order remap.
 *
 *  Scan sequence (PA4 <-> PB0 swap applied):
 *
 *      Rank 1 = CH5 (PA0)   Rank 2 = CH6 (PA1)   Rank 3 = CH7 (PA2)
 *      Rank 4 = CH8 (PA3)   Rank 5 = CH15 (PB0)  Rank 6 = CH10 (PA5)
 *      Rank 7 = CH11 (PA6)  Rank 8 = CH12 (PA7)  Rank 9 = CH9 (PA4/batt)
 *
 *  KEY FINDING: The STM32L4 ADC DMA reorder is CHANNEL-NUMBER-BASED,
 *  not rank-based. Regardless of which rank a channel is assigned to,
 *  it always lands at the same DMA slot determined by its channel number.
 *  Swapping ranks does NOT move the DMA slot.
 *
 *  Actual DMA slot order (by channel number):
 *      slot 0 = CH5  (PA0)     slot 1 = CH6  (PA1)
 *      slot 2 = CH12 (PA7)     slot 3 = CH7  (PA2)
 *      slot 4 = CH8  (PA3)     slot 5 = CH9  (PA4/batt)
 *      slot 6 = CH10 (PA5)     slot 7 = CH11 (PA6)
 *      slot 8 = CH15 (PB0)
 *
 *  The table reads "logical channel i lives at DMA slot s_dma_slot[i]".
 */
static const uint8_t s_dma_slot[ADC_DMA_CHANNELS] = {
    0,      /* logical 0 (PA0)   = DMA slot 0  (CH5)  */
    1,      /* logical 1 (PA1)   = DMA slot 1  (CH6)  */
    3,      /* logical 2 (PA2)   = DMA slot 3  (CH7)  */
    4,      /* logical 3 (PA3)   = DMA slot 4  (CH8)  */
    8,      /* logical 4 (PB0)   = DMA slot 8  (CH15) */
    6,      /* logical 5 (PA5)   = DMA slot 6  (CH10) */
    7,      /* logical 6 (PA6)   = DMA slot 7  (CH11) */
    2,      /* logical 7 (PA7)   = DMA slot 2  (CH12) */
    5,      /* logical 8 (batt)  = DMA slot 5  (CH9)  */
};

/*============= BSAU_ADC_Init ==================================================================*/

void BSAU_ADC_Init(void)
{
    LOG("ADC", "BSAU_ADC_Init", "RUN", "Calibrating ADC1...");

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
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
 *  race fix:
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

        /* Scan 0 is stable — DMA is now writing indices [ADC_DMA_CHANNELS..END].
         * Apply the channel remap so g_adc_snapshot[i] == logical channel i. */
        for (int i = 0; i < ADC_DMA_CHANNELS; i++)
        {
            g_adc_snapshot[i]           =   g_adc_dma_buf[s_dma_slot[i]];
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

        /* Scan 1 is stable — DMA wraps back to index 0. Apply the same
         * channel remap as in HAL_ADC_ConvHalfCpltCallback, but offset by
         * ADC_DMA_CHANNELS in both source and destination. */
        for (int i = 0; i < ADC_DMA_CHANNELS; i++)
        {
            g_adc_snapshot[ADC_DMA_CHANNELS + i] =
                g_adc_dma_buf[ADC_DMA_CHANNELS + s_dma_slot[i]];
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

