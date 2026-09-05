#include "hal_adc.h"
#include <stddef.h>

static hal_adc_state_t s_adc = {{0}, {0}, 0U, 0U, 0.0f, 0.0f};

int hal_adc_init(float fullscale_a)
{
    if (fullscale_a <= 0.0f) {
        return -1;
    }
    for (uint32_t i = 0U; i < HAL_ADC_BUF_LEN; i++) {
        s_adc.ia_raw[i] = 0;
        s_adc.ib_raw[i] = 0;
    }
    s_adc.head = 0U;
    s_adc.streaming = 0U;
    /* 12-bit bipolar: LSB = 2*FS/4096. */
    s_adc.ia_scale = (2.0f * fullscale_a) / 4096.0f;
    s_adc.ib_scale = (2.0f * fullscale_a) / 4096.0f;
#if defined(STM32_TARGET)
    /* Real target: ADC1+ADC2 dual regular simultaneous, TRGO on TIM1 update,
     * DMA1 circular into ia/ib interleave buffer. */
#endif
    return 0;
}

void hal_adc_start_dma(void)
{
    s_adc.streaming = 1U;
}

void hal_adc_stop_dma(void)
{
    s_adc.streaming = 0U;
}

void hal_adc_inject_sample(int16_t ia_raw, int16_t ib_raw)
{
    s_adc.ia_raw[s_adc.head] = ia_raw;
    s_adc.ib_raw[s_adc.head] = ib_raw;
    s_adc.head = (s_adc.head + 1U) % HAL_ADC_BUF_LEN;
}

uint8_t hal_adc_read_latest(float *ia, float *ib)
{
    uint32_t idx = 0U;

    if ((ia == NULL) || (ib == NULL) || (s_adc.streaming == 0U)) {
        return 0U;
    }
    idx = (s_adc.head + HAL_ADC_BUF_LEN - 1U) % HAL_ADC_BUF_LEN;
    *ia = (float)s_adc.ia_raw[idx] * s_adc.ia_scale;
    *ib = (float)s_adc.ib_raw[idx] * s_adc.ib_scale;
    return 1U;
}

const hal_adc_state_t *hal_adc_state(void)
{
    return &s_adc;
}
