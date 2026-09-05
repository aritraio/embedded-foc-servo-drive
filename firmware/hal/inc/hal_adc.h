#ifndef FOC_HAL_ADC_H
#define FOC_HAL_ADC_H

/* Dual simultaneous ADC triggered by TIM1 TRGO at PWM valley + circular DMA. */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_ADC_BUF_LEN (64U)

typedef struct {
    int16_t ia_raw[HAL_ADC_BUF_LEN];
    int16_t ib_raw[HAL_ADC_BUF_LEN];
    uint32_t head;
    uint8_t streaming;
    float ia_scale; /* A per LSB */
    float ib_scale;
} hal_adc_state_t;

int hal_adc_init(float fullscale_a);
void hal_adc_start_dma(void);
void hal_adc_stop_dma(void);
/* Mock injection for SIL/tests: push a sample pair as if DMA just wrote it. */
void hal_adc_inject_sample(int16_t ia_raw, int16_t ib_raw);
uint8_t hal_adc_read_latest(float *ia, float *ib);
const hal_adc_state_t *hal_adc_state(void);

#ifdef __cplusplus
}
#endif

#endif
