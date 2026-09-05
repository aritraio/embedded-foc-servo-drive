#include "hal_gpio.h"
#include <stddef.h>

static uint8_t s_out[HAL_PIN_COUNT] = {0U, 1U, 0U};
static uint8_t s_in[HAL_PIN_COUNT] = {0U, 1U, 0U};

void hal_gpio_init(void)
{
    s_out[HAL_PIN_GATE_EN] = 0U;
    s_out[HAL_PIN_LED] = 0U;
    s_in[HAL_PIN_FAULT_N] = 1U;
}

void hal_gpio_write(hal_pin_t pin, uint8_t level)
{
    if ((int32_t)pin < (int32_t)HAL_PIN_COUNT) {
        s_out[(uint32_t)pin] = (level != 0U) ? 1U : 0U;
    }
}

uint8_t hal_gpio_read(hal_pin_t pin)
{
    if ((int32_t)pin >= (int32_t)HAL_PIN_COUNT) {
        return 0U;
    }
    if (pin == HAL_PIN_FAULT_N) {
        return s_in[(uint32_t)pin];
    }
    return s_out[(uint32_t)pin];
}

void hal_gpio_mock_set_input(hal_pin_t pin, uint8_t level)
{
    if ((int32_t)pin < (int32_t)HAL_PIN_COUNT) {
        s_in[(uint32_t)pin] = (level != 0U) ? 1U : 0U;
    }
}
