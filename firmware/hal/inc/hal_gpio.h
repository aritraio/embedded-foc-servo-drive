#ifndef FOC_HAL_GPIO_H
#define FOC_HAL_GPIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HAL_PIN_GATE_EN = 0,
    HAL_PIN_FAULT_N = 1,
    HAL_PIN_LED = 2,
    HAL_PIN_COUNT = 3
} hal_pin_t;

void hal_gpio_init(void);
void hal_gpio_write(hal_pin_t pin, uint8_t level);
uint8_t hal_gpio_read(hal_pin_t pin);
/* Test hook: drive input pins (e.g. FAULT_N low = gate-driver fault). */
void hal_gpio_mock_set_input(hal_pin_t pin, uint8_t level);

#ifdef __cplusplus
}
#endif

#endif
