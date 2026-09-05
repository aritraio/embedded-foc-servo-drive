#include "drv8301_gate_driver.h"
#include "hal_gpio.h"
#include "hal_spi.h"

#include <stddef.h>

static drv8301_status_t s_shadow = {0U, 0U, 0U, 0U, 0U, 0U};

void drv8301_init(void)
{
    s_shadow.gate_current_ma = 0U;
    s_shadow.oc_level = 0U;
    s_shadow.fault = 0U;
    s_shadow.ot = 0U;
    s_shadow.uv = 0U;
    s_shadow.oc = 0U;
    hal_gpio_write(HAL_PIN_GATE_EN, 1U);
}

int drv8301_read_status(drv8301_status_t *st)
{
    uint8_t tx[2] = {0x80U, 0x00U};
    uint8_t rx[2] = {0U, 0U};

    if (st == NULL) {
        return -1;
    }
    /* FAULT_N pin is the fast path; SPI word is the detail path. */
    if (hal_gpio_read(HAL_PIN_FAULT_N) == 0U) {
        s_shadow.fault = 1U;
    }
    if (hal_spi_transfer(tx, rx, 2U) != 0) {
        return -1;
    }
    {
        const uint16_t w = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
        s_shadow.oc = (uint8_t)((w >> 0) & 1U);
        s_shadow.uv = (uint8_t)((w >> 1) & 1U);
        s_shadow.ot = (uint8_t)((w >> 2) & 1U);
        if ((s_shadow.oc | s_shadow.uv | s_shadow.ot) != 0U) {
            s_shadow.fault = 1U;
        }
        if (hal_gpio_read(HAL_PIN_FAULT_N) == 0U) {
            s_shadow.fault = 1U;
        }
    }
    *st = s_shadow;
    return 0;
}

uint8_t drv8301_has_fault(const drv8301_status_t *st)
{
    if (st == NULL) {
        return 0U;
    }
    return (uint8_t)(st->fault | st->oc | st->ot | st->uv);
}
