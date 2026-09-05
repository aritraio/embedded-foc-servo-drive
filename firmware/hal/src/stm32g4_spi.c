#include "hal_spi.h"
#include <string.h>

static uint8_t s_mock_rx[64];
static size_t s_mock_len = 0;
static uint8_t s_has_mock = 0;

int hal_spi_init(uint32_t baud_hz)
{
    (void)baud_hz;
#if defined(STM32_TARGET)
    /* Real target: SPI1 master, 8-bit, CPOL=1 CPHA=1 (AS5048A mode 1),
     * baud = fPCLK/prescaler closest to 10 MHz. */
#endif
    return 0;
}

int hal_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    size_t i = 0U;

    if (((tx == NULL) && (rx == NULL)) || (len == 0U) || (len > 64U)) {
        return -1;
    }
    if (s_has_mock != 0U) {
        for (i = 0U; i < len; i++) {
            uint8_t v = (i < s_mock_len) ? s_mock_rx[i] : 0x00U;
            if (rx != NULL) {
                rx[i] = v;
            }
        }
        return 0;
    }
    /* Loopback default on host. */
    for (i = 0U; i < len; i++) {
        if (rx != NULL) {
            rx[i] = (tx != NULL) ? tx[i] : 0xFFU;
        }
    }
    return 0;
}

void hal_spi_mock_set_rx(const uint8_t *data, size_t len)
{
    size_t n = len;
    if (n > sizeof(s_mock_rx)) {
        n = sizeof(s_mock_rx);
    }
    if ((data != NULL) && (n > 0U)) {
        (void)memcpy(s_mock_rx, data, n);
    }
    s_mock_len = n;
    s_has_mock = 1U;
}
