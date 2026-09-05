#ifndef FOC_HAL_SPI_H
#define FOC_HAL_SPI_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int hal_spi_init(uint32_t baud_hz);
int hal_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len);
/* Test hook: next transfer returns these bytes. */
void hal_spi_mock_set_rx(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
