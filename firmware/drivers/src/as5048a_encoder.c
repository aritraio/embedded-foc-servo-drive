#include "as5048a_encoder.h"
#include "hal_spi.h"

#include <stddef.h>

#define AS5048A_NOP_HI (0x00U)
#define AS5048A_NOP_LO (0x00U)
#define AS5048A_READ_FLAG (0x4000U)
#define AS5048A_PARITY_BIT (0x8000U)
#define AS5048A_ERROR_BIT (0x4000U)
#define AS5048A_ANGLE_MASK (0x3FFFU)

static uint32_t s_reads = 0U;
static uint32_t s_errors = 0U;

static uint8_t even_parity15(uint16_t v15)
{
    uint8_t p = 0U;
    uint16_t v = v15;
    while (v != 0U) {
        p ^= (uint8_t)(v & 1U);
        v >>= 1;
    }
    return p; /* 1 if odd number of 1s */
}

uint8_t as5048a_check_parity(uint16_t frame)
{
    uint16_t payload = (uint16_t)(frame & 0x7FFFU);
    uint8_t par = (frame & AS5048A_PARITY_BIT) != 0U ? 1U : 0U;
    /* Even parity: parity bit set iff payload has odd popcount. */
    return (par == even_parity15(payload)) ? 1U : 0U;
}

uint16_t as5048a_build_frame(uint16_t angle14, uint8_t error_flag)
{
    uint16_t payload = (uint16_t)(angle14 & AS5048A_ANGLE_MASK);
    if (error_flag != 0U) {
        payload |= AS5048A_ERROR_BIT;
        payload &= (uint16_t)~AS5048A_ANGLE_MASK; /* error frame carries flags, keep simple */
        payload |= (uint16_t)(angle14 & 0x3FFFU);
        payload |= AS5048A_ERROR_BIT;
    }
    if (even_parity15(payload) != 0U) {
        payload |= AS5048A_PARITY_BIT;
    }
    return payload;
}

void as5048a_init(void)
{
    s_reads = 0U;
    s_errors = 0U;
    (void)hal_spi_init(10000000U);
}

int as5048a_read(as5048a_reading_t *out)
{
    /* AS5048A protocol: send READ command for angle register, then clock out
     * 16-bit response on the following transfer. Host mock collapses this to
     * a single 2-byte transfer returning the pre-programmed frame. */
    uint8_t tx[2] = {0xFFU, 0xFFU};
    uint8_t rx[2] = {0U, 0U};
    uint16_t frame = 0U;

    if (out == NULL) {
        return -1;
    }
    if (hal_spi_transfer(tx, rx, 2U) != 0) {
        s_errors++;
        out->comm_ok = 0U;
        out->parity_ok = 0U;
        out->error_flag = 1U;
        out->error_count = s_errors;
        out->read_count = s_reads;
        return -1;
    }
    frame = (uint16_t)(((uint16_t)rx[0] << 8) | (uint16_t)rx[1]);
    s_reads++;

    out->read_count = s_reads;
    out->comm_ok = 1U;
    out->parity_ok = as5048a_check_parity(frame);
    out->error_flag = ((frame & AS5048A_ERROR_BIT) != 0U) ? 1U : 0U;
    out->raw = (uint16_t)(frame & AS5048A_ANGLE_MASK);
    out->angle_rad = ((float)out->raw / 16384.0f) * 6.28318530717958647692f;

    if ((out->parity_ok == 0U) || (out->error_flag != 0U)) {
        s_errors++;
        out->error_count = s_errors;
        return -1;
    }
    out->error_count = s_errors;
    return 0;
}
