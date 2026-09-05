#ifndef FOC_AS5048A_ENCODER_H
#define FOC_AS5048A_ENCODER_H

/* AS5048A 14-bit magnetic encoder over 10 MHz SPI mode 1.
 * Includes even-parity check, error-flag decoding, dropout detection. */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5048A_RESOLUTION (16384U)
#define AS5048A_ANGLE_REG  (0x3FFFU)

typedef struct {
    uint16_t raw;        /* 14-bit angle */
    float angle_rad;     /* [0, 2pi) */
    uint8_t parity_ok;
    uint8_t error_flag;
    uint8_t comm_ok;
    uint32_t error_count;
    uint32_t read_count;
} as5048a_reading_t;

void as5048a_init(void);
/* Returns 0 on valid read (parity ok, no error flag), <0 on fault. */
int as5048a_read(as5048a_reading_t *out);
/* Pure helper: verify even parity of 16-bit frame (bit15 = parity). */
uint8_t as5048a_check_parity(uint16_t frame);
/* Build a mock 16-bit response frame for tests. */
uint16_t as5048a_build_frame(uint16_t angle14, uint8_t error_flag);

#ifdef __cplusplus
}
#endif

#endif
