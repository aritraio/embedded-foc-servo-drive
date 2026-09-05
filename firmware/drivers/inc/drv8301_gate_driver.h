#ifndef FOC_DRV8301_H
#define FOC_DRV8301_H

/* DRV8301 3-phase gate-driver: SPI config + fault diagnostics. */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t gate_current_ma; /* 0..1700 (approx buckets) */
    uint8_t oc_level;        /* overcurrent threshold bucket */
    uint8_t fault;           /* latched fault bit */
    uint8_t ot;              /* overtemperature */
    uint8_t uv;              /* undervoltage */
    uint8_t oc;              /* overcurrent */
} drv8301_status_t;

void drv8301_init(void);
int drv8301_read_status(drv8301_status_t *st);
uint8_t drv8301_has_fault(const drv8301_status_t *st);

#ifdef __cplusplus
}
#endif

#endif
