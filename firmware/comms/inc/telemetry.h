#ifndef FOC_TELEMETRY_H
#define FOC_TELEMETRY_H

/* 1 kHz telemetry streaming: packed little-endian frame + CRC32 + COBS.
 * Lock-free single-producer ring; static allocation only. */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEMETRY_PAYLOAD_SIZE (32U)
#define TELEMETRY_QUEUE_DEPTH (16U)
#define TELEMETRY_TX_BUF (80U)

typedef struct {
    uint32_t timestamp_ms;
    float id;
    float iq;
    float id_ref;
    float iq_ref;
    float theta_e;
    float omega_m;
    float vbus;
} telemetry_frame_t;

typedef struct {
    uint8_t payload[TELEMETRY_QUEUE_DEPTH][TELEMETRY_PAYLOAD_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    uint32_t dropped;
} telemetry_queue_t;

void telemetry_init(telemetry_queue_t *q);
/* Pack frame -> 32-byte payload (no CRC yet). Returns 0 on success. */
int telemetry_pack(const telemetry_frame_t *f, uint8_t *out, size_t out_cap, size_t *out_len);
int telemetry_unpack(const uint8_t *in, size_t in_len, telemetry_frame_t *f);
/* Queue one frame (drops oldest on overflow, counts drops). */
int telemetry_push(telemetry_queue_t *q, const telemetry_frame_t *f);
int telemetry_pop(telemetry_queue_t *q, telemetry_frame_t *f);
uint32_t telemetry_dropped(const telemetry_queue_t *q);

#ifdef __cplusplus
}
#endif

#endif
