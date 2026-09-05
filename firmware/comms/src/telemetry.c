#include "telemetry.h"

#include <string.h>

static void put_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint32_t get_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void put_f32_le(uint8_t *p, float v)
{
    uint32_t u = 0U;
    (void)memcpy(&u, &v, sizeof(u));
    put_u32_le(p, u);
}

static float get_f32_le(const uint8_t *p)
{
    float v = 0.0f;
    uint32_t u = get_u32_le(p);
    (void)memcpy(&v, &u, sizeof(v));
    return v;
}

void telemetry_init(telemetry_queue_t *q)
{
    if (q == NULL) {
        return;
    }
    (void)memset(q, 0, sizeof(*q));
}

int telemetry_pack(const telemetry_frame_t *f, uint8_t *out, size_t out_cap, size_t *out_len)
{
    if ((f == NULL) || (out == NULL) || (out_len == NULL)) {
        return -1;
    }
    if (out_cap < TELEMETRY_PAYLOAD_SIZE) {
        return -2;
    }
    put_u32_le(&out[0], f->timestamp_ms);
    put_f32_le(&out[4], f->id);
    put_f32_le(&out[8], f->iq);
    put_f32_le(&out[12], f->id_ref);
    put_f32_le(&out[16], f->iq_ref);
    put_f32_le(&out[20], f->theta_e);
    put_f32_le(&out[24], f->omega_m);
    put_f32_le(&out[28], f->vbus);
    *out_len = TELEMETRY_PAYLOAD_SIZE;
    return 0;
}

int telemetry_unpack(const uint8_t *in, size_t in_len, telemetry_frame_t *f)
{
    if ((in == NULL) || (f == NULL)) {
        return -1;
    }
    if (in_len < TELEMETRY_PAYLOAD_SIZE) {
        return -2;
    }
    f->timestamp_ms = get_u32_le(&in[0]);
    f->id = get_f32_le(&in[4]);
    f->iq = get_f32_le(&in[8]);
    f->id_ref = get_f32_le(&in[12]);
    f->iq_ref = get_f32_le(&in[16]);
    f->theta_e = get_f32_le(&in[20]);
    f->omega_m = get_f32_le(&in[24]);
    f->vbus = get_f32_le(&in[28]);
    return 0;
}

int telemetry_push(telemetry_queue_t *q, const telemetry_frame_t *f)
{
    size_t len = 0U;

    if ((q == NULL) || (f == NULL)) {
        return -1;
    }
    if (q->count >= TELEMETRY_QUEUE_DEPTH) {
        /* Drop oldest to keep real-time stream fresh. */
        q->tail = (q->tail + 1U) % TELEMETRY_QUEUE_DEPTH;
        q->count--;
        q->dropped++;
    }
    if (telemetry_pack(f, q->payload[q->head], TELEMETRY_PAYLOAD_SIZE, &len) != 0) {
        return -1;
    }
    (void)len;
    q->head = (q->head + 1U) % TELEMETRY_QUEUE_DEPTH;
    q->count++;
    return 0;
}

int telemetry_pop(telemetry_queue_t *q, telemetry_frame_t *f)
{
    if ((q == NULL) || (f == NULL)) {
        return -1;
    }
    if (q->count == 0U) {
        return -1;
    }
    if (telemetry_unpack(q->payload[q->tail], TELEMETRY_PAYLOAD_SIZE, f) != 0) {
        return -1;
    }
    q->tail = (q->tail + 1U) % TELEMETRY_QUEUE_DEPTH;
    q->count--;
    return 0;
}

uint32_t telemetry_dropped(const telemetry_queue_t *q)
{
    if (q == NULL) {
        return 0U;
    }
    return q->dropped;
}
