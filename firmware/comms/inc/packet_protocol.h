#ifndef FOC_PACKET_PROTOCOL_H
#define FOC_PACKET_PROTOCOL_H

/* COBS framing + CRC32 integrity. Zero heap; caller-owned buffers.
 * Wire format: COBS(payload_with_crc) + 0x00 delimiter.
 * Payload = raw struct bytes + LE CRC32(payload) appended. */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PACKET_MAX_PAYLOAD (128U)
#define PACKET_OVERHEAD (4U) /* CRC32 */

uint32_t packet_crc32(const uint8_t *data, size_t len);

/* Returns 0 on success, <0 on bad args / too small output. */
int packet_encode_cobs(const uint8_t *in, size_t in_len, uint8_t *out,
                       size_t out_cap, size_t *out_len);
int packet_decode_cobs(const uint8_t *in, size_t in_len, uint8_t *out,
                       size_t out_cap, size_t *out_len);

/* CRC helpers: append LE CRC32, verify + strip. */
int packet_append_crc(const uint8_t *payload, size_t payload_len, uint8_t *out,
                      size_t out_cap, size_t *out_len);
int packet_verify_crc(const uint8_t *framed, size_t framed_len, uint8_t *payload_out,
                      size_t payload_cap, size_t *payload_len);

#ifdef __cplusplus
}
#endif

#endif
