#include "packet_protocol.h"

uint32_t packet_crc32(const uint8_t *data, size_t len)
{
    /* Bitwise IEEE 802.3 CRC32 (poly 0xEDB88320); table-free, deterministic. */
    uint32_t crc = 0xFFFFFFFFU;
    size_t i = 0U;

    if (data == NULL) {
        return 0U;
    }
    for (i = 0U; i < len; i++) {
        uint32_t b = data[i];
        crc ^= b;
        for (int k = 0; k < 8; k++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1) ^ 0xEDB88320U;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

int packet_encode_cobs(const uint8_t *in, size_t in_len, uint8_t *out,
                       size_t out_cap, size_t *out_len)
{
    size_t read_idx = 0U;
    size_t write_idx = 1U;
    size_t code_idx = 0U;
    uint8_t code = 1U;

    if ((in == NULL) || (out == NULL) || (out_len == NULL)) {
        return -1;
    }
    if (in_len == 0U) {
        return -1;
    }
    if (out_cap < (in_len + in_len / 254U + 2U)) {
        return -2;
    }

    while (read_idx < in_len) {
        if (in[read_idx] == 0U) {
            out[code_idx] = code;
            code_idx = write_idx++;
            code = 1U;
            read_idx++;
        } else {
            out[write_idx++] = in[read_idx++];
            code++;
            if (code == 0xFFU) {
                out[code_idx] = code;
                code_idx = write_idx++;
                code = 1U;
            }
        }
    }
    out[code_idx] = code;
    *out_len = write_idx;
    return 0;
}

int packet_decode_cobs(const uint8_t *in, size_t in_len, uint8_t *out,
                       size_t out_cap, size_t *out_len)
{
    size_t read_idx = 0U;
    size_t write_idx = 0U;

    if ((in == NULL) || (out == NULL) || (out_len == NULL)) {
        return -1;
    }
    if (in_len == 0U) {
        return -1;
    }

    while (read_idx < in_len) {
        uint8_t code = in[read_idx++];
        size_t i = 0U;

        if (code == 0U) {
            return -3; /* zero byte inside COBS stream: corrupt */
        }
        if ((read_idx + (size_t)(code - 1U)) > in_len) {
            return -3;
        }
        for (i = 1U; i < (size_t)code; i++) {
            if (write_idx >= out_cap) {
                return -2;
            }
            out[write_idx++] = in[read_idx++];
        }
        if ((code != 0xFFU) && (read_idx < in_len)) {
            if (write_idx >= out_cap) {
                return -2;
            }
            out[write_idx++] = 0U;
        }
    }
    *out_len = write_idx;
    return 0;
}

int packet_append_crc(const uint8_t *payload, size_t payload_len, uint8_t *out,
                      size_t out_cap, size_t *out_len)
{
    uint32_t crc = 0U;
    size_t i = 0U;

    if ((payload == NULL) || (out == NULL) || (out_len == NULL)) {
        return -1;
    }
    if ((payload_len + 4U) > out_cap) {
        return -2;
    }
    if ((payload_len + 4U) > PACKET_MAX_PAYLOAD + PACKET_OVERHEAD) {
        return -2;
    }
    for (i = 0U; i < payload_len; i++) {
        out[i] = payload[i];
    }
    crc = packet_crc32(payload, payload_len);
    out[payload_len] = (uint8_t)(crc & 0xFFU);
    out[payload_len + 1U] = (uint8_t)((crc >> 8) & 0xFFU);
    out[payload_len + 2U] = (uint8_t)((crc >> 16) & 0xFFU);
    out[payload_len + 3U] = (uint8_t)((crc >> 24) & 0xFFU);
    *out_len = payload_len + 4U;
    return 0;
}

int packet_verify_crc(const uint8_t *framed, size_t framed_len, uint8_t *payload_out,
                      size_t payload_cap, size_t *payload_len)
{
    uint32_t got = 0U;
    uint32_t exp = 0U;

    if ((framed == NULL) || (payload_out == NULL) || (payload_len == NULL)) {
        return -1;
    }
    if (framed_len < 4U) {
        return -3;
    }
    {
        const size_t pl = framed_len - 4U;
        size_t i = 0U;
        if (pl > payload_cap) {
            return -2;
        }
        for (i = 0U; i < pl; i++) {
            payload_out[i] = framed[i];
        }
        got = (uint32_t)framed[pl] | ((uint32_t)framed[pl + 1U] << 8) |
              ((uint32_t)framed[pl + 2U] << 16) | ((uint32_t)framed[pl + 3U] << 24);
        exp = packet_crc32(payload_out, pl);
        if (got != exp) {
            return -4;
        }
        *payload_len = pl;
        return 0;
    }
}
