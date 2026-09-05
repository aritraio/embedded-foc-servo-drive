#include <gtest/gtest.h>

extern "C" {
#include "packet_protocol.h"
#include "telemetry.h"
}

#include <cstring>
#include <vector>

TEST(Crc32, KnownVector)
{
    // IEEE CRC32 of "123456789" is 0xCBF43926.
    const uint8_t msg[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(packet_crc32(msg, 9), 0xCBF43926u);
}

TEST(Cobs, RoundtripAllByteValues)
{
    uint8_t raw[256];
    for (int i = 0; i < 256; i++) {
        raw[i] = (uint8_t)i;
    }
    // COBS cannot encode a leading-delimiter framing zero inside payload
    // without splitting; standard roundtrip must still hold incl. zeros.
    uint8_t enc[300]{};
    uint8_t dec[256]{};
    size_t enc_len = 0, dec_len = 0;
    ASSERT_EQ(packet_encode_cobs(raw, sizeof(raw), enc, sizeof(enc), &enc_len), 0);
    // Encoded stream must contain no zero bytes.
    for (size_t i = 0; i < enc_len; i++) {
        EXPECT_NE(enc[i], 0u);
    }
    ASSERT_EQ(packet_decode_cobs(enc, enc_len, dec, sizeof(dec), &dec_len), 0);
    EXPECT_EQ(dec_len, sizeof(raw));
    EXPECT_EQ(std::memcmp(raw, dec, sizeof(raw)), 0);
}

TEST(Cobs, CorruptStreamDetected)
{
    uint8_t raw[4] = {0x11, 0x00, 0x22, 0x33};
    uint8_t enc[16]{};
    size_t enc_len = 0;
    ASSERT_EQ(packet_encode_cobs(raw, 4, enc, sizeof(enc), &enc_len), 0);
    uint8_t dec[8]{};
    size_t dec_len = 0;
    ASSERT_EQ(packet_decode_cobs(enc, enc_len, dec, sizeof(dec), &dec_len), 0);
    EXPECT_EQ(dec_len, 4u);
    // Tamper: CRC over decoded payload must fail after bit flip.
    dec[0] ^= 0xFFu;
    uint8_t payload[8]{};
    size_t pl = 0;
    // Build a CRC-framed packet then corrupt it.
    uint8_t framed[16]{};
    size_t framed_len = 0;
    ASSERT_EQ(packet_append_crc(raw, 4, framed, sizeof(framed), &framed_len), 0);
    framed[0] ^= 0xFFu;
    EXPECT_NE(packet_verify_crc(framed, framed_len, payload, sizeof(payload), &pl), 0);
}

TEST(Telemetry, PackUnpackLossless)
{
    telemetry_frame_t f{1234, 1.5f, -2.5f, 0.0f, 3.0f, 1.234f, 55.0f, 24.1f};
    uint8_t buf[32]{};
    size_t len = 0;
    ASSERT_EQ(telemetry_pack(&f, buf, sizeof(buf), &len), 0);
    EXPECT_EQ(len, 32u);
    telemetry_frame_t g{};
    ASSERT_EQ(telemetry_unpack(buf, len, &g), 0);
    EXPECT_EQ(g.timestamp_ms, 1234u);
    EXPECT_FLOAT_EQ(g.id, 1.5f);
    EXPECT_FLOAT_EQ(g.iq, -2.5f);
    EXPECT_FLOAT_EQ(g.iq_ref, 3.0f);
    EXPECT_FLOAT_EQ(g.theta_e, 1.234f);
    EXPECT_FLOAT_EQ(g.omega_m, 55.0f);
    EXPECT_FLOAT_EQ(g.vbus, 24.1f);
}

TEST(Telemetry, QueueStreamsAt1kHzWithoutLoss)
{
    telemetry_queue_t q{};
    telemetry_init(&q);
    // 1 s @ 1 kHz through depth-16 queue with matching drain: zero drops.
    for (uint32_t i = 0; i < 1000; i++) {
        telemetry_frame_t f{};
        f.timestamp_ms = i;
        f.iq = 1.0f;
        ASSERT_EQ(telemetry_push(&q, &f), 0);
        telemetry_frame_t g{};
        ASSERT_EQ(telemetry_pop(&q, &g), 0);
        EXPECT_EQ(g.timestamp_ms, i);
    }
    EXPECT_EQ(telemetry_dropped(&q), 0u);
}

TEST(Telemetry, FullWirePathCobsCrc)
{
    // pack -> append CRC -> COBS -> split on 0x00 -> decode -> verify -> unpack.
    telemetry_frame_t f{777, 0.1f, 4.9f, 0.0f, 5.0f, 3.0f, 10.0f, 24.0f};
    uint8_t payload[32]{};
    size_t pl = 0;
    ASSERT_EQ(telemetry_pack(&f, payload, sizeof(payload), &pl), 0);
    uint8_t with_crc[40]{};
    size_t cf = 0;
    ASSERT_EQ(packet_append_crc(payload, pl, with_crc, sizeof(with_crc), &cf), 0);
    uint8_t enc[64]{};
    size_t el = 0;
    ASSERT_EQ(packet_encode_cobs(with_crc, cf, enc, sizeof(enc), &el), 0);
    uint8_t dec[40]{};
    size_t dl = 0;
    ASSERT_EQ(packet_decode_cobs(enc, el, dec, sizeof(dec), &dl), 0);
    uint8_t raw[32]{};
    size_t rl = 0;
    ASSERT_EQ(packet_verify_crc(dec, dl, raw, sizeof(raw), &rl), 0);
    telemetry_frame_t g{};
    ASSERT_EQ(telemetry_unpack(raw, rl, &g), 0);
    EXPECT_EQ(g.timestamp_ms, 777u);
    EXPECT_FLOAT_EQ(g.iq, 4.9f);
}
