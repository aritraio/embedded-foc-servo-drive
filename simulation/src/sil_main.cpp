// SIL desktop runner: closed-loop speed/current demo streaming telemetry frames
// to stdout (COBS-encoded) for tools/foc_tuner.py. No heap in hot loop.
#include "sil_simulator.h"

extern "C" {
#include "packet_protocol.h"
#include "telemetry.h"
}

#include <cstdint>
#include <cstdio>

int main()
{
    sil::PmsmParams motor;
    sil::SilConfig cfg;
    cfg.use_current_loop = true;
    cfg.id_ref = 0.0f;
    cfg.iq_ref = 3.0f;
    cfg.load_torque = 0.02f;
    cfg.current_noise_a = 0.005f;

    sil::SilSimulator sim(motor, cfg);

    // Static TX buffer: zero heap.
    static uint8_t tx_payload[40];
    static uint8_t tx_raw[40];
    static uint8_t tx_enc[64];

    // Run 2 s @ 25 kHz, emit telemetry @ 1 kHz (every 25th step).
    const int total = 25000 * 2;
    for (int i = 0; i < total; i++) {
        // Simple velocity ramp on iq* for demo: 3 A first second, -3 A second.
        if (i == 25000) {
            sim.set_iq_ref(-3.0f);
        }
        sim.step_fast();
        if ((i % 25) == 0) {
            const auto &st = sim.plant().state();
            const auto &fc = sim.controller();
            telemetry_frame_t f{};
            f.timestamp_ms = static_cast<uint32_t>(i / 25);
            f.id = st.id;
            f.iq = st.iq;
            f.id_ref = fc.id_ref;
            f.iq_ref = fc.iq_ref;
            f.theta_e = sim.plant().electrical_angle();
            f.omega_m = st.omega_m;
            f.vbus = motor.vdc_nom;
            size_t payload_len = 0;
            if (telemetry_pack(&f, tx_payload, sizeof(tx_payload), &payload_len) == 0) {
                size_t raw_len = 0;
                if (packet_append_crc(tx_payload, payload_len, tx_raw, sizeof(tx_raw), &raw_len) != 0) {
                    continue;
                }
                size_t enc_len = 0;
                if (packet_encode_cobs(tx_raw, raw_len, tx_enc, sizeof(tx_enc), &enc_len) == 0) {
                    // Delimited COBS frame on stdout (binary-safe via fwrite).
                    std::fwrite(tx_enc, 1, enc_len, stdout);
                    std::fputc(0x00, stdout);
                }
            }
            if ((i % 25000) == 0) {
                std::fprintf(stderr, "t=%.3fs id=%.3f iq=%.3f w=%.1f rad/s\n", sim.time_s(), (double)st.id,
                             (double)st.iq, (double)st.omega_m);
            }
        }
    }
    return 0;
}
