# FOC Drive Architecture

## Control hierarchy (multi-rate, deterministic)

| Loop | Rate | Period | Job |
|------|------|--------|-----|
| Current (inner) | 25 kHz | 40 us | Clarke/Park, decoupled d/q PI + feed-forward, SVPWM |
| Velocity | 2.5 kHz | 400 us | PI + accel feed-forward + disturbance-observer stiffness |
| Position | 1 kHz | 1 ms | P + S-curve trajectory profiling |
| Telemetry | 1 kHz | 1 ms | COBS+CRC32 streaming of id/iq/refs/theta/omega/vbus |

ISR flow (TIM1 update -> dual ADC TRGO at PWM valley -> DMA complete):

1. `hal_adc_read_latest()` — valley-sampled ia/ib (low-side FETs all ON at
   zero vector V0 midpoint, minimal di/dt ringing).
2. `as5048a_read()` — 14-bit SPI @ 10 MHz (<2 us), parity-checked.
3. `foc_app_supervisor()` — OC/OV/UV/thermal/encoder checks; trips PWM on fault.
4. `foc_app_step_fast()` — FOC core + `hal_pwm_set_duties()`.
5. Decimated velocity (÷10) and position (÷25) steps.

## Key equations

- Clarke: iα = ia, iβ = (ia + 2ib)/√3.
- Park/inverse Park with θe = P·θm − θoffset.
- Decoupling: Vd* = PI_d − ωe·Lq·iq, Vq* = PI_q + ωe·(Ld·id + λpm).
- Circle limit: √(Vd²+Vq²) ≤ Vdc/√3.
- PMSM RK4 plant: did/dt = (Vd − Rs·id + ωe·Lq·iq)/Ld,
  diq/dt = (Vq − Rs·iq − ωe·Ld·id − ωe·λpm)/Lq,
  Te = 1.5·P·(λpm·iq + (Ld−Lq)·id·iq), dω/dt = (Te − TL − B·ω)/J.

## Timing budget @ 170 MHz (STM32G474)

40 us period = 6800 cycles. Measured-style budget: ADC DMA (0 CPU),
encoder SPI 2 us (340 cycles, DMA-assisted), FOC math ~0.6 us,
PI+SVPWM ~0.4 us — total ISR < 1.5 us, >95% headroom for background tasks.
