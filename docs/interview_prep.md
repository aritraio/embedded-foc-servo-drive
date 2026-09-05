# Interview Prep (Tier-1 Robotics / Firmware)

1. Why sample currents at the PWM valley? — Low-side shunts conduct only when
   low FETs are ON; center-aligned valley = V0(000) midpoint, all low FETs ON,
   minimal switching transient.
2. SVPWM vs SPWM? — Third-harmonic common-mode injection → Vdc/√3 vs Vdc/2
   (+15.5%), lower THD.
3. Cross-coupling decoupling? — Rotating frame induces ωe·Lq·iq / ωe·Ld·id
   speed voltages; feed-forward cancels them for high-speed tracking.
4. Anti-windup? — Conditional integration (freeze when saturated & error pushes
   deeper) + integrator clamps; output recomputed post-clamp each step.
5. Dead-time effect? — Voltage error ≈ Tdead/Tpwm·Vdc opposing current polarity;
   modeled in SIL as dq-projected drop.
6. 14-bit encoder quantization? — 0.022° LSB; velocity estimated via observer,
   not raw differentiation.
7. Why RK4 over Euler in SIL? — Electrical time constant L/R ≈ 1.4 ms needs
   stability at 40 us; RK4 gives 4th-order accuracy for stiff dq dynamics.
8. Circle limitation? — Preserve vector angle, scale magnitude to Vdc/√3.
9. S-curve vs trapezoidal? — Bounded jerk → continuous acceleration, no
   excitation of mechanical resonances.
10. Disturbance observer? — Luenberger estimating load torque; adds stiffness
    without raising PD gains.
(Plus 10 more in-code: CORDIC use, DMA circular, break input, CRC framing...)
