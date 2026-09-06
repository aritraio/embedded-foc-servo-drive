# Technology Stack & Tooling Architecture

This document provides a comprehensive breakdown of the languages, silicon hardware, power electronics, digital signal processing algorithms, simulation mathematics, and development tools employed across the **Embedded FOC Servo Motor Drive**.

---

## 1. Programming Languages & Standards

| Technology | Standard / Version | Architectural Role | Implementation Rationale |
|---|---|---|---|
| **C Language** | **C11 (ISO/IEC 9899:2011)** | Bare-Metal Firmware (`firmware/`) | Deterministic execution, zero dynamic heap allocations (`malloc`/`free` strictly banned in hot-paths), direct register manipulation, MISRA-C aligned safety patterns. |
| **C++ Language** | **C++20 (ISO/IEC 14882:2020)** | SIL Simulation (`simulation/`) & Tests (`tests/`) | High-fidelity continuous physics simulation, strongly-typed state modeling, RAII resources, GoogleTest integration, operator overloading for state vectors. |
| **Python** | **Python 3.10+** | Telemetry Scope & Calibration (`tools/`) | Rapid real-time data visualization, step response performance analysis, automated electrical angle calibration over serial/socket. |

---

## 2. Target Silicon & Microcontroller Peripherals

The primary embedded target is the **STM32G474RET6** (STMicroelectronics), a high-performance MCU engineered specifically for digital power conversion and motor actuation:

```text
┌────────────────────────────────────────────────────────────────────────┐
│                        STM32G474RET6 ARCHITECTURE                      │
│                                                                        │
│   ARM Cortex-M4 Core @ 170 MHz                                         │
│   ├── Single-Precision Hardware FPU (IEEE 754)                         │
│   ├── DSP Extension Instructions (SIMD multiply-accumulate)            │
│   ├── Hardware CORDIC Coprocessor (Trigonometric sin/cos acceleration) │
│   └── 512 KB Dual-Bank Flash Memory + 128 KB SRAM with Parity          │
│                                                                        │
│   Synchronized Motor Control Peripherals:                              │
│   ├── Advanced Timer 1 (TIM1): 25 kHz Center-Aligned PWM with Deadtime │
│   ├── Dual Fast ADCs (ADC1 & ADC2): 12-Bit, 4.0 MSPS simultaneous      │
│   ├── Direct Memory Access (DMA1): Circular buffer for dual ADC        │
│   ├── High-Speed SPI (SPI1): 10 MHz Master for AS5048A 14-bit encoder  │
│   └── Analog Comparators (COMP1/COMP2): Hardware TIM1_BKIN trip (<1 µs)│
└────────────────────────────────────────────────────────────────────────┘
```

### Key Peripheral Mappings & Configurations:
- **TIM1 (Advanced Motor Timer):**
  - Mode: Center-Aligned Mode 1 (up-down counting).
  - Clock: 170 MHz system clock; Auto-Reload Register ($\text{ARR} = 3400$).
  - Complementary Channels: `TIM1_CH1`/`CH1N` (PA8/PA7), `TIM1_CH2`/`CH2N` (PA9/PB0), `TIM1_CH3`/`CH3N` (PA10/PB1).
  - Dead-Time Generator: $100\,\text{ns}$ (17 clock cycles @ 170 MHz).
  - Trigger Output (`TRGO`): Generated at counter valley (underflow) to trigger ADC conversion.
- **Dual ADC (ADC1 & ADC2):**
  - Mode: Regular simultaneous dual conversion mode.
  - Conversion Speed: $0.25\,\mu\text{s}$ conversion time at 12-bit resolution.
  - Channels: ADC1 on Phase A low-side shunt; ADC2 on Phase B low-side shunt.
  - DMA: `DMA1_Channel1` circular buffer writing packed dual 16-bit ADC samples directly to SRAM.
- **SPI1 (Magnetic Angle Feedback):**
  - Mode: Full-duplex Master, CPOL=0, CPHA=1 (SPI Mode 1).
  - Baud Rate: 10.625 MHz ($f_{PCLK}/16$).
  - Packet Format: 16-bit frame with even parity bit.
- **Hardware Break Line (`TIM1_BKIN`):**
  - Overcurrent analog comparator output wired directly to TIM1 emergency shutoff pin.
  - Latches PWM into tri-state / low-side dump within $< 1.0\,\mu\text{s}$ independently of software execution.

---

## 3. Power Electronics & Sensors

- **Inverter Stage:** 3-Phase Half-Bridge Inverter utilizing low-$R_{DS(on)}$ N-channel power MOSFETs (rated 60 V, 40 A continuous).
- **Gate Driver:** TI DRV8301 / STSPIN with programmable dead-time, hardware charge pump, dual low-noise internal current-sense shunt amplifiers, and SPI diagnostics.
- **Current Shunts:** 3 low-side current shunts ($R_{\text{shunt}} = 10\,\text{m}\Omega$, $1\%$ tolerance, temperature coefficient $< 50\,\text{ppm}/^\circ\text{C}$) paired with differential operational amplifiers (gain $G = 10\,\text{V/V}$).
- **Absolute Encoder:** AMS AS5048A 14-bit magnetic rotary encoder:
  - Resolution: 16,384 positions per revolution ($0.022^\circ$ angular resolution).
  - Interface: 10 MHz SPI with even parity and diagnostic error flags.
- **Target Motor Actuator:** High-torque density Permanent Magnet Synchronous Motor (PMSM) / BLDC:
  - Pole Pairs ($P$): 4
  - Phase Stator Resistance ($R_s$): $0.18\,\Omega$
  - Direct & Quadrature Inductance ($L_d, L_q$): $120\,\mu\text{H}$
  - Permanent Magnet Flux Linkage ($\lambda_{pm}$): $0.005\,\text{Wb}$
  - Nominal DC Bus Voltage ($V_{dc}$): $24.0\,\text{V}$ (range: 12.0 V – 48.0 V)

---

## 4. Control Algorithms & Mathematical Engines

```text
Discrete Control Pipeline (25 kHz Inner / 2.5 kHz Middle / 1 kHz Outer):

[Position Target θ*]
         │
         ▼  (1 kHz)
┌────────────────────────────────────────────────────────┐
│ 7-Segment Jerk-Bounded S-Curve Trajectory Profiler     │
│ Enforces: Jmax, Amax, Vmax  ──► Smooth Target θ*, ω*   │
└────────────────────────┬───────────────────────────────┘
                         │ Position Error (θ* - θ)
                         ▼
┌────────────────────────────────────────────────────────┐
│ Outer Position P-Controller + Velocity Feed-Forward    │
└────────────────────────┬───────────────────────────────┘
                         │ Velocity Target ω*
                         ▼  (2.5 kHz)
┌────────────────────────────────────────────────────────┐
│ Middle Velocity PI-Regulator + Disturbance Observer    │
│ Luenberger observer estimates load torque TL           │
└────────────────────────┬───────────────────────────────┘
                         │ Target Torque / Current Iq*
                         ▼  (25 kHz)
┌────────────────────────────────────────────────────────┐
│ Inner Field-Oriented Control (FOC) Loop                │
│ • Clarke: (ia, ib, ic) ──► (iα, iβ)                    │
│ • Park: (iα, iβ, θe) ──► (id, iq)                      │
│ • Parallel PI Regulators with Anti-Windup Clamping     │
│ • Decoupling: -ωe·Lq·iq  and  +ωe·(Ld·id + λpm)        │
│ • Circle Voltage Clamping: |Vdq*| ≤ Vdc / √3           │
│ • Inverse Park: (Vd*, Vq*, θe) ──► (Vα*, Vβ*)          │
│ • Space Vector PWM (SVPWM): Dwell times T1, T2, T0     │
│ • PWM Phase Compare Registers: Ta, Tb, Tc              │
└────────────────────────────────────────────────────────┘
```

---

## 5. Software-in-the-Loop (SIL) Physics Simulation

To validate and stress-test the control algorithms under realistic non-ideal operating conditions without hardware on the desk, the C++ SIL engine implements:

- **Numerical Integration:** 4th-Order Runge-Kutta (RK4) integration with $1.0\,\mu\text{s}$ sub-stepping to capture high-frequency electrical dynamics:
  $$\frac{di_d}{dt} = \frac{1}{L_d}(V_d - R_s i_d + \omega_e L_q i_q)$$
  $$\frac{di_q}{dt} = \frac{1}{L_q}(V_q - R_s i_q - \omega_e L_d i_d - \omega_e \lambda_{pm})$$
  $$\frac{d\omega_m}{dt} = \frac{1}{J}(T_e - T_L - B \omega_m), \quad \frac{d\theta_m}{dt} = \omega_m$$
- **Inverter Non-Linearity Modeling:** Dead-time voltage drop distortion ($V_{\text{dead}} \approx \frac{T_{\text{dead}}}{T_{\text{pwm}}} V_{dc} \text{sgn}(i)$), DC link voltage sag, and power switch saturation.
- **Sensor Noise & Quantization:**
  - Synthetic 12-bit ADC quantization on phase current measurements with additive Gaussian noise ($\sigma = 15\,\text{mA}$).
  - 14-bit angular quantization ($0.022^\circ$ LSB) with SPI transport latency.

---

## 6. Telemetry & Communications Stack

- **Framing Protocol:** Consistent Overhead Byte Stuffing (**COBS**) for zero-delimiter collision packet synchronization.
- **Integrity Verification:** Hardware/Software **CRC32** (Ethernet polynomial `0xEDB88320`) appended to every packet.
- **Payload Structure (1 kHz Streaming):**
  - Timestamp ($32$-bit microsecond counter)
  - Measured $i_d, i_q$ and Reference $i_d^*, i_q^*$ ($4 \times 32$-bit floats)
  - Rotor electrical angle $\theta_e$ and velocity $\omega_m$ ($2 \times 32$-bit floats)
  - DC bus voltage $V_{bus}$ ($32$-bit float)
  - System status & fault flags ($32$-bit bitfield)
- **Host Tools:**
  - Python 3 with `pyqtgraph` / `matplotlib` for 60 FPS multi-trace oscilloscope rendering.
  - Automated zero-angle offset calibration script ([tools/calibrate_offset.py](../tools/calibrate_offset.py)).

---

## 7. Build, Verification & Quality Standards

- **Build System:** Modern CMake 3.20+ with modular targets and cross-compilation support.
- **Compiler Warning Policy:** `-Wall -Wextra -Wpedantic -Wconversion -Werror` clean across Clang, GCC, and ARM toolchains.
- **Testing Framework:** GoogleTest 1.18.0 integrated via `ctest` with 8 dedicated test suites.
- **Static Analysis & Safety Standards:** MISRA-C aligned safety rules, zero dynamic allocations in real-time execution paths, strict variable scope containment.
