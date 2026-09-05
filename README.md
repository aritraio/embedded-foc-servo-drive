# High-Performance Field-Oriented Control (FOC) Servo Motor Drive on Bare-Metal Embedded

## Project Overview
A production-grade, bare-metal **Field-Oriented Control (FOC)** motor firmware for Permanent Magnet Synchronous Motors (PMSM) or Brushless DC (BLDC) motors with magnetic angle feedback. Features cascaded current, velocity, and position control loops with **Space Vector Pulse Width Modulation (SVPWM)**, running the innermost loop deterministically at 20–40 kHz via hardware timer interrupts and Direct Memory Access (DMA).

---

## Target Industry & Value Proposition
* **Target Industries:** Robotics (Legged robots, Humanoids, Robotic Arms), Electric Vehicles, Aerospace Actuation, Precision Gimbal Systems (e.g., Boston Dynamics, Tesla Optimus, Unitree, Moog, Maxon).
* **Resume Impact:** Demonstrates deep low-level hardware mastery (ARM Cortex-M architecture, register configuration, ADC/DMA synchronization) coupled with continuous-domain control math and digital filtering.

---

## Mathematical Foundations & Control Formulation

### 1. Vector Transformation Pipeline

* **Clarke Transformation:**
  Converts balanced three-phase currents ($i_a, i_b, i_c$ where $i_a + i_b + i_c = 0$) into an orthogonal stationary reference frame $(\alpha, \beta)$:
  $$i_\alpha = i_a$$
  $$i_\beta = \frac{1}{\sqrt{3}}(i_a + 2i_b) = \frac{\sqrt{3}}{3}(i_b - i_c)$$

* **Park Transformation:**
  Rotates the stationary frame $(\alpha, \beta)$ into the rotor synchronous reference frame $(d, q)$ using electrical angle $\theta_e$:
  $$i_d = i_\alpha \cos\theta_e + i_\beta \sin\theta_e$$
  $$i_q = -i_\alpha \sin\theta_e + i_\beta \cos\theta_e$$
  where $i_d$ controls rotor flux (commanded to $0$ for Maximum Torque Per Ampere / MTPA below base speed), and $i_q$ produces electromagnetic torque.

* **Inverse Park Transformation:**
  Transforms desired voltages $(V_d^*, V_q^*)$ back to the stationary frame $(V_\alpha^*, V_\beta^*)$:
  $$V_\alpha^* = V_d^* \cos\theta_e - V_q^* \sin\theta_e$$
  $$V_\beta^* = V_d^* \sin\theta_e + V_q^* \cos\theta_e$$

### 2. Space Vector Pulse Width Modulation (SVPWM)
* Synthesizes the target stator voltage vector $\mathbf{V}^* = V_\alpha^* + jV_\beta^*$ using 6 active voltage vectors and 2 zero vectors.
* Determines the active sector (1 through 6) in the complex plane based on $\angle \mathbf{V}^*$.
* Computes active state dwell times $T_1$ and $T_2$ and zero vector dwell time $T_0$:
  $$T_1 = \frac{\sqrt{3} T_{\text{pwm}}}{V_{\text{dc}}} \left( \sin\left(\frac{k\pi}{3}\right)V_\alpha^* - \cos\left(\frac{k\pi}{3}\right)V_\beta^* \right)$$
  $$T_2 = \frac{\sqrt{3} T_{\text{pwm}}}{V_{\text{dc}}} \left( -\sin\left(\frac{(k-1)\pi}{3}\right)V_\alpha^* + \cos\left(\frac{(k-1)\pi}{3}\right)V_\beta^* \right)$$
  $$T_0 = T_{\text{pwm}} - T_1 - T_2$$
* Yields $\approx 15.5\%$ higher DC bus voltage utilization than sinusoidal PWM with reduced current total harmonic distortion (THD).

### 3. Cascaded Closed-Loop Control Architecture
* **Inner Current Loop (25 kHz):**
  Decoupled PI regulators with feed-forward back-EMF compensation:
  $$V_d^* = K_{p,d}(i_d^* - i_d) + K_{i,d}\int(i_d^* - i_d)dt - \omega_e L_q i_q$$
  $$V_q^* = K_{p,q}(i_q^* - i_q) + K_{i,q}\int(i_q^* - i_q)dt + \omega_e (L_d i_d + \lambda_{pm})$$
* **Middle Velocity Loop (2.5 kHz):**
  PI controller with anti-windup clamp and acceleration feed-forward.
* **Outer Position Loop (1 kHz):**
  P-gain with real-time jerk-bounded S-curve trajectory profiling.

---

## System & Software Architecture

### Directory Structure Blueprint
```text
embedded-foc-drive/
├── Makefile / CMakeLists.txt
├── linker_script.ld
├── Drivers/
│   ├── CMSIS/
│   └── HAL_LL/ (Low-Layer drivers)
├── Core/
│   ├── Inc/
│   │   ├── foc_math.h
│   │   ├── foc_core.h
│   │   ├── svpwm.h
│   │   ├── pid_controller.h
│   │   ├── encoder_as5048.h
│   │   └── hw_config.h
│   └── Src/
│       ├── main.c
│       ├── foc_math.c
│       ├── foc_core.c
│       ├── svpwm.c
│       ├── pid_controller.c
│       └── stm32g4xx_it.c (Interrupt service routines)
├── Python_Tools/
│   ├── motor_tuner.py
│   └── telemetry_scope.py
└── Docs/
    ├── hardware_schematic.pdf
    └── memory_map.md
```

### Hardware & Embedded Peripherals Configuration
* **Microcontroller:** STM32G474 (ARM Cortex-M4 @ 170 MHz, hardware CORDIC math coprocessor, high-resolution timer HRTIM) or STM32F405 / ESP32.
* **PWM Generation:** TIM1 configured in center-aligned mode (Up-Down counting) at 25 kHz.
* **ADC Synchronization:** TIM1 Update/TRGO event triggers dual ADC regular simultaneous sampling at PWM valley to sample currents when low-side MOSFETs are fully conducting.
* **Direct Memory Access (DMA):** Circular DMA transfers ADC phase current readings directly to memory with zero CPU overhead.
* **Encoder Interface:** High-speed SPI at 10 MHz reading AS5048A 14-bit magnetic absolute encoder in $< 2\,\mu\text{s}$.

---

## Implementation Roadmap

### Phase 1: Low-Level Timers & Inverter Bring-Up
* Configure center-aligned PWM with hardware dead-time (e.g., $100\,\text{ns}$) to prevent shoot-through.
* Verify complementary PWM switching on a 2-channel oscilloscope.
* Set up ADC sampling triggered by timer TRGO and configure circular DMA.

### Phase 2: Sensor Bring-Up & Open-Loop Commutation
* Implement SPI driver for AS5048A angle reading; verify 14-bit resolution and parity check.
* Implement Open-Loop V/F control to rotate the stator field at constant velocity.
* Calibrate electrical zero angle offset $\theta_{\text{offset}}$ between rotor d-axis and encoder index.

### Phase 3: Closed-Loop Current FOC
* Implement Clarke, Park, and Inverse Park transforms using hardware CORDIC / ARM DSP intrinsics (`arm_sin_f32`, `arm_cos_f32`).
* Implement SVPWM duty cycle generation.
* Close the 25 kHz current loop ($I_d = 0$, $I_q$ tracking square wave); measure step response on current probe.

### Phase 4: Velocity/Position Cascades & Telemetry
* Add 2.5 kHz velocity loop and 1 kHz position loop with S-curve profiler.
* Build serial/CAN telemetry streaming currents, velocities, and tracking error to a live Python dashboard.

---

## Resume Talking Points
* *Developed bare-metal Field-Oriented Control (FOC) firmware in C for 3-phase PMSM motors on STM32 ARM Cortex-M4, achieving a deterministic 25 kHz current control loop.*
* *Configured advanced timer center-aligned PWM with synchronized dual-ADC DMA conversions and hardware dead-time insertion, ensuring sub-$1.5\,\mu\text{s}$ interrupt response time.*
* *Implemented Space Vector PWM (SVPWM) and anti-windup PI current decoupling, boosting DC bus voltage efficiency by 15.5% while mitigating torque ripple.*
