# High-Performance Embedded Field-Oriented Control (FOC) Servo Motor Drive

[![Build & Test Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()
[![Compiler Warnings](https://img.shields.io/badge/warnings-0%20strict-brightgreen.svg)]()
[![Tests Passing](https://img.shields.io/badge/tests-8%2F8%20passed%20(100%25)-brightgreen.svg)]()
[![Current Loop](https://img.shields.io/badge/current%20loop-25%20kHz-blue.svg)]()
[![ISR Latency](https://img.shields.io/badge/ISR%20latency-%3C1.5%20%C2%B5s-blue.svg)]()
[![Language](https://img.shields.io/badge/languages-C11%20%7C%20C%2B%2B20%20%7C%20Python%203-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

A production-grade, bare-metal **Field-Oriented Control (FOC)** motor drive firmware and Software-in-the-Loop (SIL) physics simulator for Permanent Magnet Synchronous Motors (PMSM) and Brushless DC (BLDC) actuators. Built for high-dynamic robotics, electric vehicles, and precision aerospace actuation (benchmarked against Tesla Drivetrain, Boston Dynamics, and Maxon Motor standards).

Features a multi-rate cascaded control architecture running a deterministic **25 kHz inner current loop** with hardware-synchronized peripheral sampling, Space Vector PWM (SVPWM), S-curve trajectory profiling, disturbance torque observation, and a 1 kHz binary telemetry pipeline.

---

## 📚 Technical Documentation Suite

For exhaustive technical deep-dives, mathematical proofs, and interview preparation, refer to the dedicated documentation files:

| Document | Description |
|---|---|
| 📖 **[Interview & Engineering Guide](docs/interview_guide.md)** | **Master Technical Interview Guide:** Complete end-to-end breakdown of motor physics, FOC vector math, SVPWM derivations, hardware valley sampling, digital filter design, anti-windup clamping, and **25+ Tier-1 robotics/firmware interview questions with model answers**. |
| 🏛️ **[System Architecture](docs/architecture.md)** | **System Architecture & Specifications:** Multi-rate timing diagrams, STM32G4 peripheral synchronization, cycle-accurate execution timing budgets, vector transformations, and state machine design. |
| 🛠️ **[Developer Workflow](docs/workflow.md)** | **Developer & Operational Workflow:** Compilation guide for native desktop SIL and ARM cross-compilation, automated CTest harness, Python tuning scope, electrical zero-angle calibration, and hardware flashing. |
| 💻 **[Technology Stack](docs/tech_stack.md)** | **Technology Stack & Tooling:** Detailed catalog of languages (C11, C++20, Python 3), STM32G4 silicon peripherals, gate drivers, magnetic encoders, and numerical physics integration engines. |
| 🗺️ **[Memory Map](docs/memory_map.md)** | **Memory Layout:** STM32G4 Flash, SRAM, DMA circular buffer, and stack layout specifications. |
| 📐 **[SVPWM Derivation](docs/svpwm_derivation.md)** | **Modulation Geometry:** Mathematical derivation of Space Vector PWM sector identification, dwell times, and overmodulation. |

---

## ⚡ Core Features & Technical Highlights

- **Deterministic 25 kHz Inner Current Loop:** Executes in $< 1.5\,\mu\text{s}$ on an ARM Cortex-M4 @ 170 MHz, leaving $> 95\%$ CPU headroom for communication and supervisory tasks.
- **Hardware Valley-Synchronized Current Sampling:** TIM1 center-aligned PWM triggers dual simultaneous ADC conversions at the counter valley (zero vector $V_0$ midpoint) via timer TRGO and circular DMA, completely eliminating switching noise from shunt current measurements.
- **Space Vector PWM (SVPWM):** Increases DC bus voltage utilization by **$+15.47\%$** over standard sinusoidal PWM ($V_{dc}/\sqrt{3}$ vs $V_{dc}/2$) with circular voltage saturation clamping.
- **Cross-Axis Feed-Forward Decoupling:** Dynamically cancels speed voltage cross-coupling terms ($-\omega_e L_q i_q$ and $+\omega_e(L_d i_d + \lambda_{pm})$), ensuring decoupled current dynamics at high speeds.
- **Multi-Rate Cascaded Motion Hierarchy:**
  - **1.0 kHz Position Loop:** P-controller with 7-segment jerk-bounded S-curve trajectory profiling ($J_{\max}, A_{\max}, V_{\max}$).
  - **2.5 kHz Velocity Loop:** PI regulator with acceleration feedforward and Luenberger disturbance torque observer.
  - **25.0 kHz Current Loop:** Decoupled PI regulators with conditional anti-windup clamping.
- **High-Fidelity C++20 SIL Simulator:** 4th-order Runge-Kutta (RK4) integration modeling non-linear motor dynamics, gate-driver dead-time distortion, DC bus sag, and 14-bit encoder quantization.
- **Sub-$1.0\,\mu\text{s}$ Hardware Fault Protection:** Hardware analog comparator wired to `TIM1_BKIN` shuts down PWM outputs asynchronously on overcurrent, complemented by software over/undervoltage and thermal derating.
- **Real-Time Binary Telemetry Suite:** 1 kHz telemetry streaming using Consistent Overhead Byte Stuffing (COBS) and CRC32 verification with an interactive Python oscilloscope ([tools/foc_tuner.py](tools/foc_tuner.py)).
- **Zero Dynamic Memory Allocation:** Strictly 0 bytes allocated on the heap during real-time execution; MISRA-C aligned safety practices.

---

## 🏗️ System Architecture & Multi-Rate Schedule

```text
┌────────────────────────────────────────────────────────┐
│ Outer Position Loop (1 kHz / 1 ms period)              │
│ - P-controller + 7-segment jerk-bounded S-curve ramp   │
└───────────────────────────┬────────────────────────────┘
                            │ Target Velocity ω*
┌───────────────────────────▼────────────────────────────┐
│ Middle Velocity Loop (2.5 kHz / 400 µs period)         │
│ - PI-regulator + Disturbance torque observer           │
└───────────────────────────┬────────────────────────────┘
                            │ Target Torque / Iq*
┌───────────────────────────▼────────────────────────────┐
│ Inner Current Loop (25 kHz / 40 µs period)             │
│ - Valley-sampled Shunt Currents (ia, ib) via DMA       │
│ - Clarke & Park Transforms (θe from 14-bit AS5048A)    │
│ - Decoupled d/q PI regulators + Back-EMF Feed-Forward  │
│ - Circle-limited Vd*, Vq* → SVPWM Dwell Times (Ta,Tb,Tc)│
│ - Timing: < 1.5 µs execution time (>95% CPU headroom)  │
└────────────────────────────────────────────────────────┘
```

---

## 📊 Verification & Test Results

The codebase compiles with zero warnings under strict flags (`-Wall -Wextra -Wpedantic -Wconversion -Werror`) and passes 100% of automated tests across 8 comprehensive suites:

```text
Test project .../build
    Start 1: test_smoke
1/8 Test #1: test_smoke .......................   Passed (0.01 sec)
    Start 2: test_foc_math
2/8 Test #2: test_foc_math ....................   Passed (0.00 sec)
    Start 3: test_pid
3/8 Test #3: test_pid .........................   Passed (0.00 sec)
    Start 4: test_sil_openloop
4/8 Test #4: test_sil_openloop ................   Passed (0.01 sec)
    Start 5: test_sil_closed_loop
5/8 Test #5: test_sil_closed_loop .............   Passed (0.01 sec)
    Start 6: test_motion_profiler
6/8 Test #6: test_motion_profiler .............   Passed (0.00 sec)
    Start 7: test_hal_fault
7/8 Test #7: test_hal_fault ...................   Passed (0.00 sec)
    Start 8: test_packet_protocol
8/8 Test #8: test_packet_protocol .............   Passed (0.00 sec)

100% tests passed out of 8 (Total Test time: 0.04 sec)
```

---

## 🚀 Quick Start Guide

### 1. Build Desktop SIL Simulation & Run Tests
```bash
# Clone the repository
git clone https://github.com/aritraio/embedded-foc-servo-drive.git
cd embedded-foc-servo-drive

# Configure and compile native SIL simulation
cmake -B build -DTARGET_SIL=ON
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu)

# Execute the automated verification suite
ctest --test-dir build --output-on-failure
```

### 2. Launch the SIL Simulator & Real-Time Python Scope
```bash
# Set up Python environment
python3 -m venv .venv
source .venv/bin/activate
pip install -r tools/requirements.txt

# Terminal 1: Run the SIL motor physics simulation
./build/sil_main

# Terminal 2: Run the real-time Python oscilloscope tuner
python3 tools/foc_tuner.py
```

### 3. Cross-Compile for STM32 Bare-Metal Target
```bash
# Cross-compile for STM32G474 ARM Cortex-M4
cmake -B build_stm32 \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DTARGET_STM32=ON

cmake --build build_stm32
```

---

## 🎯 Target Industry & Applications

- **High-Dynamic Legged Robotics & Humanoids:** Boston Dynamics, Tesla Optimus, Unitree, Figure AI.
- **Precision Industrial Actuation:** Moog, Maxon Motor, Harmonic Drive.
- **Electric Vehicle Powertrain Actuators:** Steering rack actuators, active suspension, traction inverters.
- **Aerospace Gimbal & Flight Surface Control:** Satellite reaction wheels, thrust vector actuation.

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
