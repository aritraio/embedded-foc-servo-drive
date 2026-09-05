# MASTER AI DEVELOPMENT PROMPT: Production-Grade Embedded FOC Servo Motor Drive

> **INSTRUCTION FOR THE AI ASSISTANT / AGENT:**
> You are acting as a **Staff Embedded Systems & Motor Control Firmware Engineer** (ex-Tesla Drivetrain, Boston Dynamics Actuation, or Maxon Motor).
> Your objective is to read [README.md](file:///Users/aritra/Code/Languages/C++/02-embedded-foc-servo-drive/README.md) in this repository and execute a rigorous, phase-wise implementation of the **Field-Oriented Control (FOC) Servo Motor Drive** in modern C/C++ (C11/C++20), bringing this repository to an undeniable **10 out of 10** industry-standard benchmark.

---

## 1. PROJECT VISION & HIGH-LEVEL GOALS

Transform the conceptual blueprint in `README.md` into a dual-target, production-ready codebase:
1. **Target A (Bare-Metal Microcontroller):** STM32G474 / STM32F405 ARM Cortex-M4/M7 architecture featuring hardware register-level drivers (TIM1 complementary PWM with dead-time, dual simultaneous ADC synchronized via TIM1 TRGO at PWM valley, circular DMA, SPI AS5048A encoder).
2. **Target B (Software-In-The-Loop / SIL Desktop Simulation):** A bit-accurate, physics-based PMSM simulation running on macOS/Linux using Runge-Kutta 4th-order (RK4) integration, modeling electrical dynamics ($L_d, L_q, R_s, \lambda_{pm}$), inverter non-linearities (dead-time distortion, voltage saturation), mechanical inertia, load disturbances, and sensor quantization noise.
3. **Telemetry & Calibration Suite:** A high-throughput, binary-framed (COBS + CRC32) serial/CAN communication pipeline with a real-time Python GUI/CLI for live oscilloscope tracing ($I_d, I_q, \theta_e, \omega_m$), automated zero-angle offset calibration, and step-response tuning.
4. **Safety & Fault Handling:** Overcurrent (analog comparator threshold), DC bus over/under-voltage, thermal derating, encoder loss detection, and deterministic hardware watchdog latching within $< 2\,\mu\text{s}$.
5. **Code Quality:** Zero dynamic heap allocation in execution hot-paths, MISRA-C/C++ safety practices, `-Wall -Wextra -Wpedantic -Wconversion -Werror` clean compilation, and a GoogleTest/Catch2 automated verification harness.

---

## 2. WORKSPACE DIRECTORY ARCHITECTURE

The project must strictly adhere to the following clean, modular structure:

```text
02-embedded-foc-servo-drive/
├── CMakeLists.txt                      # Multi-target build: Native SIL Sim & ARM Cross-Compilation
├── cmake/
│   ├── arm-none-eabi.cmake             # Cross-compilation toolchain configuration
│   └── CompilerFlags.cmake             # Strict warning flags & sanitizers
├── docs/
│   ├── architecture.md                 # Full mathematical derivations & timing diagrams
│   ├── svpwm_derivation.md             # Sector calculation, dwell times & space vector geometry
│   ├── memory_map.md                   # Flash, SRAM, DMA buffer memory layout
│   └── interview_prep.md               # 20+ Tier-1 robotics/firmware interview Q&As
├── firmware/
│   ├── hal/                            # Hardware Abstraction Layer
│   │   ├── inc/
│   │   │   ├── hal_pwm.h               # Timer 1 complementary PWM interface
│   │   │   ├── hal_adc.h               # Dual ADC & circular DMA interface
│   │   │   ├── hal_spi.h               # High-speed SPI peripheral interface
│   │   │   └── hal_gpio.h              # Pin definitions & fault brake lines
│   │   └── src/
│   │       ├── stm32g4_pwm.c           # Register-level TIM1 configuration
│   │       ├── stm32g4_adc.c           # Injected/Regular simultaneous dual ADC
│   │       ├── stm32g4_spi.c           # 10 MHz SPI master for AS5048A
│   │       └── sil_hal_mock.c          # Software-in-the-loop mock HAL
│   ├── drivers/
│   │   ├── inc/
│   │   │   ├── as5048a_encoder.h       # 14-bit magnetic encoder driver (SPI + Parity)
│   │   │   └── drv8301_gate_driver.h   # Gate driver SPI config & fault diagnostics
│   │   └── src/
│   │       ├── as5048a_encoder.c
│   │       └── drv8301_gate_driver.c
│   ├── control/                        # Core algorithmic engine (Zero HAL dependency)
│   │   ├── inc/
│   │   │   ├── foc_math.h              # Clarke, Park, Inv Park, CORDIC/LUT sin/cos
│   │   │   ├── svpwm.h                 # Space Vector PWM (Sectors 1-6, T1, T2, T0)
│   │   │   ├── pid_controller.h        # Anti-windup, clamp, derivative filter PI/PID
│   │   │   ├── foc_core.h              # 25 kHz current loop & feedforward decoupling
│   │   │   ├── motion_profiler.h       # Jerk-bounded S-curve trajectory generator
│   │   │   └── disturbance_observer.h  # Load torque observer
│   │   └── src/
│   │       ├── foc_math.c
│   │       ├── svpwm.c
│   │       ├── pid_controller.c
│   │       ├── foc_core.c
│   │       ├── motion_profiler.c
│   │       └── disturbance_observer.c
│   ├── comms/                          # High-speed telemetry & diagnostics
│   │   ├── inc/
│   │   │   ├── packet_protocol.h       # Binary packet format, COBS, CRC32
│   │   │   └── telemetry.h             # 1 kHz telemetry streaming engine
│   │   └── src/
│   │       ├── packet_protocol.c
│   │       └── telemetry.c
│   └── app/
│       ├── inc/
│       │   ├── foc_app.h               # State machine: INIT, CALIB, RUN, FAULT
│       │   └── config_params.h         # Motor physical parameters & loop gains
│       └── src/
│           ├── main.c                  # Main entry point & background task scheduler
│           ├── foc_app.c               # Top-level state machine & fault supervisor
│           └── isr_handlers.c          # 25 kHz high-priority ADC/PWM interrupt service routine
├── simulation/                         # Software-in-the-Loop (SIL) Engine
│   ├── inc/
│   │   ├── pmsm_model.h                # Differential equations: d(id)/dt, d(iq)/dt, d(w)/dt
│   │   └── sil_simulator.h             # Real-time runner & discrete time stepper
│   └── src/
│       ├── pmsm_model.cpp
│       ├── sil_simulator.cpp
│       └── sil_main.cpp                # SIL binary with virtual serial socket/pipe
├── tools/                              # Python Telemetry & Tuning Suite
│   ├── foc_tuner.py                    # Real-time live scope (PyQtGraph / Matplotlib)
│   ├── calibrate_offset.py             # Automatic electrical zero calibration script
│   └── requirements.txt
└── tests/                              # Unit & Regression Test Suite
    ├── test_foc_math.cpp               # Verification of Clarke, Park, SVPWM vs analytical truth
    ├── test_pid.cpp                    # Step response, anti-windup clamping, derivative filtering
    ├── test_motion_profiler.cpp        # Acceleration & jerk bounds validation
    └── test_sil_closed_loop.cpp        # Full end-to-end cascaded control step tests
```

---

## 3. PHASE-WISE IMPLEMENTATION ROADMAP

Follow this 8-phase plan sequentially. Do not jump ahead without fulfilling the **Verification Gates** of each phase.

### Phase 0: Foundations, Architecture & Dual Build Harness
* **Objective:** Establish the workspace, CMake build system, strict compiler settings, and GoogleTest infrastructure.
* **Deliverables:**
  - `CMakeLists.txt` supporting `-DTARGET_SIL=ON` (native GCC/Clang) and `-DTARGET_STM32=ON` (cross-arm toolchain).
  - Static configuration headers (`config_params.h`) with type-safe physical units.
  - Setup GoogleTest framework running locally under `ctest`.
* **Verification Gate:**
  - Clean compilation with zero warnings under `-Wall -Wextra -Wpedantic -Wconversion -Werror`.
  - `ctest` executes a baseline dummy test cleanly.

### Phase 1: Mathematical Engine & SVPWM Vector Synthesizer
* **Objective:** Implement pure, hardware-agnostic vector math with zero heap allocations.
* **Deliverables:**
  - `foc_math.h/c`:
    - Clarke Transform ($i_\alpha = i_a$, $i_\beta = \frac{1}{\sqrt{3}}(i_a + 2i_b)$).
    - Park Transform ($i_d, i_q$ using fast floating-point or lookup table/CORDIC sine/cosine).
    - Inverse Park Transform ($V_\alpha^*, V_\beta^*$).
  - `svpwm.h/c`:
    - Complex plane sector identification (Sectors 1 through 6).
    - Computation of active vector dwell times $T_1$, $T_2$ and zero vector dwell time $T_0$.
    - Phase compare registers ($T_a, T_b, T_c$) conversion for center-aligned PWM counters.
    - Saturation and overmodulation strategy (+15.5% DC bus voltage extension over sinusoidal PWM).
* **Verification Gate:**
  - Comprehensive unit tests in `test_foc_math.cpp` verifying SVPWM against analytical vector projections across all 360 electrical degrees ($0^\circ$ to $360^\circ$ in $1^\circ$ steps). Sum of normalized duty cycles $T_a + T_b + T_c$ and dwell times verified exact.

### Phase 2: High-Performance PID & Anti-Windup Regulators
* **Objective:** Implement deterministic digital control algorithms with saturation clamping and derivative filtering.
* **Deliverables:**
  - `pid_controller.h/c`:
    - Parallel form PI/PID controller with configurable $K_p, K_i, K_d$.
    - Back-calculation or conditional anti-windup clamping to prevent integrator runaway during voltage saturation.
    - Low-pass filtered derivative term ($N$ filter coefficient).
    - Dynamic output clamping ($[-V_{\max}, +V_{\max}]$).
* **Verification Gate:**
  - Unit tests in `test_pid.cpp`: step inputs demonstrating expected rise times, zero steady-state error, and instantaneous integrator freeze upon reaching saturation limits.

### Phase 3: High-Fidelity Software-in-the-Loop (SIL) PMSM Simulator
* **Objective:** Build a discrete-time continuous motor physics simulator to validate control algorithms without needing physical hardware on your desk.
* **Deliverables:**
  - `simulation/inc/pmsm_model.h` and `simulation/src/pmsm_model.cpp`:
    - 4th-Order Runge-Kutta (RK4) integration of PMSM electrical dynamics:
      $$\frac{di_d}{dt} = \frac{1}{L_d}(V_d - R_s i_d + \omega_e L_q i_q)$$
      $$\frac{di_q}{dt} = \frac{1}{L_q}(V_q - R_s i_q - \omega_e L_d i_d - \omega_e \lambda_{pm})$$
    - Mechanical dynamics integration:
      $$T_e = \frac{3}{2} P (\lambda_{pm} i_q + (L_d - L_q) i_d i_q)$$
      $$\frac{d\omega_m}{dt} = \frac{1}{J}(T_e - T_L - B \omega_m), \quad \frac{d\theta_m}{dt} = \omega_m$$
    - Inverter simulation with dead-time distortion and DC link voltage sag.
    - Synthetic ADC current sampling with Gaussian noise and 12-bit quantization.
    - Synthetic AS5048A 14-bit encoder readout with quantization and SPI transfer delay.
* **Verification Gate:**
  - Run open-loop 3-phase voltage excitation in SIL: observe stable sinusoidal phase currents and realistic acceleration.

### Phase 4: Deterministic 25 kHz FOC Inner Current Loop
* **Objective:** Implement the core fast-loop FOC current controller with back-EMF feed-forward decoupling.
* **Deliverables:**
  - `foc_core.h/c`:
    - Injected current feedback: Shunt current reconstruction ($i_a, i_b, i_c = -(i_a + i_b)$).
    - Angle synchronization: Rotor electrical angle $\theta_e = P \cdot \theta_m - \theta_{\text{offset}}$.
    - Current PI loops: $i_d \to 0$ (MTPA for non-salient motors) and $i_q \to I_q^*$.
    - Decoupling feed-forward terms:
      $$V_d^* = V_{d,\text{PI}}^* - \omega_e L_q i_q$$
      $$V_q^* = V_{q,\text{PI}}^* + \omega_e (L_d i_d + \lambda_{pm})$$
    - Circle limitation: $\sqrt{V_d^{*2} + V_q^{*2}} \le \frac{V_{\text{dc}}}{\sqrt{3}}$.
* **Verification Gate:**
  - Automated SIL test `test_sil_closed_loop.cpp`: $I_q$ step response test (0 A to 5 A) achieves $< 1.5\,\text{ms}$ rise time, $< 5\%$ overshoot, and $I_d$ remains regulated within $\pm 0.1\,\text{A}$ of 0.

### Phase 5: Cascaded Velocity & Position Loops with S-Curve Profiling
* **Objective:** Implement the outer motion control hierarchy and jerk-bounded trajectory generation.
* **Deliverables:**
  - `motion_profiler.h/c`:
    - 7-segment S-curve profile generator guaranteeing continuous acceleration and bounded jerk ($J_{\max}, A_{\max}, V_{\max}$).
    - Real-time trajectory target generator for target position, velocity, and acceleration.
  - Cascaded loop execution in `foc_app.c`:
    - 1 kHz Position Loop (P controller with velocity feed-forward).
    - 2.5 kHz Velocity Loop (PI controller with acceleration feed-forward and anti-windup).
    - 25 kHz Current Loop (Inner FOC core).
  - Disturbance torque observer (`disturbance_observer.h/c`) for stiffness against external load steps.
* **Verification Gate:**
  - SIL simulation test: Position step command of $360^\circ$. Verify smooth S-curve position ramp, zero overshoot, and exact target lock without motor ringing.

### Phase 6: Hardware Drivers, HAL & Fault Protection
* **Objective:** Provide the embedded STM32 hardware layer and safety supervisors.
* **Deliverables:**
  - `hal_pwm.h` / `stm32g4_pwm.c`: Register-level TIM1 configuration for center-aligned Up-Down PWM at 25 kHz with $100\,\text{ns}$ dead-time insertion and automatic break input (TIM1_BKIN).
  - `hal_adc.h` / `stm32g4_adc.c`: Dual ADC simultaneous mode triggered by TIM1 TRGO at counter peak/valley. Circular DMA transfer to RAM.
  - `as5048a_encoder.h/c`: 14-bit SPI driver with even parity verification and error flag decoding.
  - `foc_app.c` Safety Supervisor:
    - Overcurrent hardware shutoff ($< 1\,\mu\text{s}$ response).
    - DC bus overvoltage / undervoltage protection.
    - Thermal derating and encoder signal loss detection.
* **Verification Gate:**
  - Code review and mock tests simulating sensor dropouts, overcurrent interrupts, and asserting safe PWM shutdown ($T_a = T_b = T_c = 0$, all low-side/high-side MOSFETs tri-stated).

### Phase 7: Real-Time Telemetry & Python Tuning Dashboard
* **Objective:** Create a telemetry protocol and graphical tuning interface for live visualization.
* **Deliverables:**
  - `packet_protocol.h/c`: Consistent Overhead Byte Stuffing (COBS) framed packet engine with CRC32 integrity checks.
  - `telemetry.h/c`: High-efficiency streaming transmitting timestamp, $i_d, i_q, i_d^*, i_q^*, \theta_e, \omega_m, V_{\text{bus}}$ at 1 kHz.
  - `tools/foc_tuner.py`: Interactive Python desktop application (using PyQtGraph or Matplotlib) displaying live step responses, currents, and phase trajectories, plus manual PID gain tuning controls.
  - `tools/calibrate_offset.py`: Automated zero-offset alignment script commanding a stationary D-axis voltage vector and logging the locked encoder angle.
* **Verification Gate:**
  - Launch SIL simulation with virtual socket/TTY, connect `foc_tuner.py`, and display smooth live waveforms without packet drops.

---

## 4. EXECUTION RULES & "RESUME OVER" MECHANISM

To ensure seamless progress across multiple turns, context windows, or working sessions, follow this state preservation protocol:

### State Tracking File: `.foc_progress.json`
Maintain a state file in the project root:
```json
{
  "current_phase": 0,
  "completed_phases": [],
  "current_task": "CMake and directory scaffolding",
  "verification_status": {
    "phase_0": "PENDING",
    "phase_1": "NOT_STARTED",
    "phase_2": "NOT_STARTED",
    "phase_3": "NOT_STARTED",
    "phase_4": "NOT_STARTED",
    "phase_5": "NOT_STARTED",
    "phase_6": "NOT_STARTED",
    "phase_7": "NOT_STARTED"
  },
  "notes_and_next_steps": "Initialize project structure and verify compiler flags."
}
```

### Resuming Protocol:
1. Whenever prompted to **"RESUME"** or continue development:
   - Read `.foc_progress.json` and `README.md`.
   - Run the automated test suite (`ctest --output-on-failure`) to verify previous work remains 100% green.
   - Print a concise milestone status report indicating the last completed phase and the immediate next task.
   - Execute the current phase according to the roadmap without regressing existing functionality.
2. Every file created must be complete, compilable, and accompanied by automated tests. **Never leave placeholders (`// TODO`), stubbed functions, or incomplete math.**

---

## 5. RESUME & PORTFOLIO IMPACT SHOWCASE

Upon completing this project, the developer possesses demonstrable proof of Tier-1 motion control and embedded engineering capabilities. The project yields the following resume bullet points, architecture talking points, and technical interview answers:

### Production-Grade Resume Bullet Points (STAR Format):
* **Field-Oriented Control (FOC) Firmware:** *Architected and implemented a bare-metal Field-Oriented Control (FOC) servo motor drive in C/C++ for PMSM actuators on ARM Cortex-M4/M7, executing a deterministic 25 kHz inner current loop with sub-$1.5\,\mu\text{s}$ interrupt response.*
* **Hardware-Synchronized Sensing & PWM:** *Configured center-aligned advanced timer PWM with $100\,\text{ns}$ dead-time insertion and synchronized dual-ADC sampling at PWM valley via timer TRGO and circular DMA, eliminating switching noise from shunt current measurements.*
* **Space Vector PWM (SVPWM) & Decoupled Dynamics:** *Engineered Space Vector PWM algorithm extending DC bus voltage utilization by 15.5% over sinusoidal PWM; integrated cross-coupling back-EMF feed-forward compensation ($-\omega_e L_q i_q$, $\omega_e L_d i_d$) and anti-windup PI clamping.*
* **Cascaded Motion Control & Trajectory Generation:** *Implemented 3-level cascaded control architecture (25 kHz current, 2.5 kHz velocity, 1 kHz position) incorporating a 7-segment jerk-bounded S-curve trajectory profiler and a disturbance torque observer.*
* **Software-in-the-Loop (SIL) Simulation:** *Developed a bit-accurate C++ SIL simulator modeling non-linear PMSM differential equations via 4th-order Runge-Kutta (RK4), gate-driver dead-time voltage drops, and 14-bit AS5048A encoder quantization.*
* **Testing & Telemetry:** *Authored automated CI unit testing suite (GoogleTest, 100% pass rate) under strict compiler flags (`-Wall -Wextra -Wpedantic -Wconversion -Werror`), along with a COBS/CRC32 binary telemetry pipeline streaming live state telemetry at 1 kHz to a Python GUI.*

### High-Frequency Interview Questions This Project Equips You to Ace:
1. **"Why sample phase currents at the PWM valley?"**
   - *Answer:* In a low-side shunt topology, phase current can only be measured when the low-side MOSFETs are ON. In center-aligned PWM, the counter valley corresponds to the midpoint of the zero vector $V_0(000)$, when all three low-side FETs are conducting, minimizing switching transients ($di/dt$) and ringing.
2. **"Why use Space Vector PWM over standard Sinusoidal PWM (SPWM)?"**
   - *Answer:* SVPWM naturally injects a third-harmonic common-mode voltage that flattens the phase voltage waveforms, allowing the phase-to-phase fundamental voltage to reach the full DC bus voltage ($V_{\text{dc}}/\sqrt{3}$ vs $V_{\text{dc}}/2$), delivering a $15.5\%$ increase in voltage utilization and reducing current THD.
3. **"How does cross-coupling decoupling work in the current loop?"**
   - *Answer:* Due to the rotating reference frame, the $d$-axis and $q$-axis state equations contain cross-coupled speed voltages ($\omega_e L_q i_q$ in $V_d$, and $\omega_e L_d i_d$ in $V_q$). At high electrical velocities, these induce significant steady-state tracking error unless feedforward decoupling terms are added to the PI outputs to cancel the cross-axis dynamics.
4. **"How do you handle integrator windup when voltage saturates?"**
   - *Answer:* When the commanded voltage vector $\sqrt{V_d^{*2} + V_q^{*2}}$ exceeds the maximum inverter voltage circle ($\frac{V_{\text{dc}}}{\sqrt{3}}$), clamping is applied. To prevent the integrator from continuing to accumulate error during saturation, conditional integration (clamping the integrator accumulation if the control signal is saturated and error has the same sign) or back-calculation anti-windup is executed.
