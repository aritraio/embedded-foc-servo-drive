# Development & Operational Workflow

This guide details the complete development lifecycle for the **Embedded FOC Servo Motor Drive**, including multi-target compilation, Software-in-the-Loop (SIL) simulation, automated verification, live Python tuning, electrical angle calibration, and bare-metal hardware flashing.

---

## 1. Prerequisites & Toolchain Setup

### Host Development Environment (macOS / Linux)
- **C/C++ Compiler:** Clang (AppleClang 15+) or GCC (12+) supporting C11 and C++20.
- **Build System:** CMake >= 3.20 and GNU Make / Ninja.
- **Testing Framework:** GoogleTest (detected automatically via Homebrew `/opt/homebrew` or system package manager).
- **Python Runtime:** Python >= 3.10 with `venv`.

Install host dependencies:
```bash
# macOS (Homebrew)
brew install cmake googletest ninja python@3.11

# Ubuntu / Debian
sudo apt update && sudo apt install -y cmake build-essential libgtest-dev ninja-build python3-pip python3-venv
```

### ARM Cross-Compilation Toolchain (for Bare-Metal Hardware)
To compile for the target STM32G474 / STM32F405 microcontroller:
```bash
# macOS
brew install --cask gcc-arm-embedded
brew install openocd

# Ubuntu / Debian
sudo apt install -y gcc-arm-none-eabi gdb-multiarch openocd
```

---

## 2. Build Workflows

The repository uses a unified CMake build system configured via [CMakeLists.txt](file:///Users/aritra/Code/Languages/C++/Project-2/CMakeLists.txt) supporting dual targets:

### Target A: Desktop Software-in-the-Loop (SIL) Simulation & Tests
Builds the native desktop simulation engine and all unit tests with strict compiler flags (`-Wall -Wextra -Wpedantic -Wconversion -Werror`):
```bash
# Configure for native desktop SIL
cmake -B build -DTARGET_SIL=ON

# Compile all libraries, simulation executables, and test suites
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu)
```

### Target B: Embedded Bare-Metal STM32 Target
Cross-compiles the bare-metal ELF firmware image for ARM Cortex-M4:
```bash
# Configure using the ARM toolchain file
cmake -B build_stm32 \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DTARGET_STM32=ON

# Compile the firmware binary
cmake --build build_stm32 -j$(nproc 2>/dev/null || sysctl -n hw.ncpu)
```
Output artifacts in `build_stm32/`:
- `foc_firmware.elf`: Full debug ELF binary with symbol tables.
- `foc_firmware.bin`: Raw flat binary for flashing via ST-Link / OpenOCD.
- `foc_firmware.hex`: Intel HEX file for production programming.

---

## 3. Automated Testing & Verification Pipeline

The project includes an automated test suite with 100% code verification across 8 dedicated test harnesses:

### Running All Tests via CTest
```bash
ctest --test-dir build --output-on-failure
```
Expected output:
```text
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

100% tests passed out of 8
```

### Running Individual Test Suites
```bash
# Test pure vector math (Clarke, Park, SVPWM 360-degree sector sweeps)
./build/test_foc_math

# Test PID anti-windup clamping and derivative low-pass filtering
./build/test_pid

# Test closed-loop step response in SIL physics simulation
./build/test_sil_closed_loop

# Test 7-segment S-curve jerk-bounded motion profiler
./build/test_motion_profiler

# Test hardware fault supervisor (overcurrent, overvoltage trip <1 µs)
./build/test_hal_fault

# Test COBS packet encoding, decoding, and CRC32 verification
./build/test_packet_protocol
```

---

## 4. Software-in-the-Loop (SIL) & Tuning Workflow

You can run and tune the complete Field-Oriented Control loop directly on your workstation without physical hardware connected:

```text
┌───────────────────────────┐      Virtual Socket / PIPE      ┌───────────────────────────┐
│     sil_main (C++)        │  ◄────────────────────────────► │     foc_tuner.py (Python) │
│ - 4th-Order RK4 Physics   │      COBS + CRC32 Packets       │ - Live Multi-Axis Scope   │
│ - 25 kHz Control ISR      │      (1 kHz State Stream)       │ - Step Response Evaluator │
│ - Dual Shunts & AS5048A   │                                 │ - Gain Parameter Tuning   │
└───────────────────────────┘                                 └───────────────────────────┘
```

### Step 1: Install Python Dependencies
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r tools/requirements.txt
```

### Step 2: Launch the Virtual SIL Motor Simulator
```bash
./build/sil_main
```
The simulator initializes the PMSM plant dynamics, loads the controller state machine, and begins streaming state packets at 1 kHz over a local virtual port.

### Step 3: Launch the Real-Time Python Scope
In a second terminal:
```bash
source .venv/bin/activate
python3 tools/foc_tuner.py
```
The GUI displays real-time live traces of:
- Quadrature Current Response: $I_q^*$ (setpoint) vs $i_q$ (measured).
- Direct Flux Current: $I_d$ (held tightly to $0\,\text{A}$).
- Rotor Electrical Angle ($\theta_e$) and Mechanical Angular Velocity ($\omega_m$).
- DC Link Bus Voltage ($V_{dc}$).

---

## 5. Automated Electrical Angle Calibration Workflow

Before running closed-loop FOC, the physical offset angle ($\theta_{\text{offset}}$) between the magnetic encoder's zero index and the rotor's permanent magnet $d$-axis must be determined:

```bash
python3 tools/calibrate_offset.py
```

### How the Calibration Sequence Works:
1. **D-Axis Alignment Vector:** The firmware commands a stationary voltage vector along the stator $d$-axis ($V_d = V_{\text{align}}, V_q = 0, \theta_e = 0$).
2. **Rotor Lock:** The rotor's magnetic flux locks into alignment with the stator field against any resting friction.
3. **Angle Acquisition:** The AS5048A 14-bit encoder angle is measured over 1,000 samples and averaged.
4. **Offset Storage:** The calibrated value $\theta_{\text{offset}}$ is programmed into non-volatile memory or loaded into [config_params.h](file:///Users/aritra/Code/Languages/C++/Project-2/firmware/app/inc/config_params.h).

---

## 6. Hardware Flashing & Pre-Flight Bring-Up Checklist

When deploying to physical hardware (e.g., STM32G474 Nucleo + DRV8301 Inverter Shield):

### Flashing via OpenOCD / ST-Link
```bash
# Flash the compiled binary using OpenOCD
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
  -c "program build_stm32/foc_firmware.elf verify reset exit"
```

### Pre-Flight Safety Checklist
- [ ] **Current-Limited Power Supply:** Power the DC bus with a benchtop power supply set to nominal motor voltage ($24.0\,\text{V}$) with current limit set to **$1.0\,\text{A}$** for initial bring-up.
- [ ] **Phase Resistance & Inductance Measurement:** Verify phase resistance ($R_s$) and line-to-line inductance ($L$) with an LCR meter match [config_params.h](file:///Users/aritra/Code/Languages/C++/Project-2/firmware/app/inc/config_params.h).
- [ ] **Oscilloscope Dead-Time Check:** Probe high-side and low-side gate signals on Phase A with an oscilloscope. Ensure dead-time is $\ge 100\,\text{ns}$ and no overlap exists.
- [ ] **Encoder Direction Verification:** Rotate the rotor by hand clockwise. Confirm measured mechanical angle $\theta_m$ increments monotonically without parity errors.
- [ ] **Current Sensor Zero-Offset Calibration:** Measure raw ADC counts at rest; verify both Phase A and Phase B shunt amplifiers sit at approximately $V_{ref}/2$ (1.65 V / ~2048 counts).
- [ ] **Open-Loop V/F Spin Test:** Run low-speed open-loop commutation to confirm correct Phase A, B, C wiring order.
