# acc_brake_steer_pcb
# Main Vehicle ECU (VCU Architecture) Firmware 🏎️⚡

This repository contains the complete embedded firmware for an Electronic Control Unit (ECU) designed for vehicle control and sensor data acquisition.

Originally built to read raw data from accelerator, brake, and steering wheel sensors and transmit it via the CAN bus to external autonomous logic, the firmware has been significantly expanded. The board now features a robust **Finite State Machine (FSM)** and fail-safe logic, allowing it to function as a standalone **Vehicle Control Unit (VCU)**.

Depending on the system state and connectivity, the ECU can either act as a pass-through node for higher-level systems or take direct control of the powertrain—managing inverter states (CANopen NMT), calculating torque requests, and driving the motors independently.

---

## 🛡️ Key Features & Safety Mechanisms

* **Dual-Mode Operation:** Seamlessly switches between a passive Data Acquisition Node (passing inputs to an external Jetson computer) and an active VCU (taking manual control of the inverters when the autonomous system is disconnected).
* **Automotive-Grade Plausibility Checks:** Continuous real-time verification of redundant sensors (e.g., Accelerator ADC1 vs. ADC2). If a hardware discrepancy or broken wire is detected, the system immediately intercepts the torque request and defaults to a safe state.
* **Robust Finite State Machine (FSM):** Centralized logic governing vehicle states (Drive, Neutral, Reverse, Parking). The FSM strictly prioritizes safety, overriding physical gear selector inputs if critical errors are active.
* **Connection Watchdogs:** Software timers monitoring the heartbeat of external subsystems (Jetson logic, PRND gear selector). A connection timeout automatically forces the powertrain into a safe neutral gear.
* **Advanced Error Handler:** Custom CAN error reporting framework that registers errors, prevents scheduler overlaps, and broadcasts standard diagnostic data (with 11-bit ID priority handling) across the vehicle network.
* **CANopen NMT Control:** Direct, standard-compliant manipulation of motor inverters using 2-byte Network Management (NMT) frames (Start, Stop, Pre-Operational).

---

## 🎛️ System Overview and Sensor Configuration

The system integrates data from three key driver inputs, utilizing redundancy to ensure safety and reliability.

### Brake Pedal
The brake pedal input is processed using three sensors to ensure precision and safety:
* **Hall Effect Sensor:** Used to detect the initial phase of the pedal press. This allows for the immediate activation of the energy recovery system through regenerative braking.
* **2 x Linear Potentiometers (LPF 25mm):** These act as a redundant measurement source, which is crucial for safety. Additionally, the data from both sensors allows for verifying and adjusting the brake bias (front-rear braking force distribution).

### Accelerator Pedal
* **2 x Built-in Rotary Potentiometers:** The board reads data from two independent potentiometers integrated into the factory accelerator pedal module.

### Steering Wheel
* **Linear Potentiometer (LPF 175mm):** Serves as a redundant sensor for the main digital encoder. It provides precise information about the steering wheel angle, which is then used by the main vehicle computer for advanced functions like torque vectoring.

---

## 🔌 Hardware Configuration (STM32 Pinout)

| Pin | Function | Peripheral |
| :--- | :--- | :--- |
| `PA0` | ADC 1 Channel 1 | Accelerator / Brake Sensors |
| `PA1` | ADC 1 Channel 2 | Accelerator / Brake Sensors |
| `PA2` | ADC 1 Channel 3 | Accelerator / Brake Sensors |
| `PA4` | ADC 2 Channel 1 | Accelerator / Brake Sensors |
| `PA5` | ADC 2 Channel 2 | Accelerator / Brake Sensors |
| `PA6` | ADC 2 Channel 3 | Accelerator / Brake Sensors |
| `PA8` | Status LED | GPIO Output |
| `PA9` | UART TX | Communication |
| `PA10`| UART RX | Communication |
| `PA11`| CAN RX | Communication |
| `PA12`| CAN TX | Communication |
| `PA13`| SWDIO | Programming / Debugging |
| `PA14`| SWCLK | Programming / Debugging |
| `PF0` | OSC_IN | Oscillator Output |

---

## 💻 Tech Stack

* **Microcontroller:** STM32F303K8T6 (ARM Cortex-M4)
* **Language:** C (Bare-metal / HAL Drivers)
* **Communication Protocols:** CAN Bus (Custom Scheduling & Hardware Filtering), UART
* **Core Peripherals:** ADC (with DMA), Hardware Timers (Interrupts), GPIO

Authors:

hardware and firmware: Michał Jurek
