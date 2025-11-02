# acc_brake_steer_pcb
This repository contains the complete embedded firmware for an Electronic Control Unit (ECU) designed to interface with accelerator, brake, and steering wheel sensors in a real vehicle. The primary function is to read sensor data in real-time and transmit it to other vehicle systems via the CAN bus.

System Overview and Sensor Configuration
The system integrates data from three key driver inputs, utilizing redundancy to ensure safety and reliability.

Brake Pedal:
  
- The brake pedal input is processed using three sensors to ensure precision and safety.

- Hall Effect Sensor: Used to detect the initial phase of the pedal press. This allows for the immediate activation of the energy recovery system through regenerative braking.

- 2 x Linear Potentiometers (LPF 25mm): These act as a redundant measurement source, which is crucial for safety. Additionally, the data from both sensors allows for verifying and adjusting the brake bias (front-rear braking force distribution).

Accelerator Pedal:

- 2 x Built-in Rotary Potentiometers: The board reads data from two independent potentiometers integrated into the factory accelerator pedal module. 

Steering Wheel:

- Linear Potentiometer (LPF 175mm): Serves as a redundant sensor for the main digital encoder. It provides precise information about the steering wheel angle, which is then used by the main vehicle computer for advanced functions like torque vectoring.

STM pin config: 
- PA0 - ADC 1 channel 1
- PA1 - ADC 1 channel 2
- PA2 - ADC 1 channel 3
- PA4 - ADC 2 channel 1
- PA5 - ADC 2 channel 2
- PA6 - ADC 2 channel 3
- PA8 - LED
- PA9 - UART_TX
- PA10 - UART_RX
- PA11 - CAN_RX
- PA12 - CAN_TX
- PA_13 - SWDIO
- PA_14 - SWCLK
- PF0/OSC_IN - osc. output

Tech Stack
Microcontroller: STM32 (e.g., L4 or F4 series)

Language: C (Bare-metal)

Communication:

- CAN Bus

- UART

Peripherals:

- ADC (Analog-to-Digital Converter)

- GPIO
