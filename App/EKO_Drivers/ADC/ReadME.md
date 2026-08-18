# STM32 Peripheral ADC Driver

## Title

**STM32 Peripheral ADC Driver (TDD Implementation)**

## Description

This directory contains source and header files for the ADC driver, designed to optimize the workflow with Analog-to-Digital Converters in the **Perła** racing car projects.

The main purpose of this ADC driver is to provide a clear API and macros that simplify the ADC workflow while ensuring firmware universality across various STM32 cores (for now, it supports F1 and F3 cores - in future implementation will support wider range of MCUs). The modular structure facilitates locating, analyzing, and debugging software errors at the low-level layer.

> **NOTE:** The driver is built using direct register access and low-level features of the ADC controller embedded in STM32 cores.

---

## Status & Development (TDD)

The project is being developed using the **TDD (Test-Driven Development)** methodology. This ensures high reliability through a suite of diagnostic functions that verify the driver's correctness within the dedicated hardware solution.

- **Current Support:** At this stage of implementation, the driver supports **multi-channel readout** with **DMA** support while ADC mode is set to **independent**.
- **Documentation:** The user manual will be provided and updated continuously with each new functional update.

---

## Project Structure

The project architecture is divided into modules to ensure a transparent flow and separation of logic:

### 📂 Inc (Headers)

- **adc_driver.h**: API function prototypes, macro definitions, and data structures supporting a logical workflow.
- **adc_utils.h**: Declarations of low-level utilities operating directly on ADC registers.
- **adc_conf.h**: Used for parameters that require manual user configuration.

### 📂 Src (Sources)

- **adc_driver.c**: Implementation of core API functions and driver variable definitions.
- **adc_utils.c**: Operational functions extracting and processing data directly from the ADC controller registers.

### 📂 Tests (Quality Assurance)

- **adc_ut.c**: Unit testing modules. Currently, these contain on-target **static tests**. In the final phase of implementation, they will be expanded into **diagnostic functions** to confirm on-target operational correctness.

---

## Application

### Mode I: Manual/Non-DMA (Discontinuous Mode)

Use this mode when you want full software control over each conversion.

- **Discontinuous Conversion Mode**: Set to `Enabled`.
- **Number of Discontinuous Conversions**: Set to `1`.
- **Scan Conversion Mode**: Must be `Enabled`.
- **Number of Conversions**: Must be equal to the number of active channels.
- **Rank Configuration**: Each rank must be assigned to a **different channel**.

> [!INFORMATION]
> While using only one channel, there is no need to set **Discontinuous** flag, moreover \*_STM32CubeMx_ would not allow such a configuration.
> Driver automatically detects number of channels and effectively switch its security checking logic.

### Mode II: Automatic/DMA (Continuous Mode)

Use this mode for high-speed, background data collection without CPU intervention.

- **Continuous Conversion Mode**: Set to `Enabled`.
- **Discontinuous Conversion Mode**: Set to `Disabled`.
- **DMA Continuous Request**: Set to `Enabled`.
- **DMA Settings**:
  1. Go to the **DMA Settings** tab (or GPDMA for H5 series).
  2. Add a new DMA request for the specific ADC instance.
  3. Configure the DMA mode as `Circular` to ensure continuous data flow into the memory buffer.
  4. Ensure **Data Width** (Half Word/Word) matches your ADC resolution.
- **Scan Conversion Mode**: Must be `Enabled`.
- **Number of Conversions**: Must be equal to the number of active channels.

> [!CAUTION]
> **ADC Clock & DMA Interrupt Overload**
> Pay close attention to the **ADC1/ADC2 clock configuration** in the `.ioc` file.
> Setting the clock frequency too high while using Continuous Mode can trigger
> DMA interrupts so frequently that the CPU spends all its cycles in the ISR.
> As a consequence, the main application thread or API functions **may become permanently starved (blocked)**.

## Manual

_The detailed manual will be expanded as development progresses._

1.  For a proper workflow, initialize the `ADC_ChannelsConfigTypeDef` object for each ADC instance.

```c
ADC_ChannelsConfigTypeDefs cadc1;         //< Channels struct object - stores rank configurations for ADC1
ADC_ChannelsConfigTypeDefs cadc2;         //< Channels struct object - stores rank configurations for ADC2
```

> [!CAUTION]
> While working with **DMA** (automatic mode), please initialize **ADC_BufferTypeDef badc**. It's common for ADCs. There is \*_no need_ to initialize it for each **ADCs**.

```c
ADC_BufferTypeDef badc;                   //< DMA buffer for ADC1 and ADC2
```

2.  Call the `ADC_Init` function for all ADCs you are working with.

```c
/*
 * @brief  ADC initialization function, initialize type of conversion, set correct ADC's regs' values
 * @param  hadc - handle to ADC instance
 * @retval status of HAL's operation
 */
HAL_StatusTypeDef ADC_Init(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, ADC_BufferTypeDef* badc);
```

3.  In **adc_conf.h** change `ADCx_USED_CHANNELS` stored number, to amount of selected channels in **.ioc file**.
    > **NOTE:** In `ADCx_USED_CHANNELS`, **x** stand for index of ADC instance - can be `ADC1_..` or `ADC2_...`.

```c
#define ADC1_USED_CHANNELS (3)  //< Macro defines number of channels for ADC1,
#define ADC2_USED_CHANNELS (3)  //< Macro defines number of channels for ADC2,
```

4.  In **adc_conf.h** change `ADCx_SAMPLING` stored number, to amount of channel's samples You want to be averaged - **must be even**
    > **NOTE:** In `ADCx_SAMPLING`, **x** stand for index of ADC instance - can be `ADC1_..` or `ADC2_...`.

```c
#define ADC1_SAMPLING      (4)  //< Macro defines number of measures of one channel to be averaged
#define ADC2_SAMPLING      (4)  //< Macro defines number of measures of one channel to be averaged
```

---

## Author

**Bartosz Rychlicki** [bartoszrychl@student.agh.edu.pl](mailto:bartoszrychl@student.agh.edu.pl)

**AGH Eko-Energy** Software Department
