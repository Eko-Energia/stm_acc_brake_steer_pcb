
/**
  ******************************************************************************
  * @file    stm32_family.h
  * @author  Bartosz Rychlicki
  * @Title   Universal driver for ADC peripheral
  * @brief   This file contains common defines of stm32 families. Code defines family whose member is detected.
  * 		 In case developer uses stm32f103rb, then code defines family of stm32f1_family.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 AGH Eko-Energy.
  * All rights reserved.
  *
  ******************************************************************************
  */


#ifndef __STM32_FAMILY_H
#define __STM32_FAMILY_H

/* ----------------------------- F0 SERIES ----------------------------- */
#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F031x6) || defined(STM32F038xx) || \
    defined(STM32F042x6) || defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
    defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
  #define STM32F0_FAMILY
#endif

/* ----------------------------- F1 SERIES ----------------------------- */
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101x6) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102x6) || defined(STM32F102xB) || \
    defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
  #define STM32F1_FAMILY
#endif

/* ----------------------------- F2 SERIES ----------------------------- */
#if defined(STM32F205xx) || defined(STM32F215xx) || defined(STM32F207xx) || defined(STM32F217xx)
  #define STM32F2_FAMILY
#endif

/* ----------------------------- F3 SERIES ----------------------------- */
#if defined(STM32F301x8) || defined(STM32F318xx) || defined(STM32F302x8) || defined(STM32F302xC) || \
    defined(STM32F302xE) || defined(STM32F303x8) || defined(STM32F303xC) || defined(STM32F303xE) || \
    defined(STM32F328xx) || defined(STM32F334x8) || defined(STM32F358xx) || defined(STM32F373xC) || \
    defined(STM32F378xx) || defined(STM32F398xx)
  #define STM32F3_FAMILY
#endif

/* ----------------------------- F4 SERIES ----------------------------- */
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || defined(STM32F407xx) || \
    defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F410Tx) || defined(STM32F411xE) || \
    defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
    defined(STM32F413xx) || defined(STM32F423xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
    defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
  #define STM32F4_FAMILY
#endif

/* ----------------------------- F7 SERIES ----------------------------- */
#if defined(STM32F722xx) || defined(STM32F723xx) || defined(STM32F730xx) || defined(STM32F732xx) || \
    defined(STM32F733xx) || defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F756xx) || \
    defined(STM32F765xx) || defined(STM32F767xx) || defined(STM32F769xx) || defined(STM32F777xx) || \
    defined(STM32F779xx)
  #define STM32F7_FAMILY
#endif

/* ----------------------------- G0 SERIES ----------------------------- */
#if defined(STM32G030xx) || defined(STM32G031xx) || defined(STM32G041xx) || \
    defined(STM32G070xx) || defined(STM32G071xx) || defined(STM32G081xx)
  #define STM32G0_FAMILY
#endif

/* ----------------------------- G4 SERIES ----------------------------- */
#if defined(STM32G431xx) || defined(STM32G441xx) || defined(STM32G471xx) || \
    defined(STM32G473xx) || defined(STM32G474xx) || defined(STM32G484xx) || \
    defined(STM32G491xx) || defined(STM32G4A1xx)
  #define STM32G4_FAMILY
#endif

/* ----------------------------- H7 SERIES ----------------------------- */
#if defined(STM32H743xx) || defined(STM32H753xx) || defined(STM32H750xx) || defined(STM32H742xx) || \
    defined(STM32H745xx) || defined(STM32H755xx) || defined(STM32H747xx) || defined(STM32H757xx) || \
    defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ) || defined(STM32H7B0xx)
  #define STM32H7_FAMILY
#endif

/* ----------------------------- L0 SERIES ----------------------------- */
#if defined(STM32L010x4) || defined(STM32L010x6) || defined(STM32L010x8) || defined(STM32L010xB) || \
    defined(STM32L011xx) || defined(STM32L021xx) || defined(STM32L031xx) || defined(STM32L041xx) || \
    defined(STM32L051xx) || defined(STM32L052xx) || defined(STM32L053xx) || defined(STM32L062xx) || \
    defined(STM32L063xx) || defined(STM32L071xx) || defined(STM32L072xx) || defined(STM32L073xx)
  #define STM32L0_FAMILY
#endif

/* ----------------------------- L1 SERIES ----------------------------- */
#if defined(STM32L100xB) || defined(STM32L100xBA) || defined(STM32L151xB) || defined(STM32L151xBA) || \
    defined(STM32L151xC) || defined(STM32L151xCA) || defined(STM32L151xD) || defined(STM32L151xDX) || \
    defined(STM32L151xE) || defined(STM32L152xB) || defined(STM32L152xBA) || defined(STM32L152xC) || \
    defined(STM32L152xCA) || defined(STM32L152xD) || defined(STM32L152xDX) || defined(STM32L152xE)
  #define STM32L1_FAMILY
#endif

/* ----------------------------- L4 SERIES ----------------------------- */
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx) || \
    defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L486xx) || \
    defined(STM32L496xx) || defined(STM32L4A6xx)
  #define STM32L4_FAMILY
#endif

/* ----------------------------- L5 SERIES ----------------------------- */
#if defined(STM32L552xx) || defined(STM32L562xx)
  #define STM32L5_FAMILY
#endif

/* ----------------------------- WB SERIES ----------------------------- */
#if defined(STM32WB10xx) || defined(STM32WB15xx) || defined(STM32WB30xx) || defined(STM32WB35xx) || \
    defined(STM32WB50xx) || defined(STM32WB55xx) || defined(STM32WB5Mxx)
  #define STM32WB_FAMILY
#endif

/* ----------------------------- WL SERIES ----------------------------- */
#if defined(STM32WL54xx) || defined(STM32WL55xx) || defined(STM32WLE4xx) || defined(STM32WLE5xx)
  #define STM32WL_FAMILY
#endif

/* ----------------------------- MP1 SERIES ----------------------------- */
#if defined(STM32MP151Axx) || defined(STM32MP151Cxx) || defined(STM32MP151Dxx) || defined(STM32MP151Fxx) || \
    defined(STM32MP153Axx) || defined(STM32MP153Cxx) || defined(STM32MP153Dxx) || defined(STM32MP153Fxx) || \
    defined(STM32MP157Axx) || defined(STM32MP157Cxx) || defined(STM32MP157Dxx) || defined(STM32MP157Fxx)
  #define STM32MP1_FAMILY
#endif

#endif /* __STM32_FAMILY_H */
