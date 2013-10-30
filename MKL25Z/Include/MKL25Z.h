/**************************************************************************//**
 * @file     MKL25Z.h
 * @brief    CMSIS Cortex-M0Plus Core Peripheral Access Layer Header File for
 *           Device MKL25Z
 * @version  V3.10
 * @date     23. November 2012
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 ARM LIMITED
   Copyright (c) 2013 Ben Harris

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


#ifndef MKL25Z_H
#define MKL25Z_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup MKL25Z_Definitions MKL25Z Definitions
  This file defines all structures and symbols for MKL25Z:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup MKL25Z_CMSIS Device CMSIS Definitions
  Configuration of the Cortex-M# Processor and Core Peripherals
  @{
*/

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M0Plus Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn           = -14,      /*!<  2 Non Maskable Interrupt                        */
  HardFault_IRQn                = -13,      /*!<  3 Hard Fault Interrupt                          */
  SVCall_IRQn                   = -5,       /*!< 11 SV Call Interrupt                             */
  PendSV_IRQn                   = -2,       /*!< 14 Pend SV Interrupt                             */
  SysTick_IRQn                  = -1,       /*!< 15 System Tick Interrupt                         */

/******  Device Specific Interrupt Numbers ********************************************************/
  DMA0_IRQn                     = 0,                /*!< DMA channel 0 transfer complete interrupt */
  DMA1_IRQn                     = 1,                /*!< DMA channel 1 transfer complete interrupt */
  DMA2_IRQn                     = 2,                /*!< DMA channel 2 transfer complete interrupt */
  DMA3_IRQn                     = 3,                /*!< DMA channel 3 transfer complete interrupt */
  FTFA_IRQn                     = 5,                /*!< FTFA interrupt                           */
  LVD_LVW_IRQn                  = 6,                /*!< Low Voltage Detect, Low Voltage Warning  */
  LLW_IRQn                      = 7,                /*!< Low Leakage Wakeup                       */
  I2C0_IRQn                     = 8,                /*!< I2C0 interrupt                           */
  I2C1_IRQn                     = 9,                /*!< I2C1 interrupt                           */
  SPI0_IRQn                     = 10,               /*!< SPI0 interrupt                           */
  SPI1_IRQn                     = 11,               /*!< SPI1 interrupt                           */
  UART0_IRQn                    = 12,               /*!< UART0 status/error interrupt             */
  UART1_IRQn                    = 13,               /*!< UART1 status/error interrupt             */
  UART2_IRQn                    = 14,               /*!< UART2 status/error interrupt             */
  ADC0_IRQn                     = 15,               /*!< ADC0 interrupt                           */
  CMP0_IRQn                     = 16,               /*!< CMP0 interrupt                           */
  TPM0_IRQn                     = 17,               /*!< TPM0 fault, overflow and channels interrupt */
  TPM1_IRQn                     = 18,               /*!< TPM1 fault, overflow and channels interrupt */
  TPM2_IRQn                     = 19,               /*!< TPM2 fault, overflow and channels interrupt */
  RTC_IRQn                      = 20,               /*!< RTC interrupt                            */
  RTC_Seconds_IRQn              = 21,               /*!< RTC seconds interrupt                    */
  PIT_IRQn                      = 22,               /*!< PIT timer interrupt                      */
  USB0_IRQn                     = 24,               /*!< USB0 interrupt                           */
  DAC0_IRQn                     = 25,               /*!< DAC interrupt                            */
  TSI0_IRQn                     = 26,               /*!< TSI0 interrupt                           */
  MCG_IRQn                      = 27,               /*!< MCG interrupt                            */
  LPTimer_IRQn                  = 28,               /*!< LPTimer interrupt                        */
  PORTA_IRQn                    = 30,               /*!< Port A interrupt                         */
  PORTD_IRQn                    = 31                /*!< Port D interrupt                         */                              
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M# Processor and Core Peripherals */
/* ToDo: set the defines according your Device                                                    */
/* ToDo: define the correct core revision                                      */
#define __CM0PLUS_REV             0x0000    /*!< Core Revision r0p0                               */
#define __NVIC_PRIO_BITS          2         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */
#define __MPU_PRESENT             0         /*!< MPU present or not                               */
#define __VTOR_PRESENT            1         /*!< VTOR present or not                                */

/*@}*/ /* end of group MKL25Z_CMSIS */

#include <core_cm0plus.h>                 /* Cortex-M# processor and core peripherals           */
#include "system_MKL25Z.h"                /* MKL25Z System  include file                      */


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup MKL25Z_Peripherals MKL25Z Peripherals
  MKL25Z Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif



/* ToDo: add here your device specific peripheral access structure typedefs
         following is an example for a timer                                  */

/*------------- 16-bit Timer/Event Counter (TMR) -----------------------------*/
/** @addtogroup MKL25Z_TMR MKL25Z 16-bit Timer/Event Counter (TMR)
  @{
*/
typedef struct
{
  __IO uint32_t EN;                         /*!< Offset: 0x0000   Timer Enable Register           */
  __IO uint32_t RUN;                        /*!< Offset: 0x0004   Timer RUN Register              */
  __IO uint32_t CR;                         /*!< Offset: 0x0008   Timer Control Register          */
  __IO uint32_t MOD;                        /*!< Offset: 0x000C   Timer Mode Register             */
       uint32_t RESERVED0[1];
  __IO uint32_t ST;                         /*!< Offset: 0x0014   Timer Status Register           */
  __IO uint32_t IM;                         /*!< Offset: 0x0018   Interrupt Mask Register         */
  __IO uint32_t UC;                         /*!< Offset: 0x001C   Timer Up Counter Register       */
  __IO uint32_t RG0                         /*!< Offset: 0x0020   Timer Register                  */
       uint32_t RESERVED1[2];
  __IO uint32_t CP;                         /*!< Offset: 0x002C   Capture register                */
} <DeviceAbbreviation>_TMR_TypeDef;
/*@}*/ /* end of group MKL25Z_TMR */


#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/*@}*/ /* end of group MKL25Z_Peripherals */

/******************************************************************************/
/*                           Peripheral declaration                           */
/******************************************************************************/
/** @addtogroup MKL25Z_MemoryMap MKL25Z Memory Mapping and declaration
  @{
*/

/* Peripheral and base address */
#define MKL25Z_PERIPH_BASE      (0x40000000UL)                              /*!< (Peripheral) Base Address */

/* Shortcut macro */
#define MKL25Z_REG_BASE(name, addr) #define name ## _BASE (MKL25Z_PERIPH_BASE + addr)
#define MKL25Z_REG_PER(name, type, addr) \
    MKL25Z_REG_BASE(name, addr) \
    #define NAME ((type ## _TypeDef *) name ## _BASE)

/* Peripherals */
#include "../../peripheral/mcg.h"

/* Peripheral memory map */
// MKL25Z_REG_PER(DMA,       DMA,        0x08000)
// MKL25Z_REG_PER(GPIO_CON,  GPIO_CON,   0x0F000)
// MKL25Z_REG_PER(FLASH_CON, FLASH_CON,  0x20000)
// MKL25Z_REG_PER(DMAMUX0,   DMAMUX,     0x21000)
// MKL25Z_REG_PER(PIT,       PIT,        0x37000)
// MKL25Z_REG_PER(TPM0,      TPM,        0x38000)
// MKL25Z_REG_PER(TPM1,      TPM,        0x39000)
// MKL25Z_REG_PER(TPM2,      TPM,        0x3A000)
// MKL25Z_REG_PER(ADC0,      ADC,        0x3B000)
// MKL25Z_REG_PER(RTC,       RTC,        0x3D000)
// MKL25Z_REG_PER(DAC0,      DAC,        0x3F000)
// MKL25Z_REG_PER(LPTMR,     LPTMR,      0x40000)
// MKL25Z_REG_PER(TSI,       TSI,        0x45000)
// MKL25Z_REG_PER(SIMLP,     SIMLP,      0x47000)
// MKL25Z_REG_PER(SIM,       SIM,        0x48000)
// MKL25Z_REG_PER(PORTA,     PORT,       0x49000)
// MKL25Z_REG_PER(PORTB,     PORT,       0x4A000)
// MKL25Z_REG_PER(PORTC,     PORT,       0x4B000)
// MKL25Z_REG_PER(PORTD,     PORT,       0x4C000)
// MKL25Z_REG_PER(PORTE,     PORT,       0x4D000)
MKL25Z_REG_PER(MCG,       MCG,        0x64000)
// MKL25Z_REG_PER(OSC,       OSC,        0x65000)
// MKL25Z_REG_PER(I2C0,      I2C,        0x66000)
// MKL25Z_REG_PER(I2C1,      I2C,        0x67000)
// MKL25Z_REG_PER(UART0,     UART,       0x6A000)
// MKL25Z_REG_PER(UART1,     UART,       0x6B000)
// MKL25Z_REG_PER(UART2,     UART,       0x6C000)
// MKL25Z_REG_PER(USB,       USB,        0x72000)
// MKL25Z_REG_PER(CMP,       CMP,        0x73000)
// MKL25Z_REG_PER(SPI0,      SPI,        0x76000)
// MKL25Z_REG_PER(SPI1,      SPI,        0x77000)
// MKL25Z_REG_PER(LLWU,      LLWU,       0x7C000)
// MKL25Z_REG_PER(PMC,       PMC,        0x7D000)
// MKL25Z_REG_PER(SMC,       SMC,        0x7E000)
// MKL25Z_REG_PER(RCM,       RCM,        0x7F000)
/*@}*/ /* end of group MKL25Z_MemoryMap */

/*@}*/ /* end of group MKL25Z_Definitions */

#ifdef __cplusplus
}
#endif

#endif  /* MKL25Z_H */
