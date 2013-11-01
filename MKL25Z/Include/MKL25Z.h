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
#if 0
}
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

/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif

#include "../../peripheral/sim.h"
#include "../../peripheral/mcg.h"
#include "../../peripheral/pit.h"
#include "../../peripheral/gpio.h"

/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif

/*@}*/ /* end of group MKL25Z_Peripherals */

/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup MKL25Z_MemoryMap MKL25Z Memory Mapping
  @{
*/

/* Peripheral and base address */
#define MKL25Z_PERIPH_BASE      (0x40000000UL)  /*!< (Peripheral) Base Address */

/* Peripheral memory map */
#define DMA_BASE (MKL25Z_PERIPH_BASE +        0x08000)
#define GPIOA_BASE (MKL25Z_PERIPH_BASE +      0x0F000)
#define GPIOB_BASE (MKL25Z_PERIPH_BASE +      0x0F040)
#define GPIOC_BASE (MKL25Z_PERIPH_BASE +      0x0F080)
#define GPIOD_BASE (MKL25Z_PERIPH_BASE +      0x0F0C0)
#define GPIOE_BASE (MKL25Z_PERIPH_BASE +      0x0F100)
#define FLASH_CON_BASE (MKL25Z_PERIPH_BASE +  0x20000)
#define DMAMUX0_BASE (MKL25Z_PERIPH_BASE +    0x21000)
#define PIT_BASE (MKL25Z_PERIPH_BASE +        0x37000)
#define TPM0_BASE (MKL25Z_PERIPH_BASE +       0x38000)
#define TPM1_BASE (MKL25Z_PERIPH_BASE +       0x39000)
#define TPM2_BASE (MKL25Z_PERIPH_BASE +       0x3A000)
#define ADC0_BASE (MKL25Z_PERIPH_BASE +       0x3B000)
#define RTC_BASE (MKL25Z_PERIPH_BASE +        0x3D000)
#define DAC0_BASE (MKL25Z_PERIPH_BASE +       0x3F000)
#define LPTMR_BASE (MKL25Z_PERIPH_BASE +      0x40000)
#define TSI_BASE (MKL25Z_PERIPH_BASE +        0x45000)
#define SIMLP_BASE (MKL25Z_PERIPH_BASE +      0x47000)
#define SIM_BASE (MKL25Z_PERIPH_BASE +        0x48000)
#define PORTA_BASE (MKL25Z_PERIPH_BASE +      0x49000)
#define PORTB_BASE (MKL25Z_PERIPH_BASE +      0x4A000)
#define PORTC_BASE (MKL25Z_PERIPH_BASE +      0x4B000)
#define PORTD_BASE (MKL25Z_PERIPH_BASE +      0x4C000)
#define PORTE_BASE (MKL25Z_PERIPH_BASE +      0x4D000)
#define MCG_BASE (MKL25Z_PERIPH_BASE +        0x64000)
#define OSC_BASE (MKL25Z_PERIPH_BASE +        0x65000)
#define I2C0_BASE (MKL25Z_PERIPH_BASE +       0x66000)
#define I2C1_BASE (MKL25Z_PERIPH_BASE +       0x67000)
#define UART0_BASE (MKL25Z_PERIPH_BASE +      0x6A000)
#define UART1_BASE (MKL25Z_PERIPH_BASE +      0x6B000)
#define UART2_BASE (MKL25Z_PERIPH_BASE +      0x6C000)
#define USB_BASE (MKL25Z_PERIPH_BASE +        0x72000)
#define CMP_BASE (MKL25Z_PERIPH_BASE +        0x73000)
#define SPI0_BASE (MKL25Z_PERIPH_BASE +       0x76000)
#define SPI1_BASE (MKL25Z_PERIPH_BASE +       0x77000)
#define LLWU_BASE (MKL25Z_PERIPH_BASE +       0x7C000)
#define PMC_BASE (MKL25Z_PERIPH_BASE +        0x7D000)
#define SMC_BASE (MKL25Z_PERIPH_BASE +        0x7E000)
#define RCM_BASE (MKL25Z_PERIPH_BASE +        0x7F000)
#define FGPIOA_BASE (                      0xF80FF000)
#define FGPIOB_BASE (                      0xF80FF040)
#define FGPIOC_BASE (                      0xF80FF080)
#define FGPIOD_BASE (                      0xF80FF0C0)
#define FGPIOE_BASE (                      0xF80FF100)
/*@}*/ /* end of group MKL25Z_MemoryMap */

/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
/** @addtogroup MKL25Z_PeripheralDecl MKL25Z Peripheral Declaration
  @{
*/

// #define DMA         (( DMA_TypeDef *)         DMA_BASE)
#define GPIOA       (( GPIO_TypeDef *)        GPIOA_BASE)
#define GPIOB       (( GPIO_TypeDef *)        GPIOB_BASE)
#define GPIOC       (( GPIO_TypeDef *)        GPIOC_BASE)
#define GPIOD       (( GPIO_TypeDef *)        GPIOD_BASE)
#define GPIOE       (( GPIO_TypeDef *)        GPIOE_BASE)
// #define FLASH_CON   (( FLASH_CON_TypeDef *)   FLASH_CON_BASE)
// #define DMAMUX0     (( DMAMUX_TypeDef *)      DMAMUX0_BASE)
#define PIT         (( PIT_TypeDef *)         PIT_BASE)
// #define TPM0        (( TPM_TypeDef *)         TPM0_BASE)
// #define TPM1        (( TPM_TypeDef *)         TPM1_BASE)
// #define TPM2        (( TPM_TypeDef *)         TPM2_BASE)
// #define ADC0        (( ADC_TypeDef *)         ADC0_BASE)
// #define RTC         (( RTC_TypeDef *)         RTC_BASE)
// #define DAC0        (( DAC_TypeDef *)         DAC0_BASE)
// #define LPTMR       (( LPTMR_TypeDef *)       LPTMR_BASE)
// #define TSI         (( TSI_TypeDef *)         TSI_BASE)
#define SIMLP       (( SIMLP_TypeDef *)       SIMLP_BASE)
#define SIM         (( SIM_TypeDef *)         SIM_BASE)
#define PORTA       (( PORT_TypeDef *)        PORTA_BASE)
#define PORTB       (( PORT_TypeDef *)        PORTB_BASE)
#define PORTC       (( PORT_TypeDef *)        PORTC_BASE)
#define PORTD       (( PORT_TypeDef *)        PORTD_BASE)
#define PORTE       (( PORT_TypeDef *)        PORTE_BASE)
#define MCG         (( MCG_TypeDef *)         MCG_BASE)
// #define OSC         (( OSC_TypeDef *)         OSC_BASE)
// #define I2C0        (( I2C_TypeDef *)         I2C0_BASE)
// #define I2C1        (( I2C_TypeDef *)         I2C1_BASE)
// #define UART0       (( UART_TypeDef *)        UART0_BASE)
// #define UART1       (( UART_TypeDef *)        UART1_BASE)
// #define UART2       (( UART_TypeDef *)        UART2_BASE)
// #define USB         (( USB_TypeDef *)         USB_BASE)
// #define CMP         (( CMP_TypeDef *)         CMP_BASE)
// #define SPI0        (( SPI_TypeDef *)         SPI0_BASE)
// #define SPI1        (( SPI_TypeDef *)         SPI1_BASE)
// #define LLWU        (( LLWU_TypeDef *)        LLWU_BASE)
// #define PMC         (( PMC_TypeDef *)         PMC_BASE)
// #define SMC         (( SMC_TypeDef *)         SMC_BASE)
#define FGPIOA      (( GPIO_TypeDef *)        FGPIOA_BASE)
#define FGPIOB      (( GPIO_TypeDef *)        FGPIOB_BASE)
#define FGPIOC      (( GPIO_TypeDef *)        FGPIOC_BASE)
#define FGPIOD      (( GPIO_TypeDef *)        FGPIOD_BASE)
#define FGPIOE      (( GPIO_TypeDef *)        FGPIOE_BASE)

/*@}*/ /* end of group MKL25Z_PeripheralDecl */
    
/*@}*/ /* end of group MKL25Z_Definitions */

#ifdef __cplusplus
}
#endif

#endif  /* MKL25Z_H */
