/**************************************************************************//**
 * @file     gpio.h
 * @brief    GPIO and Port Control header file for Kinetis devices
 * @version  V0.1
 * @date     30. October 2013
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2013 Ben Harris

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
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
#ifndef __FREESC_GPIO_H
#define __FREESC_GPIO_H

struct PORT_PCR_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO uint32_t PS        : 1;
      __IO uint32_t PE        : 1;
      __IO uint32_t SRE       : 1;
           uint32_t RESERVED0 : 1;
      __IO uint32_t PFE       : 1;
           uint32_t RESERVED1 : 1;
      __IO uint32_t DSE       : 1;
           uint32_t RESERVED2 : 1;
      __IO enum {
        PORT_MUX_DISABLED     = 0,
        PORT_MUX_ALT1         = 1,
        PORT_MUX_ALT2         = 2,
        PORT_MUX_ALT3         = 3,
        PORT_MUX_ALT4         = 4,
        PORT_MUX_ALT5         = 5,
        PORT_MUX_ALT6         = 6,
        PORT_MUX_ALT7         = 7
      } MUX                   : 3;
           uint32_t RESERVED3 : 5;
      __IO enum {
        PORT_IRQC_DISABLE     = 0,
        PORT_IRQC_DMA_RISING  = 1,
        PORT_IRQC_DMA_FALLING = 2,
        PORT_IRQC_DMA_EDGE    = 3,
        PORT_IRQC_INT_ZERO    = 8,
        PORT_IRQC_INT_RISING  = 9,
        PORT_IRQC_INT_FALLING = 10,
        PORT_IRQC_INT_EDGE    = 11,
        PORT_IRQC_INT_ONE     = 12
      } IRQC                  : 4;
           uint32_t RESERVED4 : 4;
      __IO uint32_t ISF       : 1;
           uint32_t RESERVED5 : 7;
    };
  };
};

struct PORT_GPCR_REG {
  union {
    __O  uint32_t             all;
    struct {
      __O  uint32_t GPWD      : 16;
      __O  uint32_t GPWE      : 16;
    };
  };
};

typedef struct {
  __IO uint32_t PDOR;               /*!< Offset: 0x0000   Port Data Output Register */
  __O  uint32_t PSOR;               /*!< Offset: 0x0004   Port Set Output Register */
  __O  uint32_t PCOR;               /*!< Offset: 0x0008   Port Clear Output Register */
  __O  uint32_t PTOR;               /*!< Offset: 0x000C   Port Toggle Output Register */
  __I  uint32_t PDIR;               /*!< Offset: 0x0010   Port Data Input Register */
  __IO uint32_t PDDR;               /*!< Offset: 0x0014   Port Data Direction Register */
} GPIO_TypeDef;

typedef struct {
       struct PORT_PCR_REG PCR[32]; /*!< Offset: 0x0000   Pin Control Register */
       struct PORT_GPCR_REG GPCLR;  /*!< Offset: 0x0080   Global Pin Control Low Register */
       struct PORT_GPCR_REG GPCHR;  /*!< Offset: 0x0084   Global Pin Control High Register */
       uint32_t RESERVED0[6];
  __IO uint32_t ISFR;               /*!< Offset: 0x00A0   Interrupt Status Flag Register */
} PORT_TypeDef;

#endif