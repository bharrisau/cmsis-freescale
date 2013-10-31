/**************************************************************************//**
 * @file     pit.h
 * @brief    Periodic interrupt timer header file for Kinetis devices
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
#ifndef __FREESC_PIT_H
#define __FREESC_PIT_H

struct PIT_MCR_REG {
  union {
    __IO uint32_t             all;
    struct {
           uint32_t RESERVED0 : 29;
      __IO uint32_t CHN       : 1;
      __IO uint32_t TIE       : 1;
      __IO uint32_t TEN       : 1;
    };
  };
};

struct PIT_TCTRL_REG {
  union {
    __IO uint32_t             all;
    struct {
           uint32_t RESERVED0 : 30;
      __IO uint32_t MDIS      : 1;
      __IO uint32_t FRZ       : 1;
    };
  };
};

struct PIT_TFLG_REG {
  union {
    __IO uint32_t             all;
    struct {
           uint32_t RESERVED0 : 31;
      __IO uint32_t TIF       : 1;
    };
  };
};

struct PIT_TMR {
  __IO uint32_t LDVAL;                /*!< Offset: 0x0100   Timer Load Value Register */
  __I  uint32_t CVAL;                 /*!< Offset: 0x0104   Current Timer Value Register */
       struct PIT_TCTRL_REG TCTRL;    /*!< Offset: 0x0108   Timer Control Register */
       struct PIT_TFLG_REG TFLG;      /*!< Offset: 0x010C   Timer Flag Register    */
};

typedef struct {
       struct PIT_MCR_REG MCR;        /*!< Offset: 0x0000   PIT Module Control Register */
       uint32_t RESERVED0[55];
  __I  uint32_t LTMR64H;              /*!< Offset: 0x00E0   PIT Upper Lifetime Timer Register */
  __I  uint32_t LTMR64L;              /*!< Offset: 0x00E4   PIT Lower Lifetime Timer Register */
       uint32_t RESERVED1[6];
       struct PIT_TMR TIMER[2];       /*!< Offset: 0x0100   Timer Load Value Register */
} PIT_TypeDef;

#endif